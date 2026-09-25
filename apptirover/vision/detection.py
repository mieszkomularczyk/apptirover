"""NCNN inference in a separate process, with one shared latest-frame slot.

Only the small inference image crosses into Python/shared memory. Inference is
isolated in a process because the NCNN Python binding may hold the GIL.
"""

import multiprocessing as mp
import json
import select
import socket
import os
import signal
import time
import traceback
from pathlib import Path

import cv2
import numpy as np
import yaml


def read_metadata(directory):
    directory = Path(directory)
    for filename in ("model.ncnn.param", "model.ncnn.bin", "metadata.yaml"):
        if not (directory / filename).is_file():
            raise FileNotFoundError(f"Missing {directory / filename}. Run tools/export_model.py with the export venv first.")
    metadata = yaml.safe_load((directory / "metadata.yaml").read_text())
    if metadata.get("task") != "detect":
        raise ValueError("Only YOLO object detection models are supported")
    height, width = metadata["imgsz"]
    if height != width or width < 32:
        raise ValueError("Export a square model input, for example imgsz=320")
    names = metadata["names"]
    if isinstance(names, dict):
        names = [names.get(i, names.get(str(i))) for i in range(len(names))]
    if not names or not all(isinstance(name, str) for name in names):
        raise ValueError("Model metadata must provide contiguous class names")
    return int(width), names


def fit_size(width, height, size):
    scale = min(size / width, size / height)
    return max(1, round(width * scale)), max(1, round(height * scale))


def letterbox(rgb, size):
    height, width = rgb.shape[:2]
    new_width, new_height = fit_size(width, height, size)
    if (width, height) != (new_width, new_height):
        rgb = cv2.resize(rgb, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
    left = (size - new_width) // 2
    top = (size - new_height) // 2
    padded = cv2.copyMakeBorder(rgb, top, size - new_height - top, left, size - new_width - left,
                                cv2.BORDER_CONSTANT, value=(114, 114, 114))
    return padded, (new_width, new_height, left, top)


def decode_predictions(output, geometry, class_count, confidence=0.35, iou=0.45, class_ids=None):
    """Return normalized (x1,y1,x2,y2,score,class) rows, undoing letterboxing.

    Supports the usual NCNN YOLO head (4+classes, anchors), plus an exported
    end-to-end head (detections, 6). Invalid/non-finite rows are discarded.
    """
    output = np.asarray(output, dtype=np.float32).squeeze()
    if output.ndim == 1:
        output = output[None, :]
    if output.ndim != 2:
        raise ValueError(f"Unexpected model output shape: {output.shape}")
    if output.shape[-1] == 6:
        rows = output.copy()
        rows = rows[np.isfinite(rows).all(axis=1) & (rows[:, 4] >= confidence)]
        if class_ids is not None:
            rows = rows[np.isin(rows[:, 5], class_ids)]
    else:
        if output.shape[0] == 4 + class_count:
            output = output.T
        if output.shape[1] != 4 + class_count:
            raise ValueError(f"Unsupported YOLO head shape: {output.shape}")
        output = output[np.isfinite(output).all(axis=1)]
        classes = output[:, 4:].argmax(axis=1)
        scores = output[np.arange(len(output)), classes + 4]
        selected = scores >= confidence
        if class_ids is not None:
            selected &= np.isin(classes, class_ids)
        boxes, scores, classes = output[selected, :4].copy(), scores[selected], classes[selected]
        boxes[:, :2] -= boxes[:, 2:] / 2
        # Class-aware native NMS: overlapping objects of different classes survive.
        selected = cv2.dnn.NMSBoxesBatched(boxes.tolist(), scores.tolist(), classes.tolist(), confidence, iou)
        selected = np.asarray(selected, dtype=int).reshape(-1)
        boxes, scores, classes = boxes[selected], scores[selected], classes[selected]
        boxes[:, 2:] += boxes[:, :2]
        rows = np.column_stack((boxes, scores, classes))
    if len(rows) == 0:
        return []
    width, height, left, top = geometry
    rows[:, [0, 2]] = np.clip((rows[:, [0, 2]] - left) / width, 0, 1)
    rows[:, [1, 3]] = np.clip((rows[:, [1, 3]] - top) / height, 0, 1)
    valid = ((rows[:, 2] > rows[:, 0]) & (rows[:, 3] > rows[:, 1])
             & (rows[:, 5] >= 0) & (rows[:, 5] < class_count) & (rows[:, 5] == np.floor(rows[:, 5])))
    return rows[valid].tolist()


class Detector:
    def __init__(self, directory, threads=2, confidence=0.35, iou=0.45):
        import ncnn
        self.ncnn = ncnn
        self.size, self.names = read_metadata(directory)
        if "person" not in self.names:
            raise ValueError("The model must contain a 'person' class")
        self.class_ids = (self.names.index("person"),)
        self.confidence, self.iou = confidence, iou
        cv2.setNumThreads(1)
        self.net = ncnn.Net()
        self.net.opt.use_vulkan_compute = False
        self.net.opt.num_threads = threads
        self.net.opt.openmp_blocktime = 0
        if self.net.load_param(str(Path(directory) / "model.ncnn.param")) != 0:
            raise RuntimeError("NCNN could not load model parameters")
        if self.net.load_model(str(Path(directory) / "model.ncnn.bin")) != 0:
            raise RuntimeError("NCNN could not load model weights")
        self.input_name = self.net.input_names()[0]
        self.output_name = self.net.output_names()[0]

    def predict(self, rgb):
        padded, geometry = letterbox(rgb, self.size)
        mat = self.ncnn.Mat.from_pixels(padded, self.ncnn.Mat.PixelType.PIXEL_RGB, self.size, self.size)
        mat.substract_mean_normalize([], [1 / 255.0] * 3)
        with self.net.create_extractor() as extractor:
            if extractor.input(self.input_name, mat) != 0:
                raise RuntimeError("NCNN rejected the input tensor")
            status, output = extractor.extract(self.output_name)
            if status != 0:
                raise RuntimeError(f"NCNN inference failed: {status}")
            return decode_predictions(np.array(output), geometry, len(self.names), self.confidence, self.iou,
                                      class_ids=self.class_ids)


def send(channel, message):
    try:
        channel.send(json.dumps(message, allow_nan=False).encode())
    except OSError:
        pass


def _worker(model, threads, confidence, iou, pixels, shape, lock,
            timestamp, sequence, generation, channel, parent_pid):
    from .lifecycle import die_with_parent
    die_with_parent(parent_pid)
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    channel.setblocking(False)
    try:
        detector = Detector(model, threads, confidence, iou)
        detector.predict(np.zeros(shape, dtype=np.uint8))
        send(channel, dict(type='ready'))
        shared = np.frombuffer(pixels, dtype=np.uint8).reshape(shape)
        completed_count = 0
        while True:
            if not select.select([channel], [], [], .2)[0]:
                continue
            for _ in range(64):
                try:
                    message = channel.recv(128)
                except BlockingIOError:
                    break
                if message == b'stop':
                    return
            if not lock.acquire(block=False):
                continue
            try:
                rgb = shared.copy()
                frame_time, frame_id, camera_generation = timestamp.value, sequence.value, generation.value
            finally:
                lock.release()
            started = time.monotonic()
            boxes = detector.predict(rgb)[:64]
            completed = time.monotonic()
            completed_count += 1
            send(channel, dict(type='result', captured_at=frame_time, completed_at=completed,
                               frame_sequence=frame_id, camera_generation=camera_generation,
                               boxes=boxes, completed_count=completed_count,
                               inference_ms=(completed - started) * 1000))
    except Exception:
        send(channel, dict(type='error', error=traceback.format_exc()[-2000:]))
    finally:
        channel.close()


class DetectionWorker:
    """Asynchronous model lifecycle. No start/join wait runs in video callbacks."""

    def __init__(self, args):
        # Fixed small feed size; Detector letterboxes this to the exported model size.
        self.width, self.height = fit_size(args.width, args.height, 320)
        self.shape = (self.height, self.width, 3)
        ctx = mp.get_context("spawn")
        self.pixels = ctx.RawArray("B", self.width * self.height * 3)
        self.array = np.frombuffer(self.pixels, dtype=np.uint8).reshape(self.shape)
        self.lock = ctx.Lock()
        self.timestamp = ctx.Value("d", 0, lock=False)
        self.sequence = ctx.Value("Q", 0, lock=False)
        self.generation = ctx.Value("Q", 0, lock=False)
        self.channel, self.child_channel = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.channel.setblocking(False)
        self.error = None
        self.process = ctx.Process(target=_worker, name="apptirover-inference", args=(
            str(args.model), args.threads, args.confidence, args.iou, self.pixels, self.shape,
            self.lock, self.timestamp, self.sequence, self.generation, self.child_channel, os.getpid()))
        self.ready = False
        self.started_at = None
        self.stop_at = None
        self.last_result_at = None
        self.first_submit_at = None
        self.closed = False

    def start(self):
        self.process.start()
        self.child_channel.close()
        self.started_at = time.monotonic()

    def submit(self, rgb, frame_time, sequence, generation):
        if not self.ready or self.stop_at is not None or not self.lock.acquire(block=False):
            return
        try:
            np.copyto(self.array, rgb)
            self.timestamp.value = frame_time
            self.sequence.value = sequence
            self.generation.value = generation
            try:
                self.channel.send(b'frame')
            except OSError:
                pass
            if self.first_submit_at is None:
                self.first_submit_at = time.monotonic()
        finally:
            self.lock.release()

    def latest(self):
        latest = None
        for _ in range(16):
            try:
                packet = json.loads(self.channel.recv(32768))
            except OSError:
                break
            if packet['type'] == 'result':
                latest = packet
                self.last_result_at = time.monotonic()
                self.ready = True
            elif packet['type'] == 'ready':
                self.ready = True
            elif packet['type'] == 'error':
                self.error = packet['error']
        return latest

    def check(self):
        if self.error:
            raise RuntimeError(self.error)
        if not self.process.is_alive():
            raise RuntimeError(f"Inference worker exited with code {self.process.exitcode}")
        if not self.ready and time.monotonic() - self.started_at > 45:
            raise RuntimeError("Inference model startup timed out")
        progress = self.last_result_at or self.first_submit_at
        if self.ready and progress and time.monotonic() - progress > 3:
            raise RuntimeError("Inference stopped producing results")

    def begin_stop(self):
        self.ready = False
        if self.stop_at is None:
            self.stop_at = time.monotonic()
            try:
                self.channel.send(b'stop')
            except OSError:
                pass

    def poll_stop(self):
        if self.process.pid is not None:
            self.process.join(timeout=0)
            if self.process.is_alive():
                elapsed = time.monotonic() - self.stop_at
                if elapsed > 2:
                    self.process.kill()
                elif elapsed > 1:
                    self.process.terminate()
                return False
        if not self.closed:
            self.channel.close()
            self.child_channel.close()
            self.process.close()
            self.closed = True
        return True

    def close(self):
        self.begin_stop()
        deadline = time.monotonic() + 3
        while not self.poll_stop() and time.monotonic() < deadline:
            time.sleep(.02)
