"""Camera supervision and immutable observations; no video/AI imports here.

Only this receiver thread touches the IPC socket. Consumers read a local copy,
never a cross-process lock. Unix datagrams bound messages and avoid partial reads.
"""

from dataclasses import asdict, dataclass
import copy
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time


@dataclass
class CameraConfig:
    enabled: bool = True
    no_detection: bool = False
    device: str = '/dev/video0'
    format: str = 'mjpeg'
    width: int = 640
    height: int = 480
    capture_width: int = 640
    capture_height: int = 480
    fps: int = 30
    detection_fps: int = 15
    threads: int = 2
    encoder_threads: int = 1
    model: str = str(Path(__file__).resolve().parents[1] / 'models/yolo26n_ncnn_model')
    confidence: float = .35
    iou: float = .45
    max_box_age: float = .5
    max_frame_age: float = .2
    bitrate: int = 1200
    host: str = '0.0.0.0'
    advertised_host: str = ''
    port: int = 8554
    path: str = '/camera'
    test_pattern: bool = False
    image: str | None = None


def add_arguments(parser):
    group = parser.add_argument_group('camera and person detection')
    group.add_argument('--no-camera', action='store_true', help='Disable capture, RTSP and inference')
    group.add_argument('--no-detection', action='store_true', help='Stream video without running inference')
    group.add_argument('--camera-device', help='V4L2 capture node; defaults to a sole stable USB path or /dev/video0')
    group.add_argument('--camera-format', choices=('mjpeg', 'yuyv'), default='mjpeg')
    group.add_argument('--camera-width', type=int, default=640, help='Output width')
    group.add_argument('--camera-height', type=int, default=480, help='Output height')
    group.add_argument('--capture-width', type=int)
    group.add_argument('--capture-height', type=int)
    group.add_argument('--camera-fps', type=int, default=30)
    group.add_argument('--detection-fps', type=int, default=15)
    group.add_argument('--detection-threads', type=int, default=2)
    group.add_argument('--encoder-threads', type=int, default=1)
    group.add_argument('--detection-model', default=CameraConfig.model)
    group.add_argument('--confidence', type=float, default=.35)
    group.add_argument('--rtsp-host', default='0.0.0.0', help='Listener address')
    group.add_argument('--stream-host', default='', help='LAN address/hostname advertised in the ffplay command')
    group.add_argument('--rtsp-port', type=int, default=8554)
    group.add_argument('--rtsp-path', default='/camera')
    sources = group.add_mutually_exclusive_group()
    sources.add_argument('--camera-test-pattern', action='store_true', help='Synthetic source; no USB camera opened')
    sources.add_argument('--camera-image', help='Loop a still image for detection/stream verification')


def config_from_arguments(args, parser):
    import math
    for name, low, high in (
        ('camera_width', 16, 4096), ('camera_height', 16, 4096),
        ('capture_width', 16, 4096), ('capture_height', 16, 4096),
        ('camera_fps', 1, 60), ('detection_fps', 1, 60),
        ('detection_threads', 1, 4), ('encoder_threads', 1, 4), ('rtsp_port', 1, 65535),
    ):
        value = getattr(args, name)
        if value is not None and not low <= value <= high:
            parser.error(f'--{name.replace("_", "-")} must be between {low} and {high}')
    if args.camera_width % 2 or args.camera_height % 2:
        parser.error('Camera output dimensions must be even for H.264')
    if not math.isfinite(args.confidence) or not 0 < args.confidence <= 1:
        parser.error('--confidence must be in (0, 1]')
    if not args.rtsp_path.startswith('/') or any(not (c.isascii() and (c.isalnum() or c in '/-._~%')) for c in args.rtsp_path):
        parser.error('--rtsp-path must start with / and use letters, digits, /, -, ., _, ~ or percent escapes')
    if not args.rtsp_host:
        parser.error('--rtsp-host must not be empty')
    for host in (args.rtsp_host, args.stream_host):
        if any(not (c.isalnum() or c in '.-_:[]%') for c in host):
            parser.error('RTSP/stream host must be an IP address or hostname')
    if args.stream_host in ('0.0.0.0', '::', '[::]'):
        parser.error('--stream-host must be a reachable address, not a wildcard listener')
    if args.camera_image and not Path(args.camera_image).is_file():
        parser.error('--camera-image does not exist')
    stable = sorted(Path('/dev/v4l/by-id').glob('*video-index0'))
    device = args.camera_device or (str(stable[0]) if len(stable) == 1 else '/dev/video0')
    return CameraConfig(
        enabled=not (args.no_camera or args.controller_only), no_detection=args.no_detection,
        device=device, format=args.camera_format, width=args.camera_width, height=args.camera_height,
        capture_width=args.capture_width or args.camera_width, capture_height=args.capture_height or args.camera_height,
        fps=args.camera_fps, detection_fps=args.detection_fps, threads=args.detection_threads,
        encoder_threads=args.encoder_threads, model=str(Path(args.detection_model).resolve()),
        confidence=args.confidence, host=args.rtsp_host, advertised_host=args.stream_host,
        port=args.rtsp_port, path=args.rtsp_path, test_pattern=args.camera_test_pattern,
        image=str(Path(args.camera_image).resolve()) if args.camera_image else None,
    )


def playback(config):
    host = config.advertised_host or config.host
    if host in ('0.0.0.0', '::', '[::]'):
        try:
            # Route lookup only: UDP connect sends no packet and does not resolve DNS.
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as route:
                route.connect(('192.0.2.1', 9))
                host = route.getsockname()[0]
        except OSError:
            host = socket.gethostname().removesuffix('.local') + '.local'
    if ':' in host and not host.startswith('['):
        host = f'[{host}]'
    url = f'rtsp://{host}:{config.port}{config.path}'
    command = ('ffplay -rtsp_transport tcp -fflags nobuffer -flags low_delay '
               '-analyzeduration 0 -probesize 32 -threads 1 -framedrop -sync ext '
               f'"{url}"')
    return url, command


class CameraMonitor:
    def __init__(self, config):
        self.config = config
        self.lock = threading.Lock()
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self.run, name='camera-supervisor', daemon=True)
        self.process = None
        self.channel = None
        self.desired_detection = not config.no_detection
        self.revision = 0
        self.generation = 0
        self.updated_at = None
        self.state = dict(state='starting' if config.enabled else 'disabled', stream_ready=False,
                          detection_state='loading' if self.desired_detection and config.enabled else 'disabled',
                          boxes=[], observation=None, last_error=None, restarts=0)
        self.url, self.command = playback(config)

    def start(self):
        self.thread.start()

    def set_detection(self, enabled):
        with self.lock:
            if not self.config.enabled or self.desired_detection == bool(enabled):
                return
            self.desired_detection = bool(enabled)
            self.revision += 1
            self.state.update(boxes=[], observation=None,
                              detection_state='loading' if enabled else 'disabled')

    def toggle_detection(self):
        with self.lock:
            enabled = not self.desired_detection
        self.set_detection(enabled)

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            result = copy.deepcopy(self.state)
            result.update(enabled=self.config.enabled, detection_enabled=self.desired_detection,
                          generation=self.generation, rtsp_url=self.url, ffplay_command=self.command,
                          worker_age_s=None if self.updated_at is None else max(0, now - self.updated_at),
                          requested_revision=self.revision)
        age = result['worker_age_s']
        observation = result.get('observation')
        frame_time = result.get('last_frame_at')
        result['frame_age_s'] = max(0, now - frame_time) if frame_time else None
        if age is None or age > 1 or not frame_time or now - frame_time > 1:
            result['stream_ready'] = False
            if result['state'] == 'streaming':
                result['state'] = 'stale'
        valid = (result['stream_ready'] and result['detection_enabled']
                 and result.get('revision') == result['requested_revision']
                 and result['detection_state'] == 'ready' and observation is not None
                 and 0 <= now - observation['captured_at'] <= self.config.max_box_age)
        result['observation_age_s'] = max(0, now - observation['captured_at']) if observation else None
        result['observation_valid'] = bool(valid)
        # Expired boxes must never look like a current "zero people" observation.
        result['boxes'] = observation['boxes'] if valid else []
        result['people'] = len(result['boxes']) if valid else None
        result['ready'] = result['stream_ready'] and (not result['detection_enabled'] or valid)
        return result

    def close(self):
        self.stopping.set()
        if self.thread.ident is not None:
            self.thread.join(timeout=5)

    def _send(self, message):
        try:
            self.channel.send(json.dumps(message, allow_nan=False).encode())
        except OSError:
            pass

    def _cleanup(self):
        process = self.process
        if not process:
            return
        # The process group includes the detector and multiprocessing resource tracker.
        try:
            self._send({'op': 'stop'})
            process.wait(timeout=1)
        except (subprocess.TimeoutExpired, OSError):
            pass
        for sig in (signal.SIGTERM, signal.SIGKILL):
            try:
                os.killpg(process.pid, sig)
            except ProcessLookupError:
                break
            try:
                process.wait(timeout=.5)
            except subprocess.TimeoutExpired:
                pass
        process.wait(timeout=1)
        self.channel.close()
        self.channel = self.process = None

    def run(self):
        try:
            if not self.config.enabled:
                return
            for attempt in range(4):
                if self.stopping.is_set():
                    break
                self.channel, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
                self.channel.setblocking(False)
                with self.lock:
                    self.generation += 1
                    self.updated_at = None
                    self.state.update(state='starting', stream_ready=False, boxes=[], observation=None,
                                      restarts=attempt)
                try:
                    worker_config = asdict(self.config)
                    with self.lock:
                        worker_config['no_detection'] = not self.desired_detection
                    self.process = subprocess.Popen(
                        [sys.executable, '-m', 'apptirover.vision.worker', str(child.fileno()),
                         json.dumps(worker_config), str(os.getpid())],
                        pass_fds=(child.fileno(),), start_new_session=True,
                        stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                        cwd=str(Path(__file__).resolve().parents[1]),
                    )
                finally:
                    child.close()
                started = last_received = time.monotonic()
                next_address = 0
                while not self.stopping.wait(.05):
                    now = time.monotonic()
                    with self.lock:
                        command = dict(op='detection', enabled=self.desired_detection, revision=self.revision)
                    self._send(command)
                    # Bounded drain: malformed/flooding worker cannot consume a whole CPU.
                    for _ in range(8):
                        try:
                            packet = json.loads(self.channel.recv(32768))
                        except OSError:
                            break
                        if not isinstance(packet, dict) or packet.get('type') != 'status':
                            continue
                        last_received = now
                        with self.lock:
                            if packet.get('observation'):
                                packet['observation']['process_generation'] = self.generation
                            self.state.update(packet)
                            self.updated_at = now
                    if now >= next_address:
                        url, command_line = playback(self.config)
                        with self.lock:
                            self.url, self.command = url, command_line
                        next_address = now + 5
                    if self.process.poll() is not None:
                        raise_error = f'Camera process exited ({self.process.returncode})'
                        break
                    if now - last_received > (15 if last_received == started else 3):
                        raise_error = 'Camera process stopped responding'
                        break
                else:
                    raise_error = None
                with self.lock:
                    self.state.update(state='stopped' if self.stopping.is_set() else 'failed',
                                      stream_ready=False, boxes=[], observation=None)
                    if not self.stopping.is_set():
                        self.state['last_error'] = self.state.get('last_error') or raise_error
                        self.state['previous_error'] = self.state['last_error']
                self._cleanup()
                if self.stopping.wait(2 ** attempt):
                    break
        except Exception as exc:
            with self.lock:
                self.state.update(state='failed', stream_ready=False, boxes=[], observation=None,
                                  last_error=f'Camera supervisor: {type(exc).__name__}: {exc}')
        finally:
            self._cleanup()
            if self.channel:
                self.channel.close()
