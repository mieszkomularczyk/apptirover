"""Native GStreamer video pipeline with a shared RTSP stream."""

import faulthandler
from collections import deque
import logging
import json
import signal
import threading
import time

import numpy as np

from .native import load_bindings

gi = load_bindings()

gi.require_version("Gst", "1.0")
gi.require_version("GstApp", "1.0")
gi.require_version("GstVideo", "1.0")
gi.require_version("GstRtspServer", "1.0")
gi.require_foreign("cairo")
from gi.repository import GLib, Gst, GstRtspServer, GstVideo  # noqa: E402

from .detection import DetectionWorker, fit_size  # noqa: E402

LOG = logging.getLogger(__name__)


def quote(value):
    """Quote a GStreamer property value (no shell is involved)."""
    return '"' + str(value).replace("\\", "\\\\").replace('"', '\\"') + '"'


class StreamFactory(GstRtspServer.RTSPMediaFactory):
    """Relay references to encoded buffers; all viewers share one encoder."""

    def __init__(self, fps):
        super().__init__()
        self.lock = threading.Lock()
        self.source = None
        self.waiting_keyframe = True
        self.request_keyframe = lambda: False
        self.set_shared(True)
        self.set_latency(0)
        self.set_launch(
            '( appsrc name=video is-live=true format=time do-timestamp=true block=false '
            'min-latency=0 max-latency=0 max-buffers=1 leaky-type=downstream max-bytes=0 max-time=0 '
            f'caps="video/x-h264,stream-format=byte-stream,alignment=au,framerate={fps}/1" '
            '! h264parse ! rtph264pay name=pay0 pt=96 config-interval=-1 aggregate-mode=none )'
        )
        self.connect("media-configure", self.configure)

    def configure(self, factory, media):
        source = media.get_element().get_by_name("video")
        with self.lock:
            self.source = source
            self.waiting_keyframe = True
        media.connect("unprepared", self.unprepared, source)
        GLib.idle_add(self.request_keyframe)
        LOG.info("RTSP stream opened")

    def unprepared(self, media, source):
        with self.lock:
            if self.source == source:
                self.source = None
        LOG.info("RTSP stream closed; camera capture continues")

    def reset_video(self):
        # Capture can restart without resetting the RTSP clock or clients.
        with self.lock:
            self.waiting_keyframe = True

    def push(self, buffer):
        with self.lock:
            source = self.source
            if source is None:
                return
            keyframe = not buffer.has_flags(Gst.BufferFlags.DELTA_UNIT)
            if source.get_property("current-level-buffers") >= 1 and not self.waiting_keyframe:
                self.waiting_keyframe = True
                GLib.idle_add(self.request_keyframe)
            recovering = self.waiting_keyframe
            if self.waiting_keyframe:
                if not keyframe:
                    return
                self.waiting_keyframe = False
            # copy_region copies the buffer header and references the encoded
            # GstMemory. No pixels or H.264 bytes are copied into Python.
            outgoing = buffer.copy_region(Gst.BufferCopyFlags.FLAGS | Gst.BufferCopyFlags.TIMESTAMPS
                                          | Gst.BufferCopyFlags.MEMORY, 0, buffer.get_size())
            # appsrc timestamps against the RTSP media's live clock. Starting a
            # timeline at a preroll frame can carry startup delay into playback.
            outgoing.pts = outgoing.dts = Gst.CLOCK_TIME_NONE
            if recovering:
                outgoing.set_flags(Gst.BufferFlags.DISCONT)
            source.emit("push-buffer", outgoing)


class CameraServer:
    def __init__(self, args, channel=None):
        Gst.init(None)
        self.args = args
        self.channel = channel
        self.worker = None
        self.retiring = []
        self.detection_enabled = not args.no_detection
        self.detection_state = 'loading' if self.detection_enabled else 'disabled'
        self.detection_generation = 0
        self.revision = 0
        self.generation = 1
        self.observation = None
        self.last_error = None
        self.detection_error = None
        self.detection_count = 0
        self.last_worker_count = 0
        self.inference_sequence = 0
        self.capture_fps = self.video_fps = self.detection_fps = 0.0
        self.last_status = 0
        self.loop = GLib.MainLoop()
        self.factory = StreamFactory(args.fps)
        self.pipeline = None
        self.server = None
        self.server_source = 0
        self.timer_sources = []
        self.frame_count = 0
        self.capture_count = 0
        self.stale_frames = 0
        self.raw_age_ms = 0.0
        self.encode_age_ms = 0.0
        self.last_video_time = time.monotonic()
        self.last_capture_time = self.last_video_time
        self.last_result = (0.0, [], 0.0)
        self.last_stats_time = time.monotonic()
        self.last_stats_frames = self.last_stats_detections = 0
        self.last_stats_capture = 0
        self.restart_times = deque()
        self.restarts = 0
        self.recovering = False
        self.capture_started = self.last_video_time
        self.failed = False
        self.stopping = False

    def launch_string(self):
        a = self.args
        caps = f"width={a.capture_width},height={a.capture_height},framerate={a.fps}/1"
        if a.test_pattern:
            source = f"videotestsrc is-live=true pattern=ball ! video/x-raw,{caps}"
        elif a.image:
            source = (f"filesrc location={quote(a.image.resolve())} ! decodebin ! imagefreeze is-live=true "
                      f"! videoscale ! videoconvert ! video/x-raw,{caps}")
        elif a.format == "mjpeg":
            source = (f"v4l2src name=camera device={quote(a.device)} io-mode=mmap do-timestamp=true "
                      f"! image/jpeg,{caps} ! jpegdec")
        else:
            source = (f"v4l2src name=camera device={quote(a.device)} io-mode=mmap do-timestamp=true "
                      f"! video/x-raw,format=YUY2,{caps}")
        if (a.capture_width, a.capture_height) != (a.width, a.height):
            # Scale once in native code before both branches. Detection and
            # overlays therefore share the same image, including any borders.
            source += (" ! videoscale n-threads=1 add-borders=true "
                       f"! video/x-raw,width={a.width},height={a.height},pixel-aspect-ratio=1/1")
        launch = (
            f"{source} ! tee name=frames "
            "frames. ! queue name=video_queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream "
            "! videoconvert n-threads=1 ! video/x-raw,format=BGRx "
            "! cairooverlay name=boxes ! videoconvert n-threads=1 ! video/x-raw,format=I420 "
            f"! x264enc name=encoder tune=zerolatency speed-preset=ultrafast bitrate={a.bitrate} "
            f"key-int-max={max(1, round(a.fps / 4))} bframes=0 threads={a.encoder_threads} byte-stream=true "
            "! video/x-h264,profile=baseline,stream-format=byte-stream,alignment=au "
            "! h264parse config-interval=-1 "
            "! appsink name=encoded emit-signals=true sync=false max-buffers=1 drop=true "
        )
        # Keep the inference branch connected so toggling never rebuilds RTSP/video.
        rate_limit = ""
        if a.detection_fps < a.fps:
            # Drop inference-only frames before RGB conversion and shared
            # memory copies. The video branch retains every camera frame.
            rate_limit = (
                f"! videorate drop-only=true skip-to-first=true max-rate={a.detection_fps} "
                f"! video/x-raw,framerate={a.detection_fps}/1 "
            )
        feed_width, feed_height = fit_size(a.width, a.height, 320)
        launch += (
            "frames. ! queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream "
            "! valve name=detection_gate drop=true drop-mode=transform-to-gap "
            f"{rate_limit}"
            "! videoscale n-threads=1 "
            f"! video/x-raw,width={feed_width},height={feed_height} "
            "! videoconvert n-threads=1 ! video/x-raw,format=RGB "
            "! appsink name=inference emit-signals=true sync=false async=false max-buffers=1 drop=true"
        )
        return launch

    def request_keyframe(self):
        if self.pipeline and not self.stopping:
            event = GstVideo.video_event_new_upstream_force_key_unit(Gst.CLOCK_TIME_NONE, True, 0)
            self.pipeline.get_by_name("encoder").get_static_pad("src").send_event(event)
        return False

    def running_time(self):
        clock = self.pipeline.get_clock()
        return clock.get_time() - self.pipeline.get_base_time() if clock else None

    def discard_stale(self, pad, info):
        buffer = info.get_buffer()
        now = self.running_time()
        if now is not None and buffer.pts != Gst.CLOCK_TIME_NONE:
            self.raw_age_ms = (now - buffer.pts) / Gst.MSECOND
            if now - buffer.pts > self.args.max_frame_age * Gst.SECOND:
                self.stale_frames += 1
                return Gst.PadProbeReturn.DROP
        return Gst.PadProbeReturn.OK

    def captured(self, pad, info):
        self.capture_count += 1
        self.last_capture_time = time.monotonic()
        return Gst.PadProbeReturn.OK

    def capture(self, sink):
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.EOS
        worker = self.worker
        if not worker or not worker.ready:
            return Gst.FlowReturn.OK
        buffer = sample.get_buffer()
        info = GstVideo.VideoInfo.new_from_caps(sample.get_caps())
        mapped, mapping = buffer.map(Gst.MapFlags.READ)
        if not mapped:
            self.fail("Could not map the inference image")
            return Gst.FlowReturn.ERROR
        try:
            # Respect GStreamer's padded row stride; the view itself is not a copy.
            rgb = np.ndarray((info.height, info.width, 3), dtype=np.uint8, buffer=mapping.data,
                             offset=info.offset[0], strides=(info.stride[0], 3, 1))
            # Convert this pipeline's running time to the host monotonic clock.
            # Preserve source age, including time already spent in video queues.
            running = self.running_time()
            pts = sample.get_segment().to_running_time(Gst.Format.TIME, buffer.pts)
            if running is not None and pts != Gst.CLOCK_TIME_NONE:
                age = (running - pts) / Gst.SECOND
                if 0 <= age <= self.args.max_frame_age:
                    self.inference_sequence += 1
                    worker.submit(rgb, time.monotonic() - age, self.inference_sequence, self.generation)
        except Exception as exc:
            self.detection_error = f"Inference frame submission failed: {exc}"
            GLib.idle_add(self.stop_detection, self.detection_error)
        finally:
            buffer.unmap(mapping)
        return Gst.FlowReturn.OK

    def draw(self, overlay, context, timestamp, duration):
        result = self.observation
        if not result or not self.detection_enabled:
            return
        age = time.monotonic() - result['captured_at']
        if age < 0 or age > self.args.max_box_age:
            return
        context.set_line_width(2)
        context.set_font_size(13)
        for x1, y1, x2, y2, confidence, class_id in result['boxes']:
            x, y = x1 * self.args.width, y1 * self.args.height
            width, height = (x2 - x1) * self.args.width, (y2 - y1) * self.args.height
            context.set_source_rgb(0.15, 1, 0.2)
            context.rectangle(x, y, width, height)
            context.stroke()
            label = f"person {confidence:.0%}"
            text_width = context.text_extents(label)[2]
            text_x = max(0, min(x, self.args.width - text_width - 6))
            text_y = max(0, y - 19)
            context.set_source_rgba(0, 0, 0, 0.7)
            context.rectangle(text_x, text_y, text_width + 6, 19)
            context.fill()
            context.set_source_rgb(0.15, 1, 0.2)
            context.move_to(text_x + 3, text_y + 14)
            context.show_text(label)

    def encoded(self, sink):
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.EOS
        if self.recovering:
            LOG.info("Capture recovered after %.2f s without video; RTSP server kept running",
                     time.monotonic() - self.last_video_time)
            self.recovering = False
        self.frame_count += 1
        self.last_video_time = time.monotonic()
        now = self.running_time()
        frame_time = sample.get_segment().to_running_time(Gst.Format.TIME, sample.get_buffer().pts)
        if now is not None and frame_time != Gst.CLOCK_TIME_NONE:
            self.encode_age_ms = max(0, now - frame_time) / Gst.MSECOND
        try:
            self.factory.push(sample.get_buffer())
        except Exception as exc:
            self.fail(f"RTSP buffer delivery failed: {exc}")
            return Gst.FlowReturn.ERROR
        return Gst.FlowReturn.OK

    def fail(self, message):
        LOG.error("%s", message)
        self.failed = True
        self.last_error = message
        GLib.idle_add(self.stop)

    def bus_message(self, bus, message):
        if message.type == Gst.MessageType.ERROR:
            error, details = message.parse_error()
            self.fail(f"{message.src.get_name()}: {error.message}\n{details or ''}")
        elif message.type == Gst.MessageType.EOS:
            self.fail("Camera stream ended unexpectedly")
        elif message.type == Gst.MessageType.WARNING:
            warning, _ = message.parse_warning()
            LOG.warning("%s: %s", message.src.get_name(), warning.message)

    def watch_capture(self):
        if self.stopping:
            return False
        now = time.monotonic()
        # Allow device startup five seconds, then detect interruptions promptly.
        if now - self.capture_started < 5 or now - self.last_video_time < 2:
            return True
        LOG.warning("Video stalled: last capture %.2f s ago, captured %d frames, "
                    "encoded %d, stale drops %d, raw age %.1f ms",
                    now - self.last_capture_time, self.capture_count, self.frame_count,
                    self.stale_frames, self.raw_age_ms)
        while self.restart_times and now - self.restart_times[0] >= 60:
            self.restart_times.popleft()
        if len(self.restart_times) >= 3:
            faulthandler.dump_traceback()
            self.fail("Capture failed repeatedly: three restarts within a minute")
            return False
        self.restart_times.append(now)
        self.restarts += 1
        LOG.warning("Restarting capture (attempt %d in the last minute)", len(self.restart_times))
        # NULL cancels the blocked V4L2 poll and reopens the camera on PLAYING.
        # The separate RTSP pipeline and its timestamp clock stay running.
        if self.pipeline.set_state(Gst.State.NULL) == Gst.StateChangeReturn.FAILURE:
            self.fail("Could not stop stalled capture")
            return False
        self.factory.reset_video()
        self.last_result = (0.0, [], 0.0)
        self.observation = None
        self.generation += 1
        self.capture_started = time.monotonic()
        self.recovering = True
        if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
            self.fail("Could not restart capture")
            return False
        return True

    def report(self):
        if self.stopping:
            return False
        now = time.monotonic()
        elapsed = now - self.last_stats_time
        detections = self.detection_count
        self.capture_fps = (self.capture_count - self.last_stats_capture) / elapsed
        self.video_fps = (self.frame_count - self.last_stats_frames) / elapsed
        self.detection_fps = (detections - self.last_stats_detections) / elapsed
        LOG.info("capture %.1f FPS | video %.1f FPS | detection %.1f FPS | inference %.1f ms | people %d | encode age %.1f ms | stale drops %d | restarts %d",
                 (self.capture_count - self.last_stats_capture) / elapsed,
                 (self.frame_count - self.last_stats_frames) / elapsed,
                 (detections - self.last_stats_detections) / elapsed,
                 self.last_result[2], len(self.last_result[1]), self.encode_age_ms, self.stale_frames, self.restarts)
        self.last_stats_time, self.last_stats_frames, self.last_stats_detections = now, self.frame_count, detections
        self.last_stats_capture = self.capture_count
        return True

    def stop_detection(self, error=None):
        self.pipeline.get_by_name('detection_gate').set_property('drop', True)
        worker, self.worker = self.worker, None
        if worker:
            worker.begin_stop()
            self.retiring.append(worker)
        self.observation = None
        self.detection_state = 'failed' if error else 'disabled'
        self.detection_error = error
        return False

    def set_detection(self, enabled, revision):
        if revision == self.revision and enabled == self.detection_enabled:
            return
        self.revision = revision
        self.detection_enabled = enabled
        self.stop_detection()
        self.detection_generation += 1
        self.detection_state = 'loading' if enabled else 'disabled'

    def tick(self):
        try:
            return self._tick()
        except Exception as exc:
            # GLib otherwise removes a callback that raises, silently losing health reports.
            self.fail(f'Camera supervision failed: {type(exc).__name__}: {exc}')
            self.publish()
            return False

    def _tick(self):
        if self.stopping:
            return False
        if self.channel:
            for _ in range(8):
                try:
                    message = json.loads(self.channel.recv(4096))
                except BlockingIOError:
                    break
                if message.get('op') == 'stop':
                    self.stop()
                    return False
                if message.get('op') == 'detection':
                    self.set_detection(message['enabled'], message['revision'])
        self.retiring = [worker for worker in self.retiring if not worker.poll_stop()]
        if self.detection_enabled and not self.worker and not self.retiring and self.detection_state == 'loading':
            try:
                self.worker = DetectionWorker(self.args)
                self.last_worker_count = 0
                self.worker.start()
            except Exception as exc:
                self.stop_detection(str(exc))
        if self.worker:
            try:
                # Read progress before checking its deadline.
                result = self.worker.latest()
                if result is not None and result['camera_generation'] == self.generation:
                    result['detection_generation'] = self.detection_generation
                    self.observation = result
                    self.detection_count += result['completed_count'] - self.last_worker_count
                    self.last_worker_count = result['completed_count']
                    self.last_result = (result['captured_at'], result['boxes'], result['inference_ms'])
                self.worker.check()
                self.detection_state = 'ready' if self.worker.ready else 'loading'
                self.pipeline.get_by_name('detection_gate').set_property('drop', not self.worker.ready)
            except Exception as exc:
                self.stop_detection(str(exc))
        now = time.monotonic()
        if now - self.last_stats_time >= 1:
            self.report()
        if now - self.last_status >= .1:
            self.publish()
            self.last_status = now
        return True

    def publish(self):
        if not self.channel:
            return
        now = time.monotonic()
        ready = self.frame_count > 0 and now - self.last_video_time <= 1 and not (self.failed or self.stopping)
        state = 'failed' if self.failed else ('streaming' if ready else 'starting' if not self.frame_count else 'recovering')
        packet = dict(type='status', state=state, stream_ready=ready, revision=self.revision,
                      camera_generation=self.generation, detection_generation=self.detection_generation,
                      capture_fps=self.capture_fps, video_fps=self.video_fps, detection_fps=self.detection_fps,
                      capture_frames=self.capture_count, video_frames=self.frame_count,
                      last_frame_at=self.last_video_time if self.frame_count else None,
                      detection_state=self.detection_state, observation=self.observation,
                      detection_error=self.detection_error, last_error=self.last_error,
                      encode_age_ms=self.encode_age_ms, stale_frames=self.stale_frames,
                      capture_restarts=self.restarts, detector_pid=self.worker.process.pid if self.worker else None,
                      width=self.args.width, height=self.args.height)
        try:
            self.channel.send(json.dumps(packet, allow_nan=False).encode())
        except OSError:
            pass

    def stop(self):
        self.stopping = True
        self.loop.quit()
        return False

    def run(self):
        try:
            try:
                self.pipeline = Gst.parse_launch(self.launch_string())
            except GLib.Error as exc:
                raise RuntimeError(f"Cannot construct the GStreamer pipeline: {exc.message}. See README.md for system dependencies.") from exc
            self.pipeline.get_by_name("boxes").connect("draw", self.draw)
            camera = self.pipeline.get_by_name("camera")
            capture_pad = (camera.get_static_pad("src") if camera else
                           self.pipeline.get_by_name("frames").get_static_pad("sink"))
            capture_pad.add_probe(Gst.PadProbeType.BUFFER, self.captured)
            self.pipeline.get_by_name("video_queue").get_static_pad("src").add_probe(
                Gst.PadProbeType.BUFFER, self.discard_stale)
            self.factory.request_keyframe = self.request_keyframe
            self.pipeline.get_by_name("encoded").connect("new-sample", self.encoded)
            self.pipeline.get_by_name("inference").connect("new-sample", self.capture)
            bus = self.pipeline.get_bus()
            bus.add_signal_watch()
            bus.connect("message", self.bus_message)
            self.server = GstRtspServer.RTSPServer()
            self.server.set_address(self.args.host)
            self.server.set_service(str(self.args.port))
            self.server.get_mount_points().add_factory(self.args.path, self.factory)
            self.server_source = self.server.attach(None)
            if not self.server_source:
                raise RuntimeError(f"Cannot listen on {self.args.host}:{self.args.port}; is the port already in use?")
            self.last_video_time = self.last_capture_time = self.last_stats_time = self.capture_started = time.monotonic()
            if self.pipeline.set_state(Gst.State.PLAYING) == Gst.StateChangeReturn.FAILURE:
                error = bus.timed_pop_filtered(Gst.SECOND, Gst.MessageType.ERROR)
                detail = error.parse_error()[0].message if error else "unknown error"
                raise RuntimeError(f"Cannot start camera pipeline: {detail}")
            LOG.info("RTSP listening on %s:%d%s (LAN access, no authentication)",
                     self.args.host, self.args.port, self.args.path)
            LOG.info("Capture: %dx%d; RTSP: %dx%d at %d FPS; Ctrl+C stops capture and inference",
                     self.args.capture_width, self.args.capture_height,
                     self.args.width, self.args.height, self.args.fps)
            self.timer_sources.append(GLib.timeout_add(50, self.tick))
            self.timer_sources.append(GLib.timeout_add_seconds(1, self.watch_capture))
            for sig in (signal.SIGINT, signal.SIGTERM):
                self.timer_sources.append(GLib.unix_signal_add(GLib.PRIORITY_DEFAULT, sig, self.stop))
            self.loop.run()
            return 1 if self.failed else 0
        finally:
            self.stopping = True
            self.observation = None
            self.publish()
            for source_id in self.timer_sources:
                source = GLib.MainContext.default().find_source_by_id(source_id)
                if source:
                    source.destroy()
            if self.pipeline:
                self.pipeline.set_state(Gst.State.NULL)
            if self.server:
                self.server.client_filter(lambda server, client: GstRtspServer.RTSPFilterResult.REMOVE)
            if self.server_source:
                GLib.source_remove(self.server_source)
            if self.worker:
                self.worker.close()
            for worker in self.retiring:
                worker.close()
            LOG.info("Stopped")
