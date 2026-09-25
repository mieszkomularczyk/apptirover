# Camera and person detection

Integrated from the local `/home/mieszko/rovercam` implementation. This project
owns its camera code and model assets; it does not import the neighboring project
or launch its virtual environment. That project is unchanged.

## Use

Normal `main.py` starts manual control, telemetry, camera streaming and detection.
The replacement USB camera identifies as `HD USB Camaer 4K: HD USB Camaer` and
advertises MJPEG 640×480 at 30 FPS; these are the new capture/output defaults.
Its raw YUYV 640×480 mode supports at most 20 FPS. The old Arducam 2064×1552
capture configuration is not used for this camera.

```bash
.venv/bin/python main.py                           # Also enables manual driving
.venv/bin/python main.py --no-drive                # Stationary telemetry + camera + controller
.venv/bin/python main.py --no-detection            # Video without inference; manual driving enabled
.venv/bin/python main.py --no-camera               # Manual driving + telemetry, no camera workers
.venv/bin/python main.py --camera-only             # Camera, no UART or Bluetooth
.venv/bin/python main.py --controller-only         # Controller, no UART or camera
.venv/bin/python main.py --no-controller --no-camera --duration 10 --plain
```

Press **D** in the interactive terminal to toggle detection without restarting
video or the application. No controller buttons were reassigned. In headless or
JSON mode, `kill -USR1 APP_PID` toggles detection; target the main application PID.
`CameraMonitor.set_detection(bool)` provides the same operation for future code.
Disabling detection clears its observations immediately and stops the NCNN
process. Enabling starts a new worker asynchronously. A failed detector remains
failed until toggled off/on; video continues without boxes.

The CLI always displays the full remote ffplay command, even when streaming is
disabled or unavailable. Copy it onto the other PC. It uses the Pi's LAN route
address, or its `.local` hostname when there is no route. Override with
`--stream-host 192.168.50.161` or a hostname for multi-interface networks.
`--rtsp-host` controls the listener separately; the default `0.0.0.0` permits LAN
viewing. RTSP has no authentication; use `--rtsp-host 127.0.0.1` for local tests.

```bash
ffplay -rtsp_transport tcp -fflags nobuffer -flags low_delay -analyzeduration 0 -probesize 32 -threads 1 -framedrop -sync ext "rtsp://PI_LAN_IP:8554/camera"
```

If a client's FFmpeg version cannot probe with 32 bytes, increase `-probesize` to
32768. Player/network buffering affects latency; software frame age in the CLI is
not a measurement of remote display delay. Multiple viewers share one encoder.

Defaults: 640×480 H.264 at 30 FPS, detection capped at 15 FPS, YOLO26n NCNN,
320-pixel inference feed, two inference threads, one encoder thread, confidence
0.35. Relevant overrides:

```bash
.venv/bin/python main.py --camera-only --camera-device /dev/video0 --camera-format mjpeg
.venv/bin/python main.py --camera-only --capture-width 1280 --capture-height 720 --camera-width 640 --camera-height 360
.venv/bin/python main.py --camera-only --camera-fps 15 --detection-fps 10 --detection-threads 1
.venv/bin/python main.py --camera-only --rtsp-port 8555 --rtsp-path /rover
```

Supported modes depend on the camera. The default device is a sole
`/dev/v4l/by-id/*video-index0` path when available, otherwise `/dev/video0`.
Select `--camera-device` explicitly with multiple cameras. Capture and output
sizes are independent, and native scaling preserves aspect ratio with borders.
Camera exposure/focus controls are left unchanged.

## Process and data boundaries

- **Main application:** existing Bluetooth/input thread and sole UART/control
  thread, CLI, plus a camera supervisor/receiver thread. Camera health never gates
  current manual driving. Motor shutdown precedes all camera cleanup.
- **Camera process:** GStreamer capture, scaling, Cairo boxes, CPU H.264 encoding,
  shared RTSP server and asynchronous detector lifecycle. Created by a fresh
  interpreter with only its IPC socket inherited, never the UART/input handles.
- **Detection process:** spawned Python interpreter with NCNN inference. Only a
  small RGB image is copied through a shared-memory slot. A nonblocking lock
  protects this slot; a busy/dead reader causes frame drops, not a video wait.

Both processes run at nice +5. Native conversion/scaling uses one thread; NCNN
and encoding thread counts are bounded/configurable. These processes still share
the Pi's CPU/memory; separation is not a hard real-time guarantee.

Nonblocking Unix datagrams carry frame notifications, small JSON observations,
configuration revisions and health. Datagram boundaries avoid partial-message
reads after a worker crash. Socket buffers and frame storage are bounded. Video
callbacks never wait for the detector; the supervisor never waits in the control
loop. The main application reads a copied local snapshot, not a cross-process
lock. Detection results are collected independently of rendering/viewer presence.

Raw video queues retain one recent frame, stale raw frames expire at 200 ms,
and boxes expire 500 ms after their **source** frame. GStreamer running-time PTS
is converted to the host's monotonic clock, preserving time spent queued before
inference. This is a software capture timestamp, not sensor exposure timing.
Restart generations prevent old results from becoming valid in a new pipeline.

`camera` in JSON output (also `CameraMonitor.snapshot()`) includes:

| Field | Meaning |
| --- | --- |
| `stream_ready`, `state`, `frame_age_s` | Fresh encoded video and camera state |
| `detection_enabled`, `detection_state` | Requested mode and actual worker state |
| `observation_valid` | Current generation/revision, fresh stream/worker and source age ≤500 ms |
| `people` | Fresh person count; `null` when unavailable, `0` for a fresh empty detection |
| `boxes` | Fresh normalized `[x1,y1,x2,y2,confidence,class_id]` rows; only person class |
| `observation` | Last raw result for diagnostics; consumers must check `observation_valid` |
| `observation.captured_at`, `completed_at` | Host monotonic seconds, comparable within this boot |
| `observation.frame_sequence` | Sequence of submitted inference frames; skipped values are normal |
| `process_generation`, `camera_generation`, `detection_generation` in the observation | Identities spanning process/capture/detector restarts |
| `capture_fps`, `video_fps`, `detection_fps` | Measured one-second rates |
| `last_error`, `detection_error`, `previous_error` | Current camera/detector error and previous process failure |
| `rtsp_url`, `ffplay_command` | Remote playback endpoint and full command |

Control revisions invalidate observations immediately on a detection toggle,
even if an older packet is still queued. Worker heartbeat/video freshness expires
after one second. Detector warmup is allowed 45 seconds; inference without result
progress expires after three seconds. No detector work is done in the main app.

Capture recovery retains the RTSP server and tries three pipeline restarts in a
rolling minute. Whole camera process failures/hangs get at most three relaunches
per application run, with bounded backoff; a whole-process restart requires the
viewer to reconnect. The parent detects lost camera heartbeats after three
seconds (15-second initial interpreter grace). Linux parent-death signals and
process-group cleanup prevent orphaned camera/detection workers on exit.

Future autonomy should consume valid observations and propose motion to an
arbiter in the control layer. Only the existing UART owner may write motors.
Person tracking/selection, distance estimation and autonomous driving are not
implemented. Manual takeover must cancel autonomy; stale input/vision must stop
autonomous commands, and reconnect must not silently resume following.

## Dependencies and model

The project venv has already been prepared. To recreate it on this Pi:

```bash
sudo apt-get install -y python3-venv python3-gi python3-gst-1.0 python3-gi-cairo python3-cairo \
  gir1.2-gst-rtsp-server-1.0 gstreamer1.0-tools gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly ffmpeg
/usr/bin/python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt -r requirements-camera.txt
```

Use the system Python version to match Debian's compiled GI/Cairo bindings. The
camera worker loads those two bindings from `/usr/lib/python3/dist-packages`
without exposing unrelated OS Python packages to the venv. The main control
process never imports GI, OpenCV, NumPy or NCNN. Manual operation with
`--no-camera` needs only `requirements.txt`.

The existing exported `.param`, `.bin` and `metadata.yaml` were copied into
`models/yolo26n_ncnn_model/`. This local model directory is ignored by Git; a new
checkout needs those files copied in, an explicit `--detection-model PATH`, or a
fresh export. Missing models fail detection visibly while streaming continues.
No model downloads or PyTorch imports occur during normal application startup.

For a fresh export, use a separate `.venv-export` with compatible CPU-only
PyTorch/torchvision and `requirements-export.txt`, then run
`.venv-export/bin/python tools/export_model.py`. The source project used
torch 2.14.0+cpu and torchvision 0.29.0+cpu. The default export is 320×320;
larger exports still receive the 320-pixel feed and upscale it, so they do not
recover additional image detail. Weights/export tools retain their upstream
licenses. See the source project's README for its original export procedure.

## Verification

```bash
.venv/bin/python -m unittest discover -s tests -v
.venv/bin/python -m pip check
.venv/bin/python tools/check_camera.py
.venv/bin/python tools/check_camera.py --image /path/to/person-image.jpg
.venv/bin/python main.py --camera-only --duration 30 --json
```

The integration check uses real GStreamer, NCNN and an FFmpeg receiver alongside
a simulated UART/controller. It checks stream-only startup, enable/disable,
detector crash, camera crash/freeze/recovery, fresh observations, continued motor
and telemetry scheduling, neutral stop and final zero. It never opens real
UART/Bluetooth/USB hardware. The image variant also requires a detected person.

For a stream test without camera or model:

```bash
.venv/bin/python main.py --camera-only --camera-test-pattern --no-detection --rtsp-host 127.0.0.1
```

The earlier movement-triggered Arducam fault belongs to the replaced camera.
The generic watchdogs remain; no stall diagnosis is assumed for the replacement.

Recorded 2026-09-25: 45 unit tests pass; `pip check` reports no broken requirements.
The image integration check identified four people, survived detector/camera
crashes and a frozen camera, and received 359 simulated telemetry replies. Its
control-loop gap was 20.3 ms median, 22.9 ms p99 and 26.6 ms maximum against the
existing 250 ms fault threshold. These are measured samples, not a deadline
guarantee.

A 60-second stationary test with the replacement USB camera encoded 1,771
frames, with median video 29.84 FPS, detection 14.92 FPS and inference 21.69 ms.
There were no capture or process restarts. One stale raw frame was dropped during
startup. FFmpeg decoded 300 frames without errors. Real UART produced 233 valid
telemetry replies, zero malformed lines and zero motor commands. The controller
was offline throughout; the app correctly returned exit code 2 for that enabled
subsystem. Physical joystick input under camera load remains unverified.
