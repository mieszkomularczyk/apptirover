# Single-file executable

The built artifact is `dist/apptirover`, approximately 103 MiB. It contains the
Python interpreter, application, serial/Bluetooth Python libraries, NCNN,
NumPy/OpenCV, GI/Cairo, GStreamer libraries and required plugins/typelibs, and the
three YOLO26n NCNN model files. It needs no project checkout, venv or separate
model directory at runtime. The Linux kernel, device permissions, system D-Bus
and BlueZ service remain supplied by the OS.

## Launch

```bash
./dist/apptirover
./dist/apptirover --no-drive
./dist/apptirover --follow-pulse-ms 40 --follow-distance-scale 1.4
```

Copy just `dist/apptirover` to the directory from which you want to run it. Keep
its executable bit; use `chmod +x apptirover` if your transfer method drops it.
CLI arguments and X/D behavior are the same as the Python launch. The complete
remote ffplay command remains on screen. No runtime `ffmpeg` or `ffplay` is
needed on the rover; ffplay runs on the receiving PC.

This build is for **ARM64 Linux**, built on this Pi's Debian 13 environment with
glibc 2.41. Use the same OS generation on another Pi, or build on that target's
OS. It is not a Windows or x86 executable. Hardware permissions and the saved
controller pairing are configured on each rover as usual; they are not copied
into the executable.

The one-file launcher extracts its bundled libraries into a temporary directory
at startup and removes it on normal exit. The filesystem must allow executable
libraries and have space for the extracted payload. Camera and inference
children reuse this directory. You can choose a writable location with `TMPDIR`.
This follows PyInstaller's [one-file extraction behavior](https://pyinstaller.org/en/stable/usage.html#defining-the-extraction-location).

## Rebuild after changing code or model

With the normal runtime dependencies and exported model already installed:

```bash
.venv/bin/python -m pip install -r requirements-build.txt
.venv/bin/python tools/build_executable.py
```

The builder validates the target architecture and model files, uses the current
venv's runtime libraries, and prints the resulting file size and SHA-256.
Build tools are pinned in `requirements-build.txt`; the spec is
[`packaging/apptirover.spec`](../packaging/apptirover.spec). Only GI/Cairo and
their metadata are exposed from Debian's Python installation, keeping distro
NumPy/OpenCV from shadowing the project venv. The
[GStreamer include list](https://pyinstaller.org/en/stable/hooks-config.html#gstreamer-gi-repository-gst-hook)
bundles the plugins used by USB MJPEG/YUYV capture, JPEG/PNG test images,
conversion, overlays, H.264 and RTSP. Export tooling and unused GUI frameworks
are excluded.

`build/` and `dist/` are ignored by Git. Source changes take effect in the binary
only after rebuilding; the Python launch remains available for development.

## Processes and verification

[`launcher.py`](../apptirover/launcher.py) calls multiprocessing
[`freeze_support()` before CLI dispatch](https://pyinstaller.org/en/stable/common-issues-and-pitfalls.html#multi-processing).
The frozen executable dispatches its internal camera worker itself, while the
source launch continues to use `python -m apptirover.vision.worker`. Inference
retains its separate spawned process and the existing parent-death cleanup.
Native AI/camera imports remain outside the main control process. The model's
default path resolves inside the extracted bundle; `--detection-model` still
accepts an external model override.

```bash
.venv/bin/python -m unittest discover -s tests -q
.venv/bin/python tools/check_executable.py
.venv/bin/python tools/check_executable.py --image /path/to/one-person.jpg
```

The executable checker copies only the binary and optional fixture into a new
temporary directory, strips Python/GI/GStreamer environment overrides and venv
paths, then checks help, pseudo-UART telemetry with the rendered CLI, bundled
model inference, 30 decoded RTSP frames, detector toggles, detector crash/recovery
and shutdown. It opens no real UART, Bluetooth controller or USB camera. FFmpeg
is used by this test tool as a loopback viewer, not by the application.

On 2026-10-23, 104 unit/integration tests passed. Both the copied-binary check
and the existing real-NCNN/GStreamer source check passed with a still-image
fixture and simulated rover. The source check received 410 telemetry replies,
decoded 288 frames before restart, and measured control gaps of 20.5 ms median,
25.2 ms p99 and 29.7 ms maximum. These checks do not replace a physical rover
test.
