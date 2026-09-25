# apptirover

Drive the plain WAVE ROVER from an 8BitDo Lite 2 through this Raspberry Pi 5.
The app pairs/reconnects the controller and displays grouped, colored telemetry
with fixed columns for battery, motor output, joysticks, IMU/gyro, and system health.
It also streams USB camera video over RTSP with optional person detection. The
complete remote **ffplay command is always shown above the status table**.

## Run

The Python virtual environment is already prepared on this Pi:

```bash
cd /home/mieszko/apptirover
.venv/bin/python main.py
```

The screen updates in place at 5 Hz. Press **Ctrl+C** to send zero motor power,
close the serial connection, and exit. The default is `/dev/ttyAMA0`, 115200 baud. On this Pi `/dev/serial0` is the
separate debug UART and is not the rover connection. Your account already has
the required `dialout` membership; normal use does not require `sudo`.

The display includes battery voltage, roll/pitch/yaw, acceleration, angular-rate
and magnetic readings, ESP32 temperature, reported left/right motor fields, Pi
temperature/load, USB camera presence, reply ages, request/echo counters, and
serial errors, video/detection FPS, fresh person count and camera health.
Short interactive terminals omit secondary telemetry rows to keep the full
playback command visible; plain/JSON output retains the complete status.
Unknown reply fields are retained.

Battery `v` is **already in volts** on the verified firmware. Percentage,
charging state, and current are not exposed by the two telemetry replies and are
not estimated. IMU axes retain the firmware's values without speculative unit
conversion. The motor fields are not encoder measurements of wheel speed.

## Camera and person detection

Normal launch enables camera streaming and YOLO26n/NCNN person detection. The
replacement camera uses **MJPEG 640×480 at 30 FPS**, with detection capped at
**15 FPS**. Press **D** in the terminal to enable/disable detection while video
continues. The stream is `rtsp://<PI-LAN-IP>:8554/camera`; copy the complete ffplay
command from the CLI onto the other PC.

```bash
.venv/bin/python main.py --no-drive       # Stationary camera, telemetry and controller
.venv/bin/python main.py --no-detection   # Video without AI; manual driving enabled
.venv/bin/python main.py --no-camera      # Manual driving and telemetry only
.venv/bin/python main.py --camera-only    # Camera without UART or Bluetooth
```

Camera capture/encoding/RTSP run in a separate process; NCNN has another process.
The main app receives small timestamped observations, never video frames. Camera
or detector failures do not disable manual control, and detector failure leaves
video running. CPU thread limits, bounded queues and freshness checks keep old
work from accumulating. The existing serial worker remains the only motor writer.
No autonomous driving or person tracking has been added.

Use `--stream-host HOST` to choose the address shown in the ffplay command,
`--camera-device PATH` to select a camera, and `--detection-fps N` to limit AI
work. Default RTSP listening permits LAN viewers without authentication. Models
are already present locally but are excluded from Git.
See [camera setup, architecture and verification](docs/camera.md) for all modes,
dependencies, observation fields, runtime control and recovery behavior.

## Pair and read the Lite 2

Keep the app running. For first pairing, set the controller's switch to **D**,
press **Home**, and hold the **pairing button for about 3 seconds**. With one
Lite 2 in pairing mode nearby, the app discovers, pairs, trusts, and connects it
without prompts on the Pi. See the manufacturer's
[D-mode instructions](https://manual.8bitdo.com/lite2/lite2-android-bluetooth.html).

For later use, press **Home** in D mode. The saved controller reconnects without
pairing again. The screen distinguishes Bluetooth connection from **input ready**
and shows left/right X/Y values, pressed Linux button codes, D-pad values, and
the input event count. Stick values use an 8% center deadzone and
range from -1 to +1; negative Y is up. JSON also includes raw axes, kernel ranges,
observed minimum/maximum values, latest input event, input-device path, and Bluetooth address.
The left stick's Y axis and right stick's X axis control driving as described below.

BlueZ stores the bond; the app stores the selected address separately in
`~/.local/state/apptirover/controller.json` (or under `$XDG_STATE_HOME`). An offline
saved controller does not allow a different controller to enroll. If discovery
finds multiple matching controllers, use `--controller-address MAC` to select
the intended one. This also explicitly selects a replacement. The pairing agent
accepts only the selected device and HID services; the app does not make the Pi
discoverable or install a global default pairing agent.

The input reader follows the controller's Bluetooth identity, so changing
`/dev/input/eventN` numbers do not affect it. Disconnect clears the controls;
reconnect opens a fresh reader. A held stick may produce no new events, so event
silence alone is not a disconnect. Radio-loss detection still depends on Linux
and BlueZ noticing that the controller is gone. Controller battery percentage appears only
if BlueZ reports it; it was unavailable on this Lite 2 in D mode.

The current account has `input` membership and BlueZ access; no `sudo` is needed
for normal use. Bluetooth must be unblocked. Only one apptirover controller monitor
may run at a time. No automatic boot service is installed; launch `main.py`
to enable enrollment and reconnect supervision.

## Drive

After startup/reconnection, leave both driving axes centered for a quarter second.
Once the Drive row says **ready**, no extra button is needed:

| Input | Movement |
| --- | --- |
| Left stick up/down | Forward/reverse, proportional to tilt |
| Right stick left/right | Turn the nose left/right, proportional to tilt |
| Both together | Drive a curve with different left/right motor powers |
| Right stick alone | Turn on the spot with opposite wheel directions |
| Center both driving axes | Stop immediately |

Left-stick horizontal and right-stick vertical inputs are ignored for driving.
Buttons currently have no driving actions. Default maximum power is **50%**.
Use `--max-power 0.3` for 30%, or another fraction from 0.05 to 1. DC motors may
not start at small tilts; no automatic minimum-power jump is added.

The Pi mixes `left = throttle + turn`, `right = throttle - turn`, scales both
together to stay within the power limit, and sends both sides in one `T=1` JSON
command. The board drives each side's motors together. Turning right always turns
the nose right, including when reversing. The rover has no encoders: proportional
PWM adjusts motor power, not a measured speed in m/s. Ordinary changes ramp;
centering or a fault bypasses the ramp and commands zero immediately.

The serial worker evaluates controls at up to 50 Hz and refreshes motor commands
at up to 20 Hz, independently of terminal rendering. It stops and requires neutral
again after input-device loss, a drive-loop gap or controller-worker health older than 250 ms, BlueZ
status older than 750 ms, chassis feedback older than one second, a UART reconnect,
or dropped input events. Worker health is distinct from the age of the last changed
stick value. These deadlines do not bound silent radio-loss detection latency.

Each driving UART connection sends zero first and requests the board's runtime
500 ms movement-command timeout with `T=136`. Normal exit sends zero before
Bluetooth cleanup. UART loss/process death must rely on the board's timeout;
the requested timeout needs physical verification on the installed firmware.
No firmware is flashed and no flash settings are saved. Do not operate another
board web/ESP-NOW motion controller concurrently.

`--no-drive` keeps telemetry and controller input monitoring without any motor or
watchdog configuration commands. `--controller-only` and `--no-controller` also
disable driving. The initial direction check should use securely supported wheels
off the floor. There is no automatic startup forward pulse.

During later turning checks, front/rear wheels appeared matched when raised and
responded gradually to stick tilt. Uneven speeds seen only during floor turning
are consistent with differing grip/load: tight turns require the fixed tires to
slip sideways. The board supplies power per side and cannot synchronize individual
wheel RPM. Try wider moving turns on a smooth level surface; check wheel attachment
and rubbing with power off if one wheel consistently behaves differently.
See the turning investigation in [apptirover.md](apptirover.md).

## Diagnostics

```bash
# Bounded real-rover communication check:
.venv/bin/python main.py --no-controller --no-camera --duration 10 --plain

# Bluetooth/input check without opening the rover UART:
.venv/bin/python main.py --controller-only --refresh 5

# All returned telemetry fields as machine-readable snapshots:
.venv/bin/python main.py --no-drive --duration 10 --json

# Alternate UART, only if your wiring/configuration differs:
.venv/bin/python main.py --port /dev/ttyAMA0 --baud 115200
```

`--duration` exits with code 0 only when every enabled subsystem is ready at the
end: fresh chassis/IMU feedback, controller input ready, streaming video and
fresh detection results (when enabled). Code 2 indicates a subsystem is unavailable.
`--no-controller --no-camera` requires only rover feedback; `--controller-only`
requires only controller input; `--camera-only` requires only camera readiness.
A UART/controller monitor failure returns 1. Camera faults are reported without
ending interactive manual operation.
Interactive interruption exits normally and preserves the controller bond.

The monitor alternates `T=130` chassis queries and `T=126` IMU queries, targeting
two queries per second of each type. It accepts `T=1001` and `T=1002` replies,
tracks echoed requests separately, and flags telemetry older than three seconds
at the default poll rate. Opening a port or receiving an echo alone does not mark
telemetry healthy. Unavailable ports retry every two seconds. Old data remains
visibly stale across reconnects until replaced. Serial ownership is exclusive
between cooperating applications; stop other software that uses the rover UART
before launching.

Capable terminals use a live status screen. Dumb terminals, redirected output,
or `--plain` print a snapshot every two seconds. `--json` follows `--refresh`
(default 5 Hz) and keeps
all reply fields, useful on narrow terminals. Nothing is logged to disk by default.

## Recreate the environment and test

```bash
/usr/bin/python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt -r requirements-camera.txt
.venv/bin/python -m unittest discover -s tests -v
```

The camera also needs OS GStreamer/GI bindings and exported model files; see
[camera installation](docs/camera.md#dependencies-and-model). Manual operation
with `--no-camera` needs only the base requirements. All verification described
below before camera integration is historical.

Tests use captured reply shapes and a simulated rover over a pseudo-terminal.
They check fragmented/corrupt input, query-only output, echoes, exclusive access,
stale telemetry, reconnect generations, missing hardware, and voltage display.
Controller tests also cover scoped pairing, ambiguous devices, saved-address
reconnection, Bluetooth input identity, normalization, button release, dropped
event recovery, and clearing controls on disconnect. Tests do not move the real
rover or access Bluetooth. Drive tests exercise proportional mixing, power limits,
neutral interlocks, fault stops, stale health, actual serial framing/final stop,
and fixed positions for changing signed screen values.

On 2026-09-25 the live monitor completed a 12-second test on this rover with
44 telemetry replies, 44 echoes, no invalid lines, and no reconnects. Battery was
approximately 12.33 V and both IMU and chassis updates remained live. No USB
camera was detected during that run. No movement command was sent.

The Lite 2 subsequently paired automatically on this Pi in D mode. Its Linux
identity is Bluetooth vendor `2dc8`, product `5112`; left stick uses `ABS_X/Y`,
right stick `ABS_Z/RZ`, both with raw range 0–255. Real movements reached both
extremes on all four stick axes; button presses were also received. A physical
off/on cycle reconnected automatically and resumed fresh input readings.
Chassis and IMU telemetry remained live throughout. Pi reboot and real
radio-loss timing have not yet been tested.

The combined five-minute run finished with 1,103 telemetry replies, no malformed
lines, and controller input ready after the power cycle. An eight-second app
restart check reused the saved bond and finished with both subsystems ready.
Those checks preceded motor control. The current drive implementation passes 29
automated tests; hardware driving results are recorded in [apptirover.md](apptirover.md).

With the wheels securely raised, the user confirmed forward/reverse, left/right
turning, tilt-dependent power, combined turns, and stopping on centering. The
roughly 115-second run received 449 telemetry replies and 1,784 control echoes,
with no malformed lines or UART reconnects. A controller reconnect with deflected
sticks was held at zero output until neutral. Ground handling and the physical
500 ms board-watchdog timeout are not yet measured.

The automatic startup movement test remains omitted.
A strict 5 cm limit cannot be guaranteed by a timed pulse on this encoderless
chassis. See [apptirover.md](apptirover.md) for the full design and future stages.
