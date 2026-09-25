# apptirover

Updated: 2026-09-25. Status: UART telemetry, Bluetooth pairing/input monitoring,
and proportional two-stick manual driving implemented. Camera integration and
autonomy remain future stages.

## Current implementation

The latest instruction adds left-stick forward/reverse, right-stick turning,
combined proportional steering, and a grouped fixed-column colored status screen.
The Python master script is [main.py](main.py), using
the project `.venv`. Run it with `.venv/bin/python main.py` from this directory.
See [README.md](README.md) for setup, diagnostic options, and tests.

Implemented: a continuously updating CLI status screen, a separate serial-reader
thread, bounded JSON framing, chassis and IMU polling, echo handling, stale-data
indicators, reconnect attempts, Pi health, USB camera presence, and plain/JSON
output modes. Driving is enabled by default when both UART and controller are
enabled. `--no-drive` retains stationary telemetry/input diagnostics.

### Manual driving implementation

Left Y is inverted to form forward throttle; right X controls nose-right turning.
Both inputs use the existing 8% center deadzone. Left X and right Y do not drive.
Mix left/right as `throttle + turn` and `throttle - turn`; normalize both by the
same factor if either exceeds magnitude 1, then apply `--max-power` (default 0.5).
This supports forward/reverse curves and on-the-spot turns. No drive-enable button
is needed in this version; both driving axes must first remain centered for 250 ms
after startup, reconnection, or a fault. Controller input is the only motion source.

The board accepts both sides together in `T=1` with floating-point L/R values.
The vendor [motor implementation](https://github.com/waveshareteam/ugv_base_general/blob/main/General_Driver/movtion_module.h)
multiplies non-encoder setpoints by 512. The adapter maps normalized power to
`power * 255 / 512` to stay within 8-bit PWM even at the full-power setting.
This is motor power, not regulated speed. Each side's motors share the board's
output; Python does not address the four motors individually. The `T=13` ROS
velocity mode is not used for this encoderless chassis. No vendor code is copied.

The existing serial worker is the sole motor-command writer. It evaluates input
at up to 50 Hz and refreshes both motor outputs at up to 20 Hz, independently of
the display. Changes ramp at 200 percentage points of full power per second,
with a common interpolation factor for both sides. Neutral and fault stops bypass
the ramp. Latest input replaces old state; there is no queued motion backlog.
The display distinguishes commanded power from board-reported L/R fields.

Drive stops and requires neutral again on controller input loss, changed input
generation, UART reconnection, invalid values, dropped kernel events, a drive-loop
gap longer than 250 ms, controller
worker heartbeat older than 250 ms, BlueZ status older than 750 ms, or chassis
feedback older than one second. An asyncio heartbeat checks worker execution at
50 ms intervals; BlueZ is polled every 100 ms while connected. These indicate
software/known-link health, not proof of fresh radio packets. A held stick remains
valid without change events. Real out-of-range detection latency remains unmeasured.

Each driving serial connection sends zero before other commands, then requests
`T=136, cmd=500` to set the board's movement watchdog. The vendor
[UART handler](https://github.com/waveshareteam/ugv_base_general/blob/main/General_Driver/uart_ctrl.h)
refreshes this watchdog only on motion commands; telemetry queries do not refresh
it. An echo alone does not prove firmware execution. The 500 ms timeout is a
runtime request, still requiring physical verification on this board. Normal
shutdown sends zero before waiting for Bluetooth cleanup. UART failure/process
death relies on the board watchdog; no firmware or persistent settings are changed.
Concurrent board web/ESP-NOW control is not arbitrated by this application.

The status table has fixed-width, right-aligned numeric cells with stable decimal
places and separate color bands for battery, drive, controller, IMU/gyro, and
system data. The normal 80-column display fits within 24 rows. It refreshes at
5 Hz; plain output remains periodic and JSON retains complete raw telemetry,
control state, health ages, and extra firmware fields. No camera is opened.

Twenty-nine automated tests cover mixing, limits, neutral requirements, fault
stops, stale health, dropped events, serial stop framing, and screen alignment,
alongside existing UART and Bluetooth checks. This includes requiring neutral
after a paused drive loop instead of resuming a held command.

Real hardware validation: with all wheels securely raised, the user confirmed
forward/reverse, turning both ways, proportional tilt response, simultaneous
throttle and steering, and stopping when centered. The roughly 115-second run
received 449 telemetry replies and 1,784 control echoes with no malformed input
or UART reconnects. A controller disconnect/reconnect was observed; its new
deflected input state remained at zero output behind the neutral interlock.
The diagnostic process was stopped cleanly after the test. Physical watchdog
timing, out-of-range stopping latency, and ground handling remain unmeasured.

### Turning response investigation

The user subsequently reported uneven front-right/rear-right wheel speeds during
floor turning and poor response to stick tilt. The earlier direction test does
not establish matched wheel RPM or satisfactory low-power ground handling.

The stock interface exposes two motor commands, L and R; the app cannot command
different powers for front and rear motors on the same side. Exact wheel-speed
synchronization is not available without per-wheel sensing and independently
controlled outputs. Motor variation, mechanical drag, electrical connections,
and unequal tire loading can still produce different speeds at the same PWM.
These are candidate causes, not a diagnosis of this unit.

A read-only `T=139` query returned `L=1, R=1`: neither side has a reduced stored
gain. Existing logs and a repeat raised-wheel test show equal-and-opposite
left/right turn commands, including intermediate values, with no malformed UART
messages. At the default 50% power cap, normalized steering inputs of 0.1, 0.5,
and 1.0 request approximately 13, 64, and 127 PWM counts respectively (of 255).
Normalized input is after the joystick center deadzone. The manufacturer notes
poor low-speed behavior and possible failure to rotate at low PWM in the
[WAVE ROVER movement documentation](https://www.waveshare.com/wiki/WAVE_ROVER#Chassis_Movement).
Floor resistance may therefore make much of the stick range ineffective even
though the transmitted power is proportional.

The user then confirmed that front/rear wheels appear to run at the same speed
with the wheels raised, and that speed increases gradually from partial to full
steering tilt. This supports load-dependent tire grip/scrub or mechanical effects
during floor turning as the leading explanation, rather than incorrect joystick
scaling or separate front/rear software commands. It does not prove a specific
mechanical defect or rule out a loose wheel attachment under load.

For a tight stationary turn, fixed wheels must slip sideways. Unequal grip/load
can therefore produce different motor speeds despite shared power. Wider moving
turns reduce this demand. Before changing motor calibration, compare behavior on
a smooth level surface and check wheel attachment/rubbing with power off. Do not
promise matched wheel RPM or mask the symptom with an unmeasured power jump.
No power limits, motor gains, or steering behavior were changed during this
investigation. The diagnostic process was stopped cleanly after the comparison.

The master script also starts a Bluetooth/input worker thread with an asyncio
loop. BlueZ D-Bus handles discovery, a scoped NoInputNoOutput pairing agent,
trust, and reconnect retries. Linux evdev asynchronously reads the matching
Bluetooth input device. This implementation uses threads; stronger process
isolation for future camera/autonomy workloads remains future work.

With no selected controller, a Lite 2 name plus gamepad class or HID UUID is
required before enrollment. Multiple candidates produce a visible ambiguity.
BlueZ retains the bond; the app saves the chosen address under
`~/.local/state/apptirover/controller.json`, respecting `XDG_STATE_HOME`. It
reconnects only that controller; powered-off controllers do not trigger new
enrollment. `--controller-address MAC` explicitly selects a specific controller.
The app does not make the Pi discoverable or become the global default pairing
agent. It refuses unrelated pairing/service requests. A file lock stops duplicate
monitors. Retries use bounded backoff; loss of BlueZ restarts the worker's D-Bus
connection. No boot service has been installed.

Input readiness requires the selected MAC in the Linux device's `uniq` field,
Bluetooth bus identity, and both stick axis pairs. Snapshots contain raw axes and
ranges, normalized sticks with an 8% center deadzone, pressed Linux button codes,
D-pad values, event counts, and event ages. Disconnect clears controls and
reconnect reacquires the device. Dropped kernel events resync from current kernel
state. A held stick does not falsely become stale just because no values changed.
The driving axes are routed through the guarded mixer above. Real radio-loss
stop timing remains unmeasured.

Real first enrollment succeeded in **D mode** with only controller-side pairing
action. Linux identifies `8BitDo Lite 2`, Bluetooth vendor `2dc8`, product `5112`.
Left stick uses `ABS_X/Y`; right stick uses `ABS_Z/RZ`; each has raw range 0–255
and neutral near 127/128. Triggers report `ABS_GAS/BRAKE`, D-pad `ABS_HAT0X/Y`.
Live movement reached both extremes on all four stick axes and button events
were received. A physical off/on cycle automatically reconnected and produced
fresh input events. UART telemetry stayed live with no malformed replies.
Controller battery percentage was not exposed by BlueZ. Pi reboot and Bluetooth
service restart have not been physically tested. The status screen displays
inputs; `--json` exposes raw fields. `--controller-only` runs without UART;
`--no-controller` preserves UART-only diagnostics.

The five-minute combined test completed with 1,103 rover telemetry replies,
no malformed lines, and controller input ready following the physical power
cycle. An eight-second application restart reused the saved pairing and finished
with both subsystems ready. Those 20-test checks preceded motor control.

Verified on the real hardware: GPIO14 is TXD0, GPIO15 is RXD0; `/dev/ttyAMA0`
works at 115200 baud. `/dev/serial0` points to `/dev/ttyAMA10`, the debug UART.
The current user belongs to `dialout`; no other process held the rover port.
The firmware echoes requests and returns `T=1001` chassis and `T=1002` IMU data.

A 12-second run returned 44 sensor packets plus 44 echoes, with no invalid lines
or reconnects. It reported approximately 12.33 V battery voltage, roll/pitch/yaw,
temperature, acceleration, gyroscope and magnetometer readings, and zero L/R
fields. Battery `v` is directly in volts, unlike the scaling used in parts of the
multi-model vendor app. Battery percentage, current, and charging state are not
reported by these replies. USB camera presence was not detected in this run.
Tests cover framing, valid feedback versus echoes, freshness, reconnects, bounded
storage, missing hardware, voltage display, simulated serial I/O, scoped pairing,
input identity/normalization, button release, and dropped-event recovery.

The previously requested startup movement test is deferred for this communication
milestone. This encoderless rover cannot enforce a measured 5 cm maximum using a
timed pulse. No forward pulse has been implemented or executed.

The remaining sections describe the overall requirements and proposed next
stages; they do not imply that those features are already implemented.

## 1. Intended outcome

Build an application on this Raspberry Pi 5 that controls the connected plain
Waveshare WAVE ROVER, accepts a directly connected Bluetooth gamepad, streams
camera video with person detection, and later follows a detected person.

The user's requirements are:

1. Control the whole rover through this Raspberry Pi.
2. Drive using an 8BitDo Lite 2 Bluetooth controller. The controller must have
   master override over every other source of motion, including autonomy.
3. When no controller is paired, putting the Lite 2 into pairing mode must be
   enough to pair it automatically. Subsequently, powering it on must reconnect
   it automatically, without any action on the rover.
4. Make the local camera stream and human detection available, using the work
   already present in `/home/mieszko/rovercam`.
5. Add autonomy, initially following a detected human, while retaining controller
   override at all times.
6. Consult the WAVE ROVER documentation and `/home/mieszko/ugv_rpi` to avoid
   repeating existing work, but build an application tailored to this rover
   rather than copying the vendor application wholesale.
7. Develop in Python using a project virtual environment (`venv`).
8. Launch through a master script that immediately displays a live CLI status
   screen, including rover telemetry, 8BitDo connectivity, and camera presence.
   Expand this screen as new features are added.

The architecture, button assignments, limits, and milestones below are proposed
defaults. They are distinguished from the requirements above and from observed
facts; they have not been implemented or validated on the moving rover.

The user has authorized implementation of rover telemetry and subsequently
controller pairing/input reading, followed by proportional manual driving. The earlier
planning document was committed and pushed to the configured GitHub repository.

## 2. Hardware and environment

| Item | Evidence and current understanding |
| --- | --- |
| Project | `/home/mieszko/apptirover`; this document is `apptirover.md` |
| Host | User specifies Raspberry Pi 5; device tree reports Model B Rev 1.1 |
| OS | Local `/etc/os-release` reports Debian GNU/Linux 13.7, Trixie |
| Kernel | Local inspection reports `6.18.50+rpt-rpi-2712` |
| Chassis | User specifies plain Waveshare WAVE ROVER |
| Wiring | User reports GPIO TX, RX, GND, SCL, SDA connected to rover |
| Network | User confirms Wi-Fi and internet access |
| Controller | 8BitDo Lite 2, D mode; automatic pairing and real input readings verified |
| Camera | Existing rovercam records identify Arducam B0627 USB3 / IMX900, last tested 2026-09-23 |
| Other hardware | No pan-tilt, arm, lidar, depth sensor, or additional lights confirmed |

TX/RX/GND provide the UART connection. SCL/SDA belong to a separate I²C bus.
Document the physical header pins, BCM numbers, and rover-side labels before
implementation. Read the onboard sensors through the ESP32's serial interface
where supported; direct Pi I²C access is not assumed to be necessary.

The boot configuration contains `dtparam=uart0=on`. Host-device inspection and
real telemetry queries have verified `/dev/ttyAMA0` as the rover connection.
The sandbox alone does not expose its device nodes. Bluetooth is powered,
unblocked, and has a trusted Lite 2 bond; camera identity is based on the earlier rovercam
investigation, with no USB camera detected by the current monitor run.

The clone selects `/dev/ttyAMA0` for Pi 5. Do not blindly substitute
`/dev/serial0`: on Pi 5 that alias normally identifies the debug UART. Confirm
which device actually serves the wired GPIO pins and that no serial console
uses it. Preserve Bluetooth while configuring UART. See
[Raspberry Pi UART documentation](https://www.raspberrypi.com/documentation/computers/configuration.html#configuring-uarts).

The Pi's power source and the rover battery arrangement remain unrecorded.
Do not assume that software can switch the rover's main power or power the Pi
back on after shutdown.

## 3. Findings from the existing software

### rovercam: retain the camera and detection foundation

Reviewed [README](../rovercam/README.md),
[capture-stall investigation](../rovercam/docs/capture-stall.md),
[camera server](../rovercam/rovercam/app.py),
[detector](../rovercam/rovercam/detection.py), and
[CLI settings](../rovercam/rovercam/__main__.py).

Existing functionality:

- USB V4L2 capture and native GStreamer scaling, overlays, and H.264 encoding.
- Built-in RTSP server at `rtsp://<PI-IP>:8554/camera`; multiple viewers share
  an encoder. Capture and inference continue without viewers.
- YOLO26n exported to NCNN, retaining the person class only. Default model input
  is 320×320, using two inference threads in a separate process.
- A shared latest-frame slot and bounded video queues prevent growing backlogs.
- Current configured defaults are 640×480 output at 30 FPS, detection capped at
  15 FPS. The recorded B0627 capture format is 2064×1552 YUYV; requesting 640×480
  directly from that unit is not the recorded working configuration.
- Person boxes are normalized coordinates with confidence and class, internally
  accompanied by the source frame timestamp and inference duration.
- Stale boxes expire after 0.5 seconds of source-frame age. Raw frames older than
  0.2 seconds are discarded before encoding.
- Bounded capture recovery retains the RTSP server; repeated stalls eventually
  exit with an error. No boot service is installed according to the README.

The recorded short tests achieved approximately 30 FPS video and 15 detections
per second, with roughly 21–25 ms inference. These are historical local results,
not new measurements or guarantees of camera reliability or remote display delay.

**Known issue:** capture stalls during camera movement or scene changes. This
also occurred in raw V4L2 testing after physically reconnecting the camera,
including at 15 FPS. Lowering FPS or restarting capture is not an established
fix. The report's next isolation step is the same test on another computer.

There is currently no external detection subscription API and no persistent
person tracking. The overlay consumes the internal result queue. Integration
needs an explicit result publisher and health interface; autonomy must not
decode the annotated RTSP stream or open a second camera/inference pipeline.

### ugv_rpi: use selectively as a reference

Reviewed local commit `3ae9f20`, particularly
[base_ctrl.py](../ugv_rpi/base_ctrl.py), [app.py](../ugv_rpi/app.py),
[config.yaml](../ugv_rpi/config.yaml),
[browser controller](../ugv_rpi/templates/control.js),
[setup.sh](../ugv_rpi/setup.sh), and the English protocol tutorials.

| Useful reference | How apptirover should use it |
| --- | --- |
| JSON command framing and telemetry | Implement a small adapter for the verified WAVE ROVER firmware |
| OLED and optional peripheral commands | Expose supported capabilities through the same rover service |
| Camera and UI examples | Learn interaction patterns; keep the existing rovercam pipeline |
| Browser gamepad handling | Reference only; apptirover reads the controller on the Pi directly |
| Installation and startup scripts | Review individual settings; create a separate installation plan |

Specific differences that matter:

- The clone's default configuration identifies `UGV Rover`, with motion limits
  intended for its broader set of chassis. Those defaults are not this rover's
  capability specification.
- Its browser gamepad path uses the browser's Gamepad API and sends `T=13`
  linear/angular velocity commands. This does not provide Pi-side pairing or
  the intended plain WAVE ROVER drive mapping.
- `BaseController` uses an unbounded FIFO for outgoing commands. Motion in
  apptirover must replace old pending requests, so a stop cannot wait behind
  a backlog of stale motion.
- Initializing the vendor application opens the serial port and sends startup
  commands. Treat the files as references, not harmless modules to import for
  discovery.
- **The cloned installer adds `dtoverlay=disable-bt` and disables Bluetooth
  services. Running it would conflict directly with this project's controller
  requirement.** Its hotspot setup also does not match the existing Wi-Fi setup.
- The clone contains a [GPL license](../ugv_rpi/LICENSE); retain source and license
  provenance for any code selected for reuse. No vendor code has been copied.

## 4. Rover interface and capability scope

The [WAVE ROVER wiki](https://www.waveshare.com/wiki/WAVE_ROVER) documents UART
at 115200 baud with newline-delimited JSON. Its recommended drive command is
`T=1`, with left/right values from −0.5 to +0.5. On this chassis these represent
motor power: 0.5 means full PWM, not 0.5 m/s. The motors lack encoders. `T=11`
provides direct PWM values from −255 to +255 for debugging; `T=13` velocity
control and encoder PID settings are not the basis for this plain rover.
The wiki describes stopping after three seconds without a new movement command.
Verify that behavior on the installed firmware before relying on it.

Use a single Pi service as the owner of the serial port. It should support:

| Capability | Planned behavior |
| --- | --- |
| Drive | Forward/reverse, differential steering, and turning on the spot, with bounded left/right power |
| Stop | Explicit zero drive output, available from controller and service fault handling |
| Telemetry | Parse supported chassis, battery, and IMU feedback; mark missing/stale values explicitly |
| OLED | Show connection, mode, stop reason, and useful status without delaying drive commands |
| Optional outputs | Expose only attached, verified peripherals; default unused outputs off |
| Configuration | Keep firmware-specific settings separate from ordinary driving |

The local protocol tutorials and
[Waveshare command reference](https://www.waveshare.com/wiki/08_Sub-controller_JSON_Command_Set)
describe `T=130` feedback requests, `T=131` continuous feedback, `T=142` feedback
interval, `T=143` echo, and `T=126` IMU requests. The clone demonstrates `T=3`
OLED updates and `T=132` auxiliary outputs. Verify supported commands, units, and
reply fields against this board; a multi-model sample is not proof of support.

Do not treat a successful write or echoed command as proof that the wheels
moved or stopped. Without encoders, commanded power is not measured speed or
distance. Expose these distinctions in status and logs.

During implementation, identify any other firmware control paths, including the
ESP32's own Wi-Fi/HTTP and ESP-NOW control. The Pi's priority rules cannot govern
commands that bypass it. Establish sole motion authority, disabling or excluding
those alternative paths through verified firmware settings where possible.
Record any remaining limitation before claiming override works over all sources.

## 5. Proposed architecture

Python and a project-local `.venv` are confirmed requirements. Keep the native
GStreamer/NCNN media pipeline. Use the system-compatible Python interpreter;
rovercam currently relies on OS-provided GObject/GStreamer bindings, so its
environment needs access to those bindings. Keep model-export tooling separate
from the runtime environment. Record dependencies reproducibly and exclude
virtual environments, caches, credentials, and downloaded models from Git when
implementation begins. The UART milestone now has its own prepared virtual
environment; future vision integration must account for the OS GStreamer bindings.

Proposed dependencies
include BlueZ D-Bus for Bluetooth, Linux input events through
[python-evdev](https://python-evdev.readthedocs.io/en/latest/tutorial.html), and
[pySerial](https://pyserial.readthedocs.io/en/latest/pyserial_api.html) with bounded
read/write timeouts and exclusive port access where supported. Choose exact
versions during implementation on the current OS. For the current UART milestone,
pySerial 3.5 and Rich 14.1.0 are installed in `.venv`; exact runtime dependencies
are recorded in [requirements.txt](requirements.txt). No Bluetooth libraries are
installed for this stage.

```mermaid
flowchart LR
    Pad[8BitDo Lite 2] --> BT[Bluetooth and input service]
    BT --> Core[Control authority and stop handling]
    Camera[USB camera] --> Vision[rovercam service]
    Vision --> Stream[RTSP viewers]
    Vision --> Follow[Person-following worker]
    Follow --> Core
    Core --> Serial[Single UART owner]
    Serial <--> ESP[WAVE ROVER ESP32]
    Core --> Status[Master script CLI status screen]
    Vision --> Status
```

The control authority and serial owner form one small core service. Bluetooth
pairing/input, camera processing, and autonomy run in separate processes so a
stalled inference call or pairing operation cannot hold the control loop.
Messages between processes carry sequence numbers, source identity, a monotonic
timestamp, expiry, and the current control-session identifier where relevant.
Use bounded local IPC, such as Unix sockets, with explicit reconnection behavior.

Only the core may send motion to the rover. Camera and autonomy processes never
open UART. Future web controls must request authority through the same core.
Raw JSON passthrough must not become a route around motion arbitration.

On takeover, stop, disconnect, or restart, invalidate the old control session
and discard its pending motion. The serial writer rechecks authority and expiry
immediately before transmitting. Diagnostics and OLED messages have a bounded
budget and lower priority than motion and stop requests.

## 6. Automatic Bluetooth pairing and reconnection

Pair the controller directly to the Raspberry Pi, independently of any browser,
phone, internet connection, or camera service.

Start with the controller in **D mode** and validate its Linux input mapping.
The [8BitDo manual](https://manual.8bitdo.com/lite2/lite2-android-bluetooth.html)
describes powering on with Home and holding Pair for three seconds for initial
pairing. The [manufacturer product page](https://www.8bitdo.com/lite2/) lists
Raspberry Pi compatibility. The exact firmware/mode behavior on this Pi still
requires a physical test; do not assume an Xbox or Switch button numbering.

Proposed pairing behavior:

1. At service startup, load the saved designated controller and reconcile it
   with BlueZ's bonds. Recover a single matching existing bond where possible.
2. If no designated controller is bonded, continuously offer automatic
   enrollment through repeated discovery windows with retry backoff. Putting
   the Lite 2 into pairing mode should be the only normal user action.
3. Match the expected controller name plus device/service information. Pair one
   matching candidate through a headless pairing agent, trust the successful
   bond, connect its input profile, and persist its identity.
4. Mark the controller ready only when its Linux input device is available and
   its expected axes/buttons have been validated. Bluetooth `Connected` alone
   is insufficient.
5. After enrollment, stop accepting replacement controllers automatically. An
   already paired controller being switched off is a reconnect state, not an
   invitation to pair a different device.
6. On later power-on, accept the bonded controller's reconnect and attempt
   reconnection with backoff when needed. Preserve the bond across restarts,
   Pi reboots, and temporary signal loss; do not repeatedly unpair/re-pair it.
7. If bonding was lost, allow re-enrollment of the saved controller when it is
   put back in pairing mode. Deliberate replacement uses a future “forget
   controller” action; routine startup never needs it.

BlueZ provides discovery, pairing, trust, connection, property notifications,
and pairing-agent APIs. Use those interfaces instead of scripting an interactive
terminal session. See its [Adapter](https://bluez.readthedocs.io/en/latest/adapter-api/),
[Device](https://bluez.readthedocs.io/en/latest/device-api/), and
[Agent](https://bluez.readthedocs.io/en/latest/agent-api/) documentation.

The unattended agent accepts only the intended enrollment candidate or saved
identity and appropriate input services. It must not approve arbitrary nearby
devices. If multiple indistinguishable Lite 2 candidates are present, keep the
rover stopped and show the ambiguity rather than select one arbitrarily. For
normal first pairing, have only the intended Lite 2 in pairing mode nearby.

Expose separate states: searching, pairing, bonded/disconnected, connecting,
input-ready, and error. Retry transient failures automatically and recover after
Bluetooth service restarts. Keep the motors disarmed through pairing/reconnect;
successful reconnection must never replay the last stick position.

## 7. Master override and manual driving

The controller has priority over every other motion producer. A stop or active
hardware fault still means zero output; “override” does not bypass stop handling.

Priority from highest to lowest:

1. Latched stop, invalid command, or a fault that requires stopping.
2. Controller takeover/manual drive.
3. Explicitly enabled autonomy.
4. Idle: zero output.

Proposed operating states:

| State | Output and permitted transitions |
| --- | --- |
| Disarmed | Zero; boot, core restart, or controller reconnect starts here |
| Manual ready | Zero until the drive-enable control is held |
| Manual driving | Controller determines drive; releasing drive-enable stops |
| Following | Only the current autonomy session may request movement |
| Stopped/fault | Zero; an explicit re-arm is required after the cause clears |

Proposed physical button layout, subject to checking the actual input events:

| Input | Action |
| --- | --- |
| Left stick Y | Forward/reverse (implemented) |
| Right stick X | Left/right differential steering (implemented) |
| R shoulder, held | Manual drive enable; pressing it also takes over from autonomy |
| B | Immediate latched stop in any mode |
| Plus/Start | Re-arm to manual ready, with neutral sticks, R released, and faults cleared |
| X | Explicitly enter following from manual ready when all prerequisites pass |
| D-pad up/down | Select a bounded drive-power limit |

Any deliberate drive-stick movement beyond the takeover deadzone cancels
following immediately, even without R held. With R released it stops; with R
held the controller supplies the new drive request. Releasing R returns to zero
and manual ready. Returning the sticks to center never restarts autonomy.
Following must be enabled again explicitly. A camera or autonomy restart also
must not re-enable it.

Use calibrated axes, a deadzone with hysteresis, and bounded differential mixing.
Normalize combined throttle/turn input before scaling to the rover's allowed
left/right power. Start with a conservative power cap and measure low-speed
motor response; do not add an unexpected minimum-power jump. Smooth ordinary
changes but let stop requests bypass acceleration smoothing. These are power
limits until actual physical speeds have been measured.

The R/B/Start/X button actions in this proposed autonomy design are not enabled
in the current manual version. Current manual driving uses the two sticks and
the neutral interlock described above.

Proposed timing targets, all requiring later measurement:

| Item | Initial target |
| --- | --- |
| Core arbitration | 50 Hz, independent of camera and inference |
| Normal serial motion refresh | 20 Hz; immediate handling of stop/takeover |
| Input-service and autonomy request expiry | At most 250 ms without a valid producer update |
| Person source-frame age while following | At most 250 ms; independent of overlay expiry |
| Received controller event to UART command | At most 100 ms under full camera load |

Expire commands at the core, not only at their producer. A worker that is hung
must not leave the last nonzero request active. Stop on controller disconnect,
input-device removal, reader failure, or stale input-service health. Require a
connected, healthy controller throughout the first version of autonomous driving.
Stop autonomy if that prerequisite disappears.

Distinguish input-service health from radio health. Linux input events may be
silent while a stick is held steady; “no changed input for 250 ms” is not by
itself proof of disconnect. Conversely, repeatedly publishing a cached stick
state does not prove the controller is still reachable. Validate the Lite 2's
report behavior and BlueZ disconnect detection, and measure powered-off and
out-of-range stop latency. Do not claim the 250 ms producer timeout guarantees
the same bound for a silent radio failure.

If the whole Pi/control process fails, controller input cannot be handled by
software on it. The ESP32 movement watchdog is the fallback. The current app
requests 500 ms using the vendor-supported runtime setting; that physical timeout
has not yet been measured on this board. This software stop is not an independent
physical emergency-stop circuit.

## 8. Camera streaming and detection integration

Keep rovercam as the sole camera owner and share one inference pipeline between
overlays and autonomy. Extend it with a small local metadata and health publisher
during implementation. One dispatcher consumes detection results and distributes
them to both uses, avoiding competing reads from `DetectionWorker.latest()`.
Slow subscribers must not delay capture, inference, or streaming.

Proposed metadata contract:

| Field | Meaning |
| --- | --- |
| Stream generation and frame sequence | Identify restarts and reject duplicates/old results |
| Source-frame monotonic timestamp | Age of the image used for inference |
| Inference completion timestamp | Separate processing delay from frame age |
| Frame dimensions and transform | Define coordinates, borders, orientation, and crop |
| Person detections | Normalized boxes, confidence, class; empty list is a valid observation |
| Capture and inference health | Independent last-frame/result ages and error states |
| Recovery state | Restart count and whether frames are currently reliable |

The existing detector timestamp is a GStreamer buffer PTS, not automatically a
system monotonic timestamp. Map pipeline time to the Pi's monotonic clock and
handle pipeline resets explicitly. Do not stamp an old frame as fresh at IPC
receipt or inference completion. Invalidate all pending results and the follow
target on a camera generation change.

RTSP remains the first streaming interface, compatible with VLC/ffplay. A browser
viewer would require an additional supported delivery path, such as WebRTC;
an RTSP URL is not by itself browser playback. Add that only if a browser viewer
is chosen. No internet service is needed for local capture, inference, or driving.

Stream access is initially on the local network. The existing RTSP service binds
to all interfaces and permits unauthenticated viewing; record the intended
listener/access policy before exposing it more broadly. Remote internet viewing
is not a requirement currently.

On camera failure, status must show stale/unavailable video even if a player's
last image remains visible. Person-following stops using its own freshness timer;
it must not wait for rovercam's roughly two-second stall recovery threshold.
Direct manual driving can remain available independently of the camera, provided
the controller and rover connection are healthy.

Camera reliability is an explicit acceptance condition for following. Resolve
or sufficiently isolate the recorded stall and demonstrate stable operation
with movement/scene changes before treating mobile following as complete.

## 9. First autonomy behavior: follow a person

Implement a small worker that consumes detection metadata and submits expiring
drive intentions. It receives no direct motor access. Start disabled, and enable
only by an explicit controller action with fresh vision, a healthy rover link,
a connected controller, and no active stop.

Proposed initial behavior:

1. On enable, acquire a clearly distinguishable person near the image center
   over several fresh observations. If selection is ambiguous, remain stopped.
2. Maintain a local target track using position/box overlap and continuity.
   Person detection alone is not identity recognition; crossing people can
   remain ambiguous. Stop rather than silently follow a different person.
3. Steer from the target's horizontal offset. Reduce forward power while turning
   substantially and stop forward motion when alignment is inadequate.
4. Use apparent person-box size as an initial closeness signal with a tunable
   target band. Reduce forward power as the target fills that band; stop when
   close enough. Do not present this as distance in metres.
5. Begin with forward following only; do not reverse automatically when the
   target approaches the camera. Manual reverse remains available.
6. Stop and cancel following on missing, old, uncertain, or ambiguous target
   data, a camera restart, lost controller, lost rover connection, or override.
   Require explicit re-enable; do not keep driving during a search or silently
   reacquire another person.

No depth sensor, obstacle sensor, or wheel encoders have been confirmed. The first
behavior is therefore supervised visual following in a clear test area, without
claims of obstacle avoidance, stair detection, navigation, precise following
distance, or recognizing a particular individual. Reliable distance keeping and
obstacle avoidance would require further sensing and validation.

## 10. Startup, status, and recovery

The main user entry point is a Python master script, launched inside the project
virtual environment. It starts or attaches to the managed components and shows
the CLI status screen immediately, including while devices are still being
discovered. Running it a second time must not start competing UART owners or
camera captures. Exact script/module naming will be chosen during implementation.

Initially the master script can supervise separate control, Bluetooth/input,
vision, and autonomy processes. Later systemd can manage their boot lifecycle;
the master script then attaches to the same services for its interactive status
view. Avoid two competing process supervisors. Start pairing and controller
handling at boot without a desktop login once boot deployment is implemented.
Internet or Wi-Fi availability must not gate local gamepad control. Use bounded
restart policies so camera failures do not create an unlimited restart loop or
restart the control core.

Every control-core start initializes to disarmed, clears old sessions, establishes
the verified rover link, and requests zero output before enabling motion. On
serial reconnection repeat that sequence. Never replay motion saved before a
crash. On orderly shutdown, request zero output before closing UART; abrupt
failure still depends on the board's timeout.

Configuration should cover the hardware profile, UART device/baud, controller
identity/mode/mapping, deadzones, power limits, producer deadlines, camera device
and formats, model settings, metadata limits, and following thresholds. Keep
controller bond state separate from ordinary editable configuration. Do not
persist an armed or following state.

The CLI status screen is part of the first version, not deferred to a future web
interface. Show an at-a-glance overview with expandable subsystem sections:

| Section | Initial fields and future additions |
| --- | --- |
| Application | Startup progress, uptime, armed state, active authority, stop/fault reason |
| Rover | UART connection, last feedback age, battery voltage and available IMU data |
| 8BitDo | Searching/pairing/bonded/connected/input-ready state, device name, manual override availability |
| Camera | Detected or missing, opened or unavailable, capturing or stalled, selected format |
| Stream | Available or unavailable, RTSP address, output FPS |
| Detection | Enabled/disabled/loading/error, person count, detection FPS and source-frame age |
| Autonomy | Disabled/ready/following/stopped, selected target, reason following is unavailable |
| Events | Recent connections, recoveries, mode changes, and errors |

Keep device presence separate from usability: a plugged-in camera can be stalled,
and a Bluetooth connection may not yet have a usable input device. Missing or
unimplemented telemetry shows “unknown”, “unavailable”, or “not implemented”,
never a fabricated zero or healthy state. Display units and freshness for sensor
values, and do not invent a battery percentage from uncalibrated voltage.

Refresh the terminal view at a modest rate, initially around 2 Hz, without
blocking the control loop. Keep logs from corrupting the display; use a recent
event area and a separate log destination. When output is redirected or a terminal
cannot redraw, use periodic plain-text snapshots. New features add status fields
through a common snapshot interface rather than building their own displays.

Ctrl+C should exit through an explicit lifecycle path: when the master owns the
processes, request stop and shut them down cleanly; when attached to persistent
services, request disarm before detaching. Closing a terminal must never leave an
old manual motion request valid. Document the chosen behavior in the eventual CLI.

Mirror a concise subset on the OLED where supported. A web interface is optional
and would use the same status data and motion priority rules. Log mode changes
and faults with timestamps; avoid logging every frame or flooding UART with
display updates.

## 11. Development stages and acceptance criteria

These are future implementation and validation steps, not actions performed now.

| Stage | Work | Completion evidence |
| --- | --- | --- |
| 1. Establish hardware profile | Confirm wiring, UART mapping, firmware, telemetry, power, controller mode, and camera identity; inspect other UART users and firmware command paths | Recorded working profile; no unsupported chassis assumptions; Bluetooth retained |
| 2. Core, master script, and stop rules | Establish Python/venv, master launch, initial CLI status, one serial owner, authority state machine, bounded motion, and stop handling | Status appears with missing/disconnected devices; simulated tests reject stale commands; controlled motor tests verify direction, zero output, and firmware timeout |
| 3. Controller lifecycle | Automatic enrollment, persistent bonding, reconnection, calibrated inputs, manual driving | First pairing needs only controller pairing mode; power cycling either side reconnects; reconnect remains disarmed |
| 4. Camera integration | Preserve existing pipeline, add metadata/health, investigate the recorded capture stall | RTSP and person boxes work; source timestamps remain valid across recovery; slow viewers cannot block control |
| 5. Following | Target selection/continuity, bounded steering and approach, controller cancellation | Target loss stops; close target stops approach; ambiguous people do not cause silent target switching |
| 6. Boot and combined operation | Service lifecycle, status, long-running load and failure checks | No login needed; repeated startup/recovery never resumes movement; camera load does not defeat override |

Essential behavioral checks during those stages:

- Use a simulated serial sink first, then restrained/wheels-clear tests before
  ground driving. Verify the plain rover's command scale and motor deadband.
- Pair with no prior bond; reconnect after controller power-off, Pi reboot,
  Bluetooth restart, and loss of range. Check incorrect and ambiguous devices.
- Hold a stick steadily to distinguish input silence from a lost controller.
  Measure real radio-loss-to-stop latency, not only an injected disconnect event.
- Override during following while the camera, inference, or autonomy is slow or
  hung. Check that delayed autonomy messages cannot regain authority.
- Flood normal requests and verify stop bypasses pending motion and smoothing.
  Releasing R stops; centering the sticks does not resume following.
- Disconnect the camera, send empty detections, delay metadata, restart the
  pipeline, and present multiple crossing people. Check source age and target loss.
- Interrupt UART, restart the core, and terminate it abruptly. Verify the actual
  firmware timeout separately from the normal host stop path.
- Test 30 FPS video/15 FPS detection under combined control load; record control
  latency, CPU/temperature, capture stalls, and physical stopping distance.
  Historical rovercam measurements are a baseline, not a passing integration test.
- Repeat scene-change and camera-movement tests long enough to expose the known
  stall; a short successful stream alone does not close that issue.
- Launch the master script with no controller or camera, then connect them and
  verify status transitions without restarting the screen. Check missing/stale
  telemetry, redirected output, component failure, duplicate launch, and clean
  exit while preserving the control-loop latency target.

## 12. Decisions and measurements still open

The requirements are sufficient to plan the architecture. The remaining items
can be resolved during review and implementation:

- Final controller button preferences; the proposed R/B/Start/X mapping is not
  yet a user-selected mapping.
- Exact physical wiring, UART device, firmware version, command support/units,
  exclusive motion authority, and Pi/rover power arrangement.
- Lite 2 mode and firmware, pairing-agent interaction, input identity, radio-loss
  detection behavior, and reconnect timing on this Pi.
- First-version power cap, calibrated motor response, and measured stop latency.
- Camera stall cause and the stable operating configuration.
- Desired visual following distance and how to select among multiple people;
  box-size control cannot supply a calibrated metric distance by itself.
- Whether RTSP viewing is sufficient or a browser viewer/status page is wanted.
- Whether future autonomy may run without the controller; the proposed first
  version stops if the controller is unavailable to preserve supervised override.

## 13. Repository

The application has its own repository rooted at `/home/mieszko/apptirover`.
The requested origin is
[mieszkomularczyk/apptirover](https://github.com/mieszkomularczyk/apptirover),
using HTTPS. The initial commit contains this planning document only.
The adjacent `rovercam` and `ugv_rpi` directories are local references, not
vendored project files; links to them require that workspace layout and will not
resolve as files within the GitHub repository.

GitHub HTTPS authentication was configured through GitHub CLI when publishing the
initial document. Do not store GitHub tokens or other credentials in the project
or document. Implementation changes are now being developed in this repository.

The original design review used local files and manufacturer/library
documentation. The current milestones additionally verify real serial telemetry,
automatic controller pairing/reconnection, joystick/button readings, and manual
motor control with the wheels raised. Ground driving, camera integration, and
autonomy are not validated. Neither adjacent software project was modified.
