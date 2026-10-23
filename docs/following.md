# Person tracking and following

## Controller operation

Launch `.venv/bin/python main.py` as usual. Following starts **disabled**.
After the controller and rover are ready and the driving axes are neutral:

1. Tap controller **X** to start acquisition. The rover stays stopped while a
   persistent person is selected from roughly one second of fresh observations.
2. Once locked, the rover centers that person and drives forward or **reverses**
   to keep their apparent width near the configured setpoint.
3. Tap **X** again to stop following. Moving left-stick Y or right-stick X also
   cancels following and gives manual control priority. Centering the sticks
   does not restart following; another X press is required.

X is bound to Linux `BTN_NORTH` by default. Presses are counted by the input
reader, so a short released tap is retained between control ticks and holding X
does not repeatedly toggle. Startup/reconnection, already-held buttons and
dropped-event resynchronization cannot arm following. The physical X mapping on
this controller/firmware still needs confirmation: use `--controller-only` and
the keys row/JSON to inspect it; `--follow-button` overrides the Linux binding.
The existing two-stick manual mapping needs no shoulder button.

The CLI shows following state, target ID and width; larger terminals also show
target center, source age and desired width. JSON retains the complete
`rover.drive.following` status and stop reason. The displayed motor limit uses
the currently active manual/following cap. The complete remote ffplay command
remains above the table. Video boxes show person IDs, with amber `?` labels for
ambiguous associations.

Camera and detection must already be ready to activate following. If detection
is disabled, enable it with terminal **D** first, then press X. Disabling
detection cancels following immediately. `--no-drive`, `--no-controller`,
`--controller-only` and `--camera-only` never enable autonomous motor writes.

```bash
.venv/bin/python main.py
.venv/bin/python main.py --follow-width .20 --follow-power .25
.venv/bin/python main.py --no-drive --json  # Observe detections/tracks; no motor writes
```

`--follow-width` is normalized image width, default **0.15** (96 pixels at a
640-pixel output width); supported range is 0.03–0.8. A larger setpoint asks the
rover to stay closer for the same person/camera geometry. It is not distance in
metres. Person size, orientation, camera field of view and partial visibility
affect the signal. Horizontal clipping prevents a target from authorizing motion.

`--follow-power` defaults to **0.25** and is additionally capped by `--max-power`.
It limits normalized PWM motor power, not measured physical speed. Ordinary
changes use the existing motor ramp; stops bypass it. Gains and power need
physical tuning on this rover. Automatic reverse is enabled as requested.

## Architecture and tracking algorithm

```mermaid
flowchart LR
    Camera[Camera and RTSP process] -->|latest small RGB frame| AI[NCNN inference process]
    AI --> Associate[Two-stage association and Kalman filters in same process]
    Associate -->|timestamped tracks| Snapshot[Main-process camera snapshot]
    Snapshot --> Authority[UART control loop: target selection and bounded PD]
    Pad[Controller input and counted X presses] --> Authority
    Authority --> Motors[Single UART owner]
```

The reference analysis is in [person-following-reference.md](person-following-reference.md).
The Pi uses the existing NCNN model, with no TensorRT, ROS, PyTorch, Ultralytics
runtime or new dependency. [tracking.py](../apptirover/tracking.py) implements a
small **ByteTrack-style** association pipeline rather than importing the exact
Ultralytics implementation:

- Source-time constant-velocity Kalman filters estimate center X/Y, width/height
  and their rates. All prediction, covariance updates and association run in the
  inference process, away from UART and Bluetooth execution.
- Global minimum-cost assignment matches strong person detections using box IoU,
  confidence, a center innovation gate and size consistency. Remaining active
  tracks can match weaker boxes with a stricter IoU gate. Weak boxes cannot start
  a new track or revive an already lost track.
- High threshold is the configured `--confidence` (default 0.35), low threshold
  is `min(0.1, confidence/2)`, and new-track threshold is `max(0.5, confidence)`.
  Detector decoding keeps the low-score interval for association; the normal
  `boxes`/`people` display still applies `--confidence`.
- Empty frames advance prediction and retention too. On each observation,
  unmatched tracks expire 0.75 seconds after their last measurement, with IDs never
  reused within the detector process. Each distinct source frame is processed
  once. Camera generations reset association state.
- Work is bounded to 64 detections and 32 retained tracks. Close competing
  assignments and overlapping measured people flag ambiguity. There are no
  appearance embeddings or guarantees of recognizing a particular individual.

[following.py](../apptirover/following.py) consumes these small snapshots. During
acquisition, it keeps bounded timestamped candidate votes. After about one
second, a current confirmed candidate must occur in at least 80% of that window's
fresh observations (and at least three observations). Candidates rank by count,
then mean confidence, as in the reference. Equal-count contenders whose mean
scores differ by less than 0.08 leave acquisition stationary. Only the committed
winner's box seeds control. Repeated snapshots add no votes.

After acquisition the ID remains locked. A different nearby ID is not silently
adopted. A selected target must be confirmed, measured in the current source
frame, finite, geometrically valid and unambiguous. Prediction preserves identity
memory and helps association; it does not authorize motion by itself.

The controller uses two bounded PD signals, adapted from the example's PID idea:

```text
turn = 2.5 * (center_x - 0.5) + 0.12 * center_x_rate
throttle = 0.7 * log(desired_width / width) - 0.05 * width_rate / width
```

Positive turn points the rover's nose right. Smaller boxes request forward
movement; larger boxes request reverse. Center error within 0.03 suppresses yaw;
width ratios within 10% suppress translation. Translation decreases with
horizontal error and reaches zero at an offset of 0.3. Both axes are bounded
before existing differential mixing and motor-power limits. There is no
integrator to wind up while acquiring/stopped, no metric depth estimate and no
obstacle input.

## Freshness and authority

[DriveControl](../apptirover/drive.py) is the sole motion arbiter, executed in the
existing serial worker at up to 50 Hz. Health faults stop first, manual input
cancels following, and explicitly enabled following may otherwise propose motion.
Vision never opens the UART. Snapshot reads use local main-process state, with no
cross-process lock or synchronous inference request. A camera snapshot exception
revokes following while serial telemetry/manual control continue.

Motion expires **250 ms after the real source frame**, even between camera
callbacks. This is stricter than the 500 ms display-box expiry. Capture/completion
timestamps, increasing sequences and process/camera/detector generations are
checked. A heartbeat or Kalman prediction cannot refresh measurement age.

Target absence, predicted-only/invalid/ambiguous boxes, stale input, detection
disable, changed camera/detector generation, controller loss, dropped events,
UART health failure or a control-loop stall stop/cancel following. Camera and
detector recovery never re-enable it automatically. The board watchdog and
existing neutral interlock remain additional controls. None of these software
checks establishes the physical board-watchdog or silent radio-loss timing.

## Verification

```bash
.venv/bin/python -m unittest discover -s tests -v
.venv/bin/python tools/check_camera.py --image /path/to/one-centered-person.jpg --follow-check
```

The integration tool opens a pseudo-UART and a loopback RTSP server, with real
GStreamer/NCNN and a still-image source. It never opens real UART, Bluetooth or
USB hardware. `--follow-check` uses a test setpoint of 0.08 to exercise reverse,
X acquisition, manual takeover and detection-disable cancellation. Ordinary
checks also cover detector/camera crash, freeze, recovery and final motor zero.
The image must contain one distinguishable centered person wide enough to
trigger reverse at that test setpoint.

On 2026-10-23, 74 automated tests cover tracking, following, input/transport
failures and the existing application. The real-process following check passed:
407 simulated telemetry replies, 287 decoded frames before camera restart,
control-loop gap median 20.4 ms, p99 23.9 ms and maximum 26.3 ms.
A synthetic 64-detection/32-track
association benchmark measured 6.15 ms median and 7.87 ms maximum over 100
updates. These are samples on this Pi, not scheduling guarantees or physical
following measurements. The physical X mapping, motor response/gains and mobile
following remain to be checked with the user.
