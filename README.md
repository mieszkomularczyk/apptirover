# apptirover

Live UART telemetry and 8BitDo Lite 2 Bluetooth inputs on this Raspberry Pi 5.
The app pairs and reconnects the controller automatically and displays its sticks,
buttons, and D-pad beside the plain WAVE ROVER's sensor data.
**Motor control remains disabled; UART output contains only telemetry queries.**

## Run

The Python virtual environment is already prepared on this Pi:

```bash
cd /home/mieszko/apptirover
.venv/bin/python main.py
```

The screen updates in place. Press **Ctrl+C** to close the serial connection and
exit. The default is `/dev/ttyAMA0`, 115200 baud. On this Pi `/dev/serial0` is the
separate debug UART and is not the rover connection. Your account already has
the required `dialout` membership; normal use does not require `sudo`.

The display includes battery voltage, roll/pitch/yaw, acceleration, angular-rate
and magnetic readings, ESP32 temperature, reported left/right motor fields, Pi
temperature/load, USB camera presence, reply ages, request/echo counters, and
serial errors. Unknown reply fields are retained. The camera is not opened.

Battery `v` is **already in volts** on the verified firmware. Percentage,
charging state, and current are not exposed by the two telemetry replies and are
not estimated. IMU axes retain the firmware's values without speculative unit
conversion. The motor fields are not encoder measurements of wheel speed.

## Pair and read the Lite 2

Keep the app running. For first pairing, set the controller's switch to **D**,
press **Home**, and hold the **pairing button for about 3 seconds**. With one
Lite 2 in pairing mode nearby, the app discovers, pairs, trusts, and connects it
without prompts on the Pi. See the manufacturer's
[D-mode instructions](https://manual.8bitdo.com/lite2/lite2-android-bluetooth.html).

For later use, press **Home** in D mode. The saved controller reconnects without
pairing again. The screen distinguishes Bluetooth connection from **input ready**
and shows left/right X/Y values, pressed Linux button codes, D-pad values, and
the number and latest input event. Stick values use an 8% center deadzone and
range from -1 to +1; negative Y is up. JSON also includes raw axes, kernel ranges,
observed minimum/maximum values, input-device path, and Bluetooth address.
No button or stick has a driving action yet.

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
silence alone is not a disconnect. This is an input diagnostic, not a measured
radio-loss stop mechanism for driving. Controller battery percentage appears only
if BlueZ reports it; it was unavailable on this Lite 2 in D mode.

The current account has `input` membership and BlueZ access; no `sudo` is needed
for normal use. Bluetooth must be unblocked. Only one apptirover controller monitor
may run at a time. No automatic boot service is installed; launch `main.py`
to enable enrollment and reconnect supervision.

## Diagnostics

```bash
# Bounded real-rover communication check:
.venv/bin/python main.py --no-controller --duration 10 --plain

# Bluetooth/input check without opening the rover UART:
.venv/bin/python main.py --controller-only --refresh 5

# All returned telemetry fields as machine-readable snapshots:
.venv/bin/python main.py --duration 10 --json

# Alternate UART, only if your wiring/configuration differs:
.venv/bin/python main.py --port /dev/ttyAMA0 --baud 115200
```

`--duration` exits with code 0 only when every enabled subsystem is ready at the
end: fresh chassis/IMU feedback and controller input ready. Code 2 indicates a
subsystem is unavailable. `--no-controller` requires only rover feedback;
`--controller-only` requires only controller input. A monitor failure returns 1.
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
(default 2 Hz) and keeps
all reply fields, useful on narrow terminals. Nothing is logged to disk by default.

## Recreate the environment and test

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python -m unittest discover -s tests -v
```

Tests use captured reply shapes and a simulated rover over a pseudo-terminal.
They check fragmented/corrupt input, query-only output, echoes, exclusive access,
stale telemetry, reconnect generations, missing hardware, and voltage display.
Controller tests also cover scoped pairing, ambiguous devices, saved-address
reconnection, Bluetooth input identity, normalization, button release, dropped
event recovery, and clearing controls on disconnect. Tests do not move the real
rover or access Bluetooth.

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
All 20 automated tests and the dependency consistency check pass.

The requested startup movement test is deferred while verifying communications.
A strict 5 cm limit cannot be guaranteed by a timed pulse on this encoderless
chassis. See [apptirover.md](apptirover.md) for the full design and future stages.
