"""One serial owner for telemetry and bounded, continuously refreshed motor control."""

import threading
import time

import serial

from .protocol import BASE_QUERY, IMU_QUERY, JsonLines, STOP_COMMAND, WATCHDOG_COMMAND, motor_command
from .telemetry import Telemetry
from .drive import DriveControl


class SerialMonitor:
    def __init__(self, port, baud=115200, poll_interval=0.5, serial_factory=serial.Serial,
                 controller_source=None, max_power=0.5):
        self.state = Telemetry(port, baud)
        self.poll_interval = poll_interval
        self.serial_factory = serial_factory
        self.controller_source = controller_source
        self.drive = DriveControl(max_power) if controller_source else None
        self.drive_status = self.drive.snapshot() if self.drive else dict(enabled=False, armed=False, state='disabled')
        self.motion_commands = 0
        self.lock = threading.Lock()
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self.run, name="rover-telemetry", daemon=True)

    def start(self):
        self.thread.start()

    def snapshot(self):
        with self.lock:
            snapshot = self.state.snapshot(time.monotonic(), max(3.0, self.poll_interval * 3))
            snapshot['drive'] = dict(self.drive_status, commands=self.motion_commands)
            return snapshot

    def close(self):
        self.stopping.set()
        if self.thread.ident is not None:
            self.thread.join(timeout=2)

    def run(self):
        try:
            while not self.stopping.is_set():
                try:
                    # Exclusive flock prevents cooperating instances sharing the UART.
                    with self.serial_factory(
                        self.state.port, self.state.baud, timeout=0.01,
                        write_timeout=0.1, exclusive=True,
                    ) as port:
                        with self.lock:
                            self.state.opened(time.monotonic())
                        self.read_connection(port)
                except (OSError, serial.SerialException) as exc:
                    with self.lock:
                        self.state.connected = False
                        self.state.last_error = str(exc)
                    self.stopping.wait(2.0)
        except Exception as exc:
            # Make an unexpected reader failure visible instead of freezing a green UI.
            with self.lock:
                self.state.last_error = f"Reader failed: {type(exc).__name__}: {exc}"
        finally:
            with self.lock:
                self.state.connected = False

    def write_control(self, port, command):
        if port.write(command) != len(command):
            raise serial.SerialException('Incomplete motor control write')
        with self.lock:
            self.motion_commands += 1

    def read_connection(self, port):
        try:
            if self.drive:
                self.drive.stop('center sticks after connection')
                self.write_control(port, STOP_COMMAND)
                self.write_control(port, WATCHDOG_COMMAND)
            self.poll_connection(port)
        finally:
            if self.drive:
                self.drive.stop('stopped; serial connection closing')
                with self.lock:
                    self.drive_status = self.drive.snapshot()
                # Explicit zero on every ordinary exit/error; board timeout covers loss of UART.
                try:
                    self.write_control(port, STOP_COMMAND)
                except (OSError, serial.SerialException) as error:
                    with self.lock:
                        self.state.last_error = f'Could not send final stop: {error}'

    def poll_connection(self, port):
        parser = JsonLines()
        # Give a newly opened port a short settle period; retry queries indefinitely.
        next_query = time.monotonic() + 0.2
        query = BASE_QUERY
        previous_invalid = 0
        next_control = next_motion = 0
        last_power = (0, 0)
        while not self.stopping.is_set():
            now = time.monotonic()
            if self.drive and now >= next_control:
                # Pull current state in the serial worker, independently of terminal rendering.
                pad, rover = self.controller_source(), self.snapshot()
                now = time.monotonic()
                control = self.drive.step(pad, rover, now)
                with self.lock:
                    self.drive_status = control
                power = (control['left_power'], control['right_power'])
                if now >= next_motion or (power == (0, 0) and last_power != (0, 0)):
                    self.write_control(port, motor_command(*power))
                    last_power = power
                    next_motion = now + 0.05
                next_control = now + 0.02
            if now >= next_query:
                if port.write(query) != len(query):
                    raise serial.SerialException("Incomplete telemetry query write")
                with self.lock:
                    self.state.tx_queries += 1
                query = IMU_QUERY if query == BASE_QUERY else BASE_QUERY
                next_query = time.monotonic() + self.poll_interval / 2
            # A bounded read and no outgoing queue keep shutdown and polling responsive.
            data = port.read(max(1, min(port.in_waiting, 4096)))
            if not data:
                continue
            messages = parser.feed(data)
            now = time.monotonic()
            with self.lock:
                self.state.rx_bytes += len(data)
                self.state.invalid_lines += parser.invalid_lines - previous_invalid
                previous_invalid = parser.invalid_lines
                for message in messages:
                    self.state.accept(message, now)
