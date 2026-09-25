"""One serial owner. This implementation sends telemetry queries only."""

import threading
import time

import serial

from .protocol import BASE_QUERY, IMU_QUERY, JsonLines
from .telemetry import Telemetry


class SerialMonitor:
    def __init__(self, port, baud=115200, poll_interval=0.5, serial_factory=serial.Serial):
        self.state = Telemetry(port, baud)
        self.poll_interval = poll_interval
        self.serial_factory = serial_factory
        self.lock = threading.Lock()
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self.run, name="rover-telemetry", daemon=True)

    def start(self):
        self.thread.start()

    def snapshot(self):
        with self.lock:
            return self.state.snapshot(time.monotonic(), max(3.0, self.poll_interval * 3))

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
                        self.state.port, self.state.baud, timeout=0.05,
                        write_timeout=0.3, exclusive=True,
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

    def read_connection(self, port):
        parser = JsonLines()
        # Give a newly opened port a short settle period; retry queries indefinitely.
        next_query = time.monotonic() + 0.2
        query = BASE_QUERY
        previous_invalid = 0
        while not self.stopping.is_set():
            now = time.monotonic()
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
