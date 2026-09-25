import io
import json
import os
import pty
import select
import threading
import time
import unittest

import serial
from rich.console import Console

from apptirover.protocol import JsonLines
from apptirover.serial_monitor import SerialMonitor
from apptirover.status import render
from apptirover.telemetry import Telemetry


# Representative replies captured from this WAVE ROVER on 2026-09-25.
BASE = {"T": 1001, "L": 0, "R": 0, "r": -0.155141413, "p": 0.297435671,
        "y": -114.9006348, "temp": 57.77777863, "v": 12.26241016}
IMU = {"T": 1002, "r": -0.364729017, "p": 0.306781083, "y": -114.6069641,
       "ax": 13.33007813, "ay": -6.26953125, "az": 986.7675781,
       "gx": -0.486249983, "gy": 5.011250019, "gz": 0.77125001,
       "mx": 37, "my": -76, "mz": -145, "temp": 57.77777863}


class DecoderTests(unittest.TestCase):
    def test_split_and_combined_lines_with_echo_and_noise(self):
        decoder = JsonLines()
        self.assertEqual(decoder.feed(b'boot message\r\n{"T":10'), [])
        self.assertEqual(decoder.feed(b'01,"v":12.2}\r\n{"T":130}\n'),
                         [{"T": 1001, "v": 12.2}, {"T": 130}])
        self.assertEqual(decoder.invalid_lines, 1)

    def test_reject_nonfinite_nonobjects_and_invalid_types(self):
        decoder = JsonLines()
        invalid = b'[]\n{"T":true}\n{"T":1001,"v":NaN}\n{"T":1001,"v":1e999}\n\xff\n'
        self.assertEqual(decoder.feed(invalid), [])
        self.assertEqual(decoder.invalid_lines, 5)

    def test_oversized_line_cannot_leave_a_valid_looking_tail(self):
        decoder = JsonLines(max_line=32)
        self.assertEqual(decoder.feed(b'x' * 100), [])
        self.assertLessEqual(len(decoder.buffer), 32)
        self.assertEqual(decoder.feed(b'{"T":1001,"v":99}\n{"T":1001,"v":12}\n'),
                         [{"T": 1001, "v": 12}])
        self.assertEqual(decoder.invalid_lines, 1)


class StateTests(unittest.TestCase):
    def setUp(self):
        self.state = Telemetry('/dev/fake')
        self.state.opened(10)

    def test_echoes_and_empty_telemetry_do_not_prove_communication(self):
        for value in ({"T": 130}, {"T": 126}, {"T": 1001}, {"T": 1001, "v": None}):
            self.state.accept(value, 11)
        snapshot = self.state.snapshot(12)
        self.assertEqual(snapshot['state'], 'waiting for telemetry')
        self.assertEqual(snapshot['echoes'], 2)
        self.assertEqual(snapshot['telemetry_packets'], 0)

    def test_freshness_is_per_reply_type(self):
        self.state.accept(BASE, 11)
        self.state.accept(IMU, 12)
        self.assertEqual(self.state.snapshot(12)['state'], 'telemetry live')
        self.assertEqual(self.state.snapshot(14.5)['state'], 'partial telemetry')
        self.assertEqual(self.state.snapshot(16)['state'], 'telemetry stale')

    def test_reconnect_invalidates_old_data_without_hiding_it(self):
        self.state.accept(BASE, 11)
        self.state.accept(IMU, 11)
        self.state.connected = False
        self.state.opened(11.1)
        snapshot = self.state.snapshot(11.2)
        self.assertEqual(snapshot['state'], 'telemetry stale')
        self.assertFalse(snapshot['packets']['1001']['fresh'])
        self.assertEqual(snapshot['packets']['1001']['data']['v'], BASE['v'])

    def test_omitted_fields_are_not_relabelled_as_fresh(self):
        self.state.accept(BASE, 11)
        self.state.accept({'T': 1001, 'L': 0, 'R': 0}, 12)
        self.assertNotIn('v', self.state.snapshot(12)['packets']['1001']['data'])

    def test_unknown_reply_storage_is_bounded(self):
        for kind in range(2000, 2100):
            self.state.accept({'T': kind, 'detail': 'new field'}, 11)
        self.assertEqual(len(self.state.snapshot(12)['other_packets']), 16)

    def test_status_uses_volts_and_preserves_unknown_sensor_fields(self):
        self.state.accept({**BASE, 'extra_sensor': 42}, 11)
        self.state.accept(IMU, 11)
        output = io.StringIO()
        console = Console(file=output, width=130, color_system=None)
        host = {'cpu_temperature_c': 50.2, 'load_1m': 0.1, 'usb_cameras': {}}
        console.print(render(self.state.snapshot(12), host, 2))
        text = output.getvalue()
        self.assertIn('12.26 V', text)
        self.assertIn('extra_sensor', text)
        self.assertIn('not reported', text)
        self.assertNotIn('0.12 V', text)


class SerialTests(unittest.TestCase):
    def test_real_serial_transport_queries_only_handles_fragments_and_expires(self):
        master, slave = pty.openpty()
        path = os.ttyname(slave)
        monitor = SerialMonitor(path, poll_interval=0.1)
        stopped = threading.Event()
        replies_enabled = threading.Event()
        replies_enabled.set()
        commands = []
        errors = []

        def fake_rover():
            pending = b''
            try:
                while not stopped.is_set():
                    if not select.select([master], [], [], 0.1)[0]:
                        continue
                    pending += os.read(master, 4096)
                    while b'\n' in pending:
                        line, pending = pending.split(b'\n', 1)
                        command = json.loads(line)
                        commands.append(command)
                        if not replies_enabled.is_set():
                            continue
                        data = BASE if command['T'] == 130 else IMU
                        encoded = json.dumps(data).encode() + b'\r\n'
                        os.write(master, line + b'\n' + encoded[:17])
                        time.sleep(0.005)
                        os.write(master, encoded[17:])
            except Exception as exc:
                errors.append(exc)

        worker = threading.Thread(target=fake_rover)
        worker.start()
        monitor.start()
        try:
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline and monitor.snapshot()['state'] != 'telemetry live':
                time.sleep(0.02)
            snapshot = monitor.snapshot()
            self.assertEqual(snapshot['state'], 'telemetry live')
            self.assertEqual(snapshot['packets']['1001']['data']['v'], BASE['v'])
            self.assertGreater(snapshot['echoes'], 0)
            with self.assertRaises(serial.SerialException):
                with serial.Serial(path, 115200, exclusive=True):
                    pass
            replies_enabled.clear()
            deadline = time.monotonic() + 4
            while time.monotonic() < deadline and monitor.snapshot()['state'] != 'telemetry stale':
                time.sleep(0.05)
            self.assertEqual(monitor.snapshot()['state'], 'telemetry stale')
        finally:
            monitor.close()
            stopped.set()
            worker.join(timeout=1)
            os.close(slave)
            os.close(master)
        self.assertFalse(monitor.thread.is_alive())
        self.assertEqual(errors, [])
        self.assertTrue(commands)
        self.assertTrue(all(command in ({'T': 130}, {'T': 126}) for command in commands))

    def test_unavailable_port_shows_error_and_shutdown_interrupts_retry(self):
        monitor = SerialMonitor('/nonexistent/apptirover-test-uart')
        monitor.start()
        try:
            deadline = time.monotonic() + 1
            while not monitor.snapshot()['last_error'] and time.monotonic() < deadline:
                time.sleep(0.01)
            snapshot = monitor.snapshot()
            self.assertEqual(snapshot['state'], 'disconnected')
            self.assertTrue(snapshot['last_error'])
        finally:
            monitor.close()
        self.assertFalse(monitor.thread.is_alive())


if __name__ == '__main__':
    unittest.main()
