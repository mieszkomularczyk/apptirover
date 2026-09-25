import copy
import io
import json
import os
import pty
import select
import threading
import time
import unittest

from rich.console import Console

from apptirover.drive import DriveControl, mix
from apptirover.protocol import STOP_COMMAND, motor_command
from apptirover.serial_monitor import SerialMonitor
from apptirover.status import render
from test_telemetry import BASE, IMU


def pad(throttle=0, turn=0):
    return dict(connected=True, input_ready=True, input_generation=1, worker_age_s=0,
                link_age_s=0, input=dict(sticks=dict(left_x=0, left_y=-throttle,
                right_x=turn, right_y=0), resyncing=False))


def rover():
    return dict(port_open=True, generation=1, packets={'1001': dict(fresh=True, age_s=0)})


class DriveTests(unittest.TestCase):
    def ready(self):
        control = DriveControl(0.5)
        control.step(pad(), rover(), 10)
        for now in (10.1, 10.2, 10.3):
            control.step(pad(), rover(), now)
        self.assertTrue(control.armed)
        return control

    def test_proportional_mix_forward_reverse_spin_and_combined(self):
        self.assertEqual(mix(1, 0, .5), (.5, .5))
        self.assertEqual(mix(-1, 0, .5), (-.5, -.5))
        self.assertEqual(mix(0, 1, .5), (.5, -.5))
        self.assertEqual(mix(0, -1, .5), (-.5, .5))
        self.assertEqual(mix(.5, .25, .5), (.375, .125))
        self.assertEqual(mix(1, 1, .5), (.5, 0))
        self.assertEqual(mix(-1, 1, .5), (0, -.5))
        for throttle in (-1, -.5, 0, .5, 1):
            for turn in (-1, -.5, 0, .5, 1):
                self.assertLessEqual(max(map(abs, mix(throttle, turn, .5))), .5)

    def test_wire_values_are_floats_bounded_to_8_bit_pwm(self):
        self.assertEqual(STOP_COMMAND, b'{"T":1,"L":0.0,"R":0.0}\n')
        full = json.loads(motor_command(1, -1))
        self.assertLessEqual(round(full['L'] * 512), 255)
        self.assertGreaterEqual(round(full['R'] * 512), -255)
        for value in (float('nan'), float('inf'), True, 1.1):
            with self.assertRaises(ValueError):
                motor_command(value, 0)

    def test_startup_and_reconnect_require_neutral_dwell(self):
        control = DriveControl()
        self.assertFalse(control.step(pad(1), rover(), 10)['armed'])
        self.assertEqual(control.left, 0)
        control.step(pad(), rover(), 10.1)
        self.assertFalse(control.step(pad(), rover(), 10.2)['armed'])
        control.step(pad(), rover(), 10.3)
        self.assertTrue(control.step(pad(), rover(), 10.4)['armed'])
        reconnected = pad(1)
        reconnected['input_generation'] = 2
        self.assertFalse(control.step(reconnected, rover(), 12)['armed'])
        self.assertEqual(control.left, 0)

    def test_drive_loop_pause_cannot_resume_old_held_input(self):
        control = self.ready()
        control.step(pad(1), rover(), 10.4)
        result = control.step(pad(1), rover(), 11.4)
        self.assertFalse(result['armed'])
        self.assertEqual(result['left_power'], 0)
        self.assertIn('stalled', result['state'])
        self.assertFalse(control.step(pad(1), rover(), 11.5)['armed'])

    def test_faults_stop_without_smoothing_and_latch_neutral_interlock(self):
        cases = [dict(connected=False), dict(input_ready=False), dict(worker_age_s=.3),
                 dict(link_age_s=.8), dict(input=None)]
        for changes in cases:
            control = self.ready()
            control.step(pad(1), rover(), 10.4)
            self.assertGreater(control.left, 0)
            self.assertEqual(control.step(dict(pad(1), **changes), rover(), 10.42)['left_power'], 0)
            self.assertFalse(control.step(pad(1), rover(), 10.5)['armed'])
        for bad_rover in (dict(rover(), port_open=False), dict(rover(), generation=2),
                          dict(rover(), packets={'1001': dict(fresh=True, age_s=1.1)})):
            control = self.ready()
            self.assertFalse(control.step(pad(1), bad_rover, 10.4)['armed'])

    def test_stick_center_stops_immediately_and_unchanged_input_stays_valid(self):
        control = self.ready()
        moving = pad(.5, .25)
        moving['input']['last_event_age_s'] = 100  # A held stick need not produce changes.
        for index in range(1, 101):
            result = control.step(moving, rover(), 10.3 + index * .02)
        self.assertEqual((result['left_power'], result['right_power']), (.375, .125))
        self.assertEqual(control.step(pad(), rover(), 12.32)['left_power'], 0)

    def test_invalid_and_dropped_input_requires_neutral_again(self):
        for invalid in (dict(sticks={'left_y': float('nan'), 'right_x': 0}),
                        dict(sticks={'right_x': 0}),
                        dict(pad(1)['input'], resyncing=True)):
            control = self.ready()
            self.assertFalse(control.step(dict(pad(), input=invalid), rover(), 10.4)['armed'])


class SerialDriveTests(unittest.TestCase):
    def test_real_transport_mixing_disconnect_and_final_stop(self):
        master, slave = pty.openpty()
        current = pad()
        commands = []
        stopping = threading.Event()
        def fake_rover():
            pending = b''
            while not stopping.is_set():
                if not select.select([master], [], [], .02)[0]:
                    continue
                pending += os.read(master, 4096)
                while b'\n' in pending:
                    line, pending = pending.split(b'\n', 1)
                    command = json.loads(line)
                    commands.append(command)
                    if command['T'] in (126, 130):
                        os.write(master, json.dumps(BASE if command['T'] == 130 else IMU).encode() + b'\n')
        worker = threading.Thread(target=fake_rover)
        worker.start()
        monitor = SerialMonitor(os.ttyname(slave), poll_interval=.1, controller_source=lambda: copy.deepcopy(current))
        monitor.start()
        def until(predicate):
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline and not predicate():
                time.sleep(.005)
            self.assertTrue(predicate())
        try:
            until(lambda: monitor.snapshot()['drive']['armed'])
            current['input'] = pad(.6, .2)['input']
            until(lambda: any(x['T'] == 1 and x['L'] > x['R'] > 0 for x in commands))
            self.assertTrue(all(abs(x[key]) <= .25 for x in commands if x['T'] == 1 for key in ('L', 'R')))
            current['connected'] = False
            until(lambda: not monitor.snapshot()['drive']['armed'])
            until(lambda: [x for x in commands if x['T'] == 1][-1] == json.loads(STOP_COMMAND))
        finally:
            monitor.close()
            time.sleep(.03)
            stopping.set()
            worker.join(timeout=1)
            os.close(master)
            os.close(slave)
        self.assertFalse(monitor.thread.is_alive())
        self.assertEqual(commands[0], json.loads(STOP_COMMAND))
        self.assertIn({'T': 136, 'cmd': 500}, commands)
        self.assertEqual(commands[-1], json.loads(STOP_COMMAND))


class ScreenTests(unittest.TestCase):
    def test_fixed_positions_with_changed_signs_and_lengths(self):
        from apptirover.telemetry import Telemetry
        state = Telemetry('/dev/ttyAMA0')
        state.opened(1)
        state.accept(BASE, 1)
        state.accept(IMU, 1)
        host = dict(cpu_temperature_c=50, load_1m=.1, usb_cameras={})
        def screen(data):
            output = io.StringIO()
            Console(file=output, width=80, color_system=None).print(render(data, host, 2, width=80))
            return output.getvalue().splitlines()
        first = screen(state.snapshot(2))
        state.accept(dict(IMU, ax=-1000.0, ay=999.99, az=0.0, r=-179.9, p=0.1), 2)
        second = screen(state.snapshot(2))
        for label in ('Attitude', 'Acceleration', 'Gyroscope'):
            a = next(line for line in first if label in line)
            b = next(line for line in second if label in line)
            self.assertEqual([i for i, char in enumerate(a) if char == '.'],
                             [i for i, char in enumerate(b) if char == '.'])
        self.assertLessEqual(len(first), 24)
        self.assertTrue(all(len(line) <= 80 for line in first))


if __name__ == '__main__':
    unittest.main()
