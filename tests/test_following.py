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

from apptirover.drive import DriveControl
from apptirover.following import FollowConfig, Follower
from apptirover.serial_monitor import SerialMonitor
from apptirover.status import render
from apptirover.camera import CameraConfig, CameraMonitor
from apptirover.telemetry import Telemetry
from test_drive import pad, rover


def controller(presses=0, held=False, throttle=0, turn=0):
    result = pad(throttle, turn)
    result['input'].update(button_presses={'BTN_NORTH': presses}, buttons=['BTN_NORTH'] if held else [], dropped=0)
    return result


def target(width=.075, center=.5, track_id=1, ambiguous=False):
    box = [center - width / 2, .15, center + width / 2, .85]
    return dict(id=track_id, box=box, measurement=box.copy(), score=.9, velocity=[0, 0, 0, 0],
                measured=True, confirmed=True, ambiguous=ambiguous)


def vision(now, sequence=1, tracks=None, epoch=(1, 1, 1)):
    tracks = copy.deepcopy([target()] if tracks is None else tracks)
    for track in tracks:
        track.setdefault('seen_at', now - .02)
    return dict(enabled=True, detection_enabled=True, observation_valid=True, stream_ready=True,
                generation=epoch[0], camera_generation=epoch[1], detection_generation=epoch[2],
                revision=0, requested_revision=0,
                observation=dict(captured_at=now - .02, completed_at=now - .01,
                                 process_generation=epoch[0], camera_generation=epoch[1], detection_generation=epoch[2],
                                 frame_sequence=sequence, tracks=tracks))


class FollowingTests(unittest.TestCase):
    def ready(self, config=None):
        drive = DriveControl(follow_config=config)
        for index in range(5):
            drive.step(controller(), rover(), 10 + index * .1)
        self.assertTrue(drive.armed)
        return drive

    def follow(self, tracks=None, config=None):
        drive = self.ready(config)
        for index in range(25):
            now = 10.5 + index * .05
            status = drive.step(controller(1, held=True), rover(), now, vision(now, index + 1, tracks))
        self.assertEqual(status['following']['state'], 'following', status)
        return drive, now

    def test_x_acquires_then_toggles_off_once_per_press(self):
        drive = self.ready()
        now = 10.5
        state = drive.step(controller(1, held=True), rover(), now, vision(now))
        self.assertEqual(state['following']['state'], 'acquiring')
        self.assertEqual(state['left_power'], 0)
        for index in range(1, 25):
            now = 10.5 + index * .05
            state = drive.step(controller(1, held=True), rover(), now, vision(now, index + 1))
        self.assertEqual(state['following']['state'], 'following')
        self.assertGreater(state['left_power'], 0)
        state = drive.step(controller(2), rover(), now + .02, vision(now + .02, 30))
        self.assertEqual(state['following']['state'], 'disabled')
        self.assertEqual(state['left_power'], 0)
        state = drive.step(controller(2), rover(), now + .04, vision(now + .04, 31))
        self.assertFalse(state['following']['active'])

    def test_reverse_and_turn_signs_with_bounded_motor_power(self):
        drive, _ = self.follow([target(width=.3)])
        self.assertLess(drive.left, 0)
        self.assertEqual(drive.left, drive.right)
        self.assertLessEqual(abs(drive.left), .25)
        right, _ = self.follow([target(center=.65)])
        self.assertGreater(right.left, right.right)
        left, _ = self.follow([target(center=.35)])
        self.assertLess(left.left, left.right)
        centered, _ = self.follow([target(width=.15)])
        self.assertEqual((centered.left, centered.right), (0, 0))

    def test_manual_takeover_does_not_resume_when_sticks_center(self):
        drive, now = self.follow()
        state = drive.step(controller(1, turn=-.5), rover(), now + .02, vision(now + .02, 30))
        self.assertEqual(state['authority'], 'manual')
        self.assertLess(state['left_power'], state['right_power'])
        state = drive.step(controller(1), rover(), now + .04, vision(now + .04, 31))
        self.assertFalse(state['following']['active'])
        self.assertEqual(state['left_power'], 0)

    def test_x_disable_stops_even_with_simultaneous_stick_movement(self):
        drive, now = self.follow()
        state = drive.step(controller(2, throttle=1), rover(), now + .02, vision(now + .02, 30))
        self.assertEqual((state['left_power'], state['right_power']), (0, 0))
        self.assertFalse(state['following']['active'])

    def test_loss_ambiguity_and_predicted_only_tracks_stop_immediately(self):
        for tracks in ([], [target(track_id=2)], [dict(target(), measured=False)], [target(ambiguous=True)]):
            with self.subTest(tracks=tracks):
                drive, now = self.follow()
                state = drive.step(controller(1), rover(), now + .02, vision(now + .02, 30, tracks))
                self.assertEqual((state['left_power'], state['right_power']), (0, 0))
                self.assertFalse(state['following']['active'])
                state = drive.step(controller(1), rover(), now + .04, vision(now + .04, 31))
                self.assertFalse(state['following']['active'])

    def test_own_source_deadline_expires_between_camera_callbacks(self):
        drive, now = self.follow()
        frozen = vision(now, 25)
        for step in range(1, 15):
            state = drive.step(controller(1), rover(), now + step * .02, frozen)
        self.assertFalse(state['following']['active'])
        self.assertEqual(state['left_power'], 0)
        self.assertIn('stale', state['following']['reason'])

    def test_restart_toggle_and_controller_faults_cancel_authority(self):
        for field in ('camera', 'detector', 'process', 'revision', 'detection', 'controller', 'loop', 'rover'):
            with self.subTest(field=field):
                drive, now = self.follow()
                current = controller(1)
                data = vision(now + .02, 30)
                board = rover()
                if field in ('camera', 'detector', 'process'):
                    epoch = tuple(2 if name == field else 1 for name in ('process', 'camera', 'detector'))
                    data = vision(now + .02, 30, epoch=epoch)
                elif field == 'revision':
                    data['requested_revision'] = 1
                elif field == 'detection':
                    data['detection_enabled'] = False
                elif field == 'controller':
                    current['connected'] = False
                elif field == 'loop':
                    now += 1
                else:
                    board['port_open'] = False
                state = drive.step(current, board, now + .02, data)
                self.assertEqual(state['left_power'], 0)
                self.assertFalse(state['following']['active'])

    def test_repeated_snapshot_cannot_supply_acquisition_votes(self):
        drive = self.ready()
        first = vision(10.5)
        for index in range(10):
            drive.step(controller(1), rover(), 10.5 + index * .02, first)
        self.assertEqual(len(drive.follower.history), 1)
        self.assertEqual(drive.left, 0)

    def test_reused_frame_id_cannot_refresh_a_motion_deadline(self):
        drive, now = self.follow()
        refreshed = vision(now + .02, 25)
        state = drive.step(controller(1), rover(), now + .02, refreshed)
        self.assertFalse(state['following']['active'])
        self.assertEqual(state['left_power'], 0)

    def test_startup_held_x_reconnect_and_dropped_events_do_not_activate(self):
        drive = DriveControl()
        for index in range(10):
            now = 10 + index * .1
            drive.step(controller(1, held=True), rover(), now, vision(now, index + 1))
        self.assertFalse(drive.follower.active)
        changed = controller(2, held=True)
        changed['input_generation'] = 2
        drive.step(changed, rover(), 11, vision(11, 12))
        self.assertFalse(drive.follower.active)
        changed['input']['dropped'] = 1
        changed['input']['button_presses']['BTN_NORTH'] = 3
        drive.step(changed, rover(), 11.1, vision(11.1, 13))
        self.assertFalse(drive.follower.active)

    def test_no_camera_or_detection_prevents_activation(self):
        for data in (None, dict(enabled=False), dict(enabled=True, detection_enabled=False)):
            drive = self.ready()
            state = drive.step(controller(1), rover(), 10.5, data)
            self.assertFalse(state['following']['active'])
            self.assertEqual(state['left_power'], 0)

    def test_nan_invalid_and_clipped_boxes_cannot_preserve_last_motion(self):
        for field, value in (('box', [float('nan'), .1, .3, .8]), ('box', [.3, .1, .3, .8]),
                             ('velocity', [float('inf'), 0, 0, 0]), ('measurement', [0, .1, .2, .8])):
            drive, now = self.follow()
            broken = dict(target(), **{field: value})
            state = drive.step(controller(1), rover(), now + .02, vision(now + .02, 30, [broken]))
            self.assertFalse(state['following']['active'])
            self.assertEqual(state['left_power'], 0)

    def test_acquisition_uses_winning_id_when_another_current_score_is_higher(self):
        follower = Follower()
        follower.activate(vision(10), 10)
        for index in range(12):
            now = 10 + index * .1
            other = dict(target(.3, track_id=2), score=.6 if index < 9 else .99)
            follower.step(vision(now, index + 1, [target(), other]), now)
        self.assertEqual(follower.target_id, 1)
        self.assertAlmostEqual(follower.target['box'][2] - follower.target['box'][0], .075)
        self.assertGreater(follower.throttle, 0)

    def test_similar_candidates_remain_stationary_and_history_is_bounded(self):
        follower = Follower()
        follower.activate(vision(10), 10)
        for index in range(301):
            now = 10 + index * .1
            follower.step(vision(now, index + 1, [target(center=.3), target(center=.7, track_id=2)]), now)
        self.assertEqual(follower.state, 'acquiring')
        self.assertEqual(follower.throttle, 0)
        self.assertLessEqual(len(follower.history), 12)

    def test_changed_drop_counter_cancels_even_when_resync_finished_between_ticks(self):
        drive, now = self.follow()
        changed = controller(1)
        changed['input']['dropped'] = 1
        state = drive.step(changed, rover(), now + .02, vision(now + .02, 30))
        self.assertFalse(state['following']['active'])
        self.assertFalse(state['armed'])
        self.assertEqual(state['left_power'], 0)

    def test_reordered_and_nonfinite_observation_timestamps_cancel_motion(self):
        for field, value in (('captured_at', float('nan')), ('completed_at', float('inf')),
                             ('captured_at', 99), ('frame_sequence', 2)):
            drive, now = self.follow()
            data = vision(now + .02, 30)
            data['observation'][field] = value
            state = drive.step(controller(1), rover(), now + .02, data)
            self.assertFalse(state['following']['active'])
            self.assertEqual(state['left_power'], 0)

    def test_follow_power_also_respects_smaller_global_limit(self):
        drive, now = self.follow([target(.025)], FollowConfig(max_power=.8))
        self.assertLessEqual(max(abs(drive.left), abs(drive.right)), .5)
        self.assertEqual(drive.snapshot()['power_limit'], .5)

    def test_tracking_status_and_full_command_fit_short_terminal(self):
        drive, _ = self.follow()
        state = Telemetry('/fake').snapshot(time.monotonic())
        state['drive'] = drive.snapshot()
        camera = CameraMonitor(CameraConfig(enabled=False, advertised_host='192.168.1.42')).snapshot()
        host = dict(cpu_temperature_c=50, load_1m=.2, usb_cameras={})
        for width in (60, 80, 120):
            output = io.StringIO()
            Console(file=output, width=width, color_system=None).print(
                render(state, host, 1, camera=camera, width=width, height=24))
            rendered = output.getvalue()
            self.assertIn('Following', rendered)
            self.assertIn(''.join(camera['ffplay_command'].split()), ''.join(rendered.split()))
            self.assertLessEqual(len(rendered.splitlines()), 24)


class FollowingTransportTests(unittest.TestCase):
    def test_follow_reverse_freshness_failure_manual_takeover_and_shutdown_over_uart(self):
        master, slave = pty.openpty()
        current = controller()
        scene = dict(width=.075, available=True, failure=False)
        commands = []
        gaps = []
        stopping = threading.Event()
        def board():
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
                        os.write(master, json.dumps(dict(T=1001, v=12) if command['T'] == 130 else dict(T=1002, ax=0)).encode() + b'\n')
        last_frame, frame, latest = 0, 0, None
        def camera():
            nonlocal last_frame, frame, latest
            if scene['failure']:
                raise RuntimeError('camera receiver failed')
            now = time.monotonic()
            if now - last_frame >= .05:
                frame += 1
                latest = vision(now, frame, [target(scene['width'])] if scene['available'] else [])
                last_frame = now
            return latest
        monitor = SerialMonitor(os.ttyname(slave), poll_interval=.1, controller_source=lambda: copy.deepcopy(current),
                                vision_source=camera, follow_config=FollowConfig(acquire_seconds=.3))
        original = monitor.drive.step
        previous = None
        def measure(pad, board, now, data):
            nonlocal previous
            if previous is not None:
                gaps.append(now - previous)
            previous = now
            return original(pad, board, now, data)
        monitor.drive.step = measure
        worker = threading.Thread(target=board)
        worker.start()
        monitor.start()
        def until(predicate):
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline and not predicate():
                time.sleep(.005)
            self.assertTrue(predicate(), monitor.snapshot())
        try:
            until(lambda: monitor.snapshot()['drive']['armed'])
            current['input']['button_presses']['BTN_NORTH'] = 1  # Tap is already released.
            until(lambda: monitor.snapshot()['drive']['left_power'] > .02)
            until(lambda: any(c['T'] == 1 and c['L'] > 0 and c['R'] > 0 for c in commands))
            scene['width'] = .3
            until(lambda: any(c['T'] == 1 and c['L'] < 0 and c['R'] < 0 for c in commands))
            scene['available'] = False
            until(lambda: monitor.snapshot()['drive']['following']['state'] == 'stopped')
            until(lambda: [c for c in commands if c['T'] == 1][-1] == dict(T=1, L=0., R=0.))
            scene['available'] = True
            time.sleep(.1)
            self.assertFalse(monitor.snapshot()['drive']['following']['active'])
            current['input']['button_presses']['BTN_NORTH'] = 2
            until(lambda: monitor.snapshot()['drive']['following']['state'] == 'following')
            scene['failure'] = True
            until(lambda: not monitor.snapshot()['drive']['following']['active'])
            self.assertTrue(monitor.thread.is_alive())
            current['input']['sticks']['right_x'] = .5
            until(lambda: monitor.snapshot()['drive']['left_power'] > 0 > monitor.snapshot()['drive']['right_power'])
            self.assertGreater(monitor.snapshot()['telemetry_packets'], 5)
            self.assertLess(max(gaps), .25)
        finally:
            monitor.close()
            time.sleep(.03)
            stopping.set()
            worker.join(timeout=1)
            os.close(master)
            os.close(slave)
        self.assertFalse(monitor.thread.is_alive())
        self.assertEqual(commands[-1], dict(T=1, L=0., R=0.))


if __name__ == '__main__':
    unittest.main()
