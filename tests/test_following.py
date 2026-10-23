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
from apptirover.following import BoundedPID, FollowConfig, Follower
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
    def ready(self, config=None, max_power=1., manual_power=.5):
        drive = DriveControl(max_power=max_power, follow_config=config or FollowConfig(desired_width=.15),
                             manual_power=manual_power)
        for index in range(5):
            drive.step(controller(), rover(), 10 + index * .1)
        self.assertTrue(drive.armed)
        return drive

    def follow(self, tracks=None, config=None):
        drive = self.ready(config)
        for index in range(25):
            now = 10.5 + index * .08
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
        self.assertLessEqual(abs(drive.left), 1.)
        right, _ = self.follow([target(center=.65)])
        self.assertGreater(right.left, right.right)
        left, _ = self.follow([target(center=.35)])
        self.assertLess(left.left, left.right)
        centered, _ = self.follow([target(width=.15)])
        self.assertEqual((centered.left, centered.right), (0, 0))

    def test_manual_priority_preserves_mode_and_neutral_resumes_following(self):
        drive, now = self.follow()
        state = drive.step(controller(1, turn=-.5), rover(), now + .02, vision(now + .02, 30))
        self.assertEqual(state['authority'], 'manual')
        self.assertLess(state['left_power'], state['right_power'])
        self.assertTrue(state['following']['enabled'])
        self.assertEqual(state['following']['state'], 'manual')
        self.assertFalse(state['following']['motion_active'])
        state = drive.step(controller(1), rover(), now + .04, vision(now + .04, 31))
        self.assertTrue(state['following']['active'])
        self.assertEqual(state['authority'], 'following')
        self.assertEqual(state['following']['state'], 'following')
        self.assertGreater(state['left_power'], 0)
        self.assertGreater(state['right_power'], 0)

    def test_x_disable_stops_even_with_simultaneous_stick_movement(self):
        drive, now = self.follow()
        state = drive.step(controller(2, throttle=1), rover(), now + .02, vision(now + .02, 30))
        self.assertEqual((state['left_power'], state['right_power']), (0, 0))
        self.assertFalse(state['following']['active'])

    def test_loss_ambiguity_and_predictions_stop_motion_and_automatically_recover(self):
        for tracks in ([], [target(track_id=2, center=.8)], [dict(target(), measured=False)], [target(ambiguous=True)]):
            with self.subTest(tracks=tracks):
                drive, now = self.follow()
                state = drive.step(controller(1), rover(), now + .02, vision(now + .02, 30, tracks))
                self.assertEqual((state['left_power'], state['right_power']), (0, 0))
                self.assertTrue(state['following']['active'])
                self.assertFalse(state['following']['motion_active'])
                state = drive.step(controller(1), rover(), now + .04, vision(now + .04, 31))
                self.assertTrue(state['following']['active'])
                self.assertTrue(state['following']['motion_active'])
                self.assertGreater(state['left_power'], 0)

    def test_own_source_deadline_expires_between_camera_callbacks(self):
        drive, now = self.follow()
        frozen = vision(now, 25)
        for step in range(1, 15):
            state = drive.step(controller(1), rover(), now + step * .02, frozen)
        self.assertTrue(state['following']['active'])
        self.assertEqual(state['left_power'], 0)
        self.assertIn('stale', state['following']['reason'])
        state = drive.step(controller(1), rover(), now + .30, vision(now + .30, 30))
        self.assertTrue(state['following']['motion_active'])
        self.assertGreater(state['left_power'], 0)

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
                self.assertTrue(state['following']['active'])
                self.assertFalse(state['following']['motion_active'])

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
        self.assertTrue(state['following']['active'])
        self.assertFalse(state['following']['motion_active'])
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

    def test_x_enables_mode_while_waiting_for_camera_or_detection(self):
        for data in (None, dict(enabled=False), dict(enabled=True, detection_enabled=False)):
            drive = self.ready()
            state = drive.step(controller(1), rover(), 10.5, data)
            self.assertTrue(state['following']['active'])
            self.assertFalse(state['following']['motion_active'])
            self.assertEqual(state['left_power'], 0)
            self.assertEqual(drive.step(controller(2), rover(), 10.52, data)['following']['state'], 'disabled')

    def test_nan_and_invalid_boxes_cannot_preserve_last_motion(self):
        for field, value in (('box', [float('nan'), .1, .3, .8]), ('box', [.3, .1, .3, .8]),
                             ('velocity', [float('inf'), 0, 0, 0]), ('measurement', [-.1, .1, .2, .8])):
            drive, now = self.follow()
            broken = dict(target(), **{field: value})
            state = drive.step(controller(1), rover(), now + .02, vision(now + .02, 30, [broken]))
            self.assertTrue(state['following']['active'])
            self.assertFalse(state['following']['motion_active'])
            self.assertEqual(state['left_power'], 0)

    def test_acquisition_uses_winning_id_when_another_current_score_is_higher(self):
        follower = Follower(FollowConfig(desired_width=.15))
        follower.activate(vision(10), 10)
        for index in range(12):
            now = 10 + index * .1
            other = dict(target(.3, track_id=2), score=.6 if index < 9 else .99)
            follower.step(vision(now, index + 1, [target(), other]), now)
        self.assertEqual(follower.target_id, 1)
        self.assertAlmostEqual(follower.target['box'][2] - follower.target['box'][0], .075)
        self.assertGreater(follower.throttle, 0)

    def test_similar_candidates_select_a_stable_target_instead_of_waiting_forever(self):
        follower = Follower()
        follower.activate(vision(10), 10)
        for index in range(301):
            now = 10 + index * .1
            follower.step(vision(now, index + 1, [target(center=.3), target(center=.7, track_id=2)]), now)
        self.assertEqual(follower.state, 'following')
        self.assertEqual(follower.target_id, 1)
        self.assertLessEqual(len(follower.history), 12)

    def test_changed_drop_counter_cancels_even_when_resync_finished_between_ticks(self):
        drive, now = self.follow()
        changed = controller(1)
        changed['input']['dropped'] = 1
        state = drive.step(changed, rover(), now + .02, vision(now + .02, 30))
        self.assertTrue(state['following']['active'])
        self.assertFalse(state['armed'])
        self.assertEqual(state['left_power'], 0)

    def test_reordered_and_nonfinite_observation_timestamps_cancel_motion(self):
        for field, value in (('captured_at', float('nan')), ('completed_at', float('inf')),
                             ('captured_at', 99), ('frame_sequence', 2)):
            drive, now = self.follow()
            data = vision(now + .02, 30)
            data['observation'][field] = value
            state = drive.step(controller(1), rover(), now + .02, data)
            self.assertTrue(state['following']['active'])
            self.assertFalse(state['following']['motion_active'])
            self.assertEqual(state['left_power'], 0)

    def test_small_following_errors_produce_short_usable_pulses_with_off_time(self):
        for width, center, signs in ((.15, .54, (1, -1)), (.15, .46, (-1, 1)),
                                     (.13, .5, (1, 1)), (.17, .5, (-1, -1))):
            with self.subTest(width=width, center=center):
                drive, now = self.follow([target(width=width, center=center)])
                powers = []
                for index in range(100):
                    moment = now + (index + 1) * .02
                    state = drive.step(controller(1), rover(), moment,
                                       vision(moment, 30 + index, [target(width=width, center=center)]))
                    powers.append((state['left_power'], state['right_power']))
                moving = [power for power in powers if any(power)]
                self.assertTrue(moving)
                self.assertIn((0., 0.), powers)
                for a, b in moving:
                    self.assertAlmostEqual(max(abs(a), abs(b)), .5)
                    self.assertGreater(a * signs[0], 0)
                    self.assertGreater(b * signs[1], 0)

    def test_image_edge_still_allows_turning_but_clipping_cannot_trigger_approach(self):
        clipped = target(width=.08, center=.04)
        drive, _ = self.follow([clipped])
        self.assertEqual(drive.throttle, 0)
        self.assertLess(drive.following_output.left, 0)
        self.assertGreater(drive.following_output.right, 0)

    def test_small_filter_overshoot_on_a_vertically_clipped_person_does_not_block_steering(self):
        clipped = target(width=.15, center=.6)
        clipped['box'][1], clipped['box'][3] = -1e-15, 1 + 1e-15
        clipped['measurement'][1], clipped['measurement'][3] = 0, 1
        drive, _ = self.follow([clipped])
        self.assertGreater(drive.following_output.left, 0)
        self.assertLess(drive.following_output.right, 0)

    def test_new_nearby_track_id_can_continue_the_person_after_id_churn(self):
        drive, now = self.follow()
        state = drive.step(controller(1), rover(), now + .02, vision(now + .02, 30, [target(track_id=2)]))
        self.assertEqual(state['following']['target_id'], 2)
        self.assertTrue(state['following']['motion_active'])
        self.assertGreater(state['following_output']['average_left'], 0)

    def test_long_loss_reacquires_a_new_person_without_another_x_press(self):
        drive, now = self.follow()
        for index in range(1, 30):
            state = drive.step(controller(1), rover(), now + index * .05, vision(now + index * .05, 30 + index, []))
            self.assertTrue(state['following']['enabled'])
            self.assertEqual(state['left_power'], 0)
        for index in range(30, 57):
            moment = now + index * .05
            state = drive.step(controller(1), rover(), moment, vision(moment, 30 + index, [target(center=.7, track_id=2)]))
        self.assertEqual(state['following']['state'], 'following')
        self.assertEqual(state['following']['target_id'], 2)

    def test_camera_generation_change_reacquires_automatically(self):
        drive, now = self.follow()
        for index in range(1, 28):
            moment = now + index * .05
            state = drive.step(controller(1), rover(), moment, vision(moment, index, epoch=(2, 1, 1)))
        self.assertEqual(state['following']['state'], 'following')
        self.assertTrue(state['following']['enabled'])
        self.assertEqual(state['following']['session'], 1)

    def test_x_can_disable_mode_even_while_rover_feedback_is_unavailable(self):
        drive, now = self.follow()
        state = drive.step(controller(2), dict(rover(), port_open=False), now + .02, None)
        self.assertFalse(state['following']['enabled'])
        self.assertEqual(state['left_power'], 0)

    def test_x_can_enable_during_manual_driving_and_release_resumes_autonomy(self):
        drive = self.ready()
        for index in range(25):
            now = 10.5 + index * .05
            state = drive.step(controller(1, turn=.5), rover(), now, vision(now, index + 1))
        self.assertTrue(state['following']['enabled'])
        self.assertEqual(state['authority'], 'manual')
        self.assertEqual(state['following']['tracking_state'], 'following')
        state = drive.step(controller(1), rover(), now + .02, vision(now + .02, 30))
        self.assertTrue(state['following']['motion_active'])
        self.assertEqual(state['authority'], 'following')

    def test_controller_fault_keeps_mode_and_resumes_after_neutral_rearm(self):
        drive, now = self.follow()
        offline = dict(controller(1), connected=False)
        state = drive.step(offline, rover(), now + .02, None)
        self.assertTrue(state['following']['enabled'])
        self.assertFalse(state['armed'])
        for index in range(1, 35):
            moment = now + .02 + index * .05
            state = drive.step(controller(1), rover(), moment, vision(moment, 30 + index))
        self.assertTrue(state['armed'])
        self.assertEqual(state['following']['state'], 'following')
        self.assertGreater(state['left_power'], 0)

    def test_follow_power_also_respects_smaller_global_limit(self):
        drive = self.ready(FollowConfig(desired_width=.15), max_power=.3)
        for index in range(40):
            now = 10.5 + index * .05
            state = drive.step(controller(1), rover(), now, vision(now, index + 1))
        self.assertLessEqual(max(abs(drive.left), abs(drive.right)), .3)
        self.assertEqual(state['power_limit'], .3)

    def test_automatic_distance_reference_drives_forward_and_reverse_around_starting_distance(self):
        drive, now = self.follow([target(width=.3)], FollowConfig(distance_scale=1.))
        self.assertAlmostEqual(drive.follower.desired_width, .3)
        self.assertEqual((drive.left, drive.right), (0, 0))
        state = drive.step(controller(1), rover(), now + .02, vision(now + .02, 30, [target(width=.24)]))
        self.assertGreater(state['throttle'], 0)
        self.assertGreater(state['left_power'], 0)
        for index in range(1, 5):
            moment = now + .02 + index * .02
            state = drive.step(controller(1), rover(), moment, vision(moment, 30 + index, [target(width=.36)]))
        self.assertLess(state['throttle'], 0)
        self.assertLess(state['left_power'], 0)
        self.assertAlmostEqual(state['following']['desired_width'], .3)

    def test_default_reference_requests_closer_distance_and_fixed_width_ignores_scale(self):
        drive, now = self.follow([target(width=.3)], FollowConfig())
        self.assertAlmostEqual(drive.follower.desired_width, .375)
        self.assertGreater(drive.throttle, 0)
        state = drive.step(controller(1), rover(), now + .02, vision(now + .02, 30, [target(width=.375)]))
        self.assertEqual(state['throttle'], 0)
        self.assertEqual((state['left_power'], state['right_power']), (0., 0.))
        fixed, _ = self.follow([target(width=.3)], FollowConfig(desired_width=.3, distance_scale=2.))
        self.assertEqual(fixed.throttle, 0)
        self.assertEqual(fixed.follower.desired_width, .3)

    def test_manual_override_and_reacquisition_preserve_learned_distance_and_clear_integral(self):
        drive, now = self.follow([target(width=.3, center=.65)], FollowConfig())
        self.assertGreater(drive.follower.yaw_pid.integral, 0)
        state = drive.step(controller(1, turn=.5), rover(), now + .02,
                           vision(now + .02, 30, [target(width=.2, center=.65)]))
        self.assertEqual(state['following']['yaw_integral'], 0)
        self.assertAlmostEqual(state['following']['desired_width'], .375)
        for index in range(1, 30):
            moment = now + .02 + index * .05
            state = drive.step(controller(1), rover(), moment, vision(moment, index, [target(width=.2)], epoch=(2, 1, 1)))
        self.assertEqual(state['following']['state'], 'following')
        self.assertAlmostEqual(state['following']['desired_width'], .375)
        self.assertGreater(state['throttle'], 0)

    def test_clipped_target_turns_until_distance_can_be_learned(self):
        drive, now = self.follow([target(width=.08, center=.04)], FollowConfig())
        self.assertIsNone(drive.follower.desired_width)
        self.assertLess(drive.turn, 0)
        self.assertEqual(drive.throttle, 0)
        state = drive.step(controller(1), rover(), now + .02, vision(now + .02, 30, [target(width=.3)]))
        self.assertAlmostEqual(state['following']['desired_width'], .375)
        self.assertGreater(state['throttle'], 0)

    def test_only_a_new_x_session_relearns_automatic_distance(self):
        drive, now = self.follow([target(width=.3)], FollowConfig())
        state = drive.step(controller(2), rover(), now + .02, vision(now + .02, 30, [target(width=.2)]))
        self.assertIsNone(state['following']['desired_width'])
        for index in range(1, 30):
            moment = now + .02 + index * .05
            state = drive.step(controller(3), rover(), moment, vision(moment, 30 + index, [target(width=.2)]))
        self.assertEqual(state['following']['state'], 'following')
        self.assertAlmostEqual(state['following']['desired_width'], .25)
        self.assertGreater(state['throttle'], 0)

    def test_even_a_brief_empty_frame_clears_integral_before_automatic_resume(self):
        drive, now = self.follow([target(width=.15, center=.65)])
        self.assertGreater(drive.follower.yaw_pid.integral, 0)
        state = drive.step(controller(1), rover(), now + .02, vision(now + .02, 30, []))
        self.assertEqual(state['following']['yaw_integral'], 0)
        self.assertEqual(state['left_power'], 0)
        state = drive.step(controller(1), rover(), now + .04,
                           vision(now + .04, 31, [target(width=.15, center=.65)]))
        self.assertEqual(state['following']['yaw_integral'], 0)
        self.assertGreater(state['turn'], 0)

    def test_joystick_x_and_stale_data_interrupt_an_active_pulse_on_next_tick(self):
        for cause in ('manual', 'X', 'stale', 'rover'):
            with self.subTest(cause=cause):
                drive = self.ready()
                for index in range(100):
                    now = 10.5 + index * .02
                    state = drive.step(controller(1), rover(), now,
                                       vision(now, index + 1, [target(width=.15, center=.6)]))
                    if state['left_power'] > 0 > state['right_power']:
                        break
                else:
                    self.fail('No steering pulse')
                data = vision(now + .02, index + 2, [target(width=.15, center=.6)])
                current, board = controller(1), rover()
                if cause == 'manual':
                    current = controller(1, throttle=.5)
                elif cause == 'X':
                    current = controller(2)
                elif cause == 'stale':
                    data = vision(now - .3, index + 2)
                else:
                    board['port_open'] = False
                state = drive.step(current, board, now + .02, data)
                self.assertFalse(state['following_output']['pulse_active'])
                self.assertEqual(state['following']['enabled'], cause != 'X')
                if cause == 'manual':
                    self.assertGreater(state['left_power'], 0)
                    self.assertGreater(state['right_power'], 0)
                else:
                    self.assertEqual((state['left_power'], state['right_power']), (0., 0.))

    def test_full_following_cap_keeps_manual_driving_at_half_power(self):
        drive = self.ready()
        for index in range(1, 30):
            state = drive.step(controller(throttle=1), rover(), 10.4 + index * .05)
        self.assertEqual((drive.left, drive.right), (.5, .5))
        self.assertEqual(state['power_limit'], .5)

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
    def test_turn_approach_reverse_loss_and_manual_resume_with_one_x_over_uart(self):
        master, slave = pty.openpty()
        current = controller()
        scene = dict(width=.13, center=.5, available=True, failure=False)
        commands = []
        command_times = []
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
                    command_times.append(time.monotonic())
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
                latest = vision(now, frame, [target(scene['width'], scene['center'])] if scene['available'] else [])
                last_frame = now
            return latest
        monitor = SerialMonitor(os.ttyname(slave), poll_interval=.1, controller_source=lambda: copy.deepcopy(current),
                                vision_source=camera, follow_config=FollowConfig(desired_width=.15, acquire_seconds=.3))
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
        def wire_motion(predicate):
            start = len(commands)
            until(lambda: any(predicate(c) for c in commands[start:] if c['T'] == 1))
        try:
            until(lambda: monitor.snapshot()['drive']['armed'])
            current['input']['button_presses']['BTN_NORTH'] = 1  # Tap is already released.
            wire_motion(lambda c: c['L'] > .12 and c['R'] > .12)  # About 25% PWM, not a barely nonzero command.
            scene['width'] = .15
            until(lambda: monitor.snapshot()['drive']['left_power'] == monitor.snapshot()['drive']['right_power'] == 0)
            scene['center'] = .54
            turn_start = len(commands)
            wire_motion(lambda c: c['L'] > .12 and c['R'] < -.12)
            first_turn = next(i for i in range(turn_start, len(commands))
                              if commands[i]['T'] == 1 and commands[i]['L'] > .12 and commands[i]['R'] < -.12)
            until(lambda: any(c == dict(T=1, L=0., R=0.) for c in commands[first_turn + 1:]))
            first_stop = next(i for i in range(first_turn + 1, len(commands)) if commands[i] == dict(T=1, L=0., R=0.))
            self.assertGreater(command_times[first_stop] - command_times[first_turn], .04)
            self.assertLess(command_times[first_stop] - command_times[first_turn], .15)
            self.assertEqual(monitor.snapshot()['drive']['following']['state'], 'following')
            scene['center'] = .46
            wire_motion(lambda c: c['L'] < -.12 and c['R'] > .12)
            scene.update(center=.5, width=.17)
            wire_motion(lambda c: c['L'] < -.12 and c['R'] < -.12)
            scene['available'] = False
            until(lambda: not monitor.snapshot()['drive']['following']['motion_active'])
            until(lambda: [c for c in commands if c['T'] == 1][-1] == dict(T=1, L=0., R=0.))
            self.assertTrue(monitor.snapshot()['drive']['following']['enabled'])
            scene['available'] = True
            wire_motion(lambda c: c['L'] < -.12 and c['R'] < -.12)
            scene['width'] = .13
            current['input']['sticks']['right_x'] = .5
            until(lambda: monitor.snapshot()['drive']['following']['state'] == 'manual')
            wire_motion(lambda c: c['L'] > 0 > c['R'])
            self.assertTrue(monitor.snapshot()['drive']['following']['enabled'])
            current['input']['sticks']['right_x'] = 0
            wire_motion(lambda c: c['L'] > .12 and c['R'] > .12)
            scene['failure'] = True
            until(lambda: not monitor.snapshot()['drive']['following']['motion_active'])
            self.assertTrue(monitor.snapshot()['drive']['following']['enabled'])
            until(lambda: [c for c in commands if c['T'] == 1][-1] == dict(T=1, L=0., R=0.))
            self.assertTrue(monitor.thread.is_alive())
            current['input']['sticks']['right_x'] = .5
            until(lambda: monitor.snapshot()['drive']['left_power'] > 0 > monitor.snapshot()['drive']['right_power'])
            current['input']['sticks']['right_x'] = 0
            until(lambda: monitor.snapshot()['drive']['left_power'] == 0)
            scene['failure'] = False
            wire_motion(lambda c: c['L'] > .12 and c['R'] > .12)
            self.assertEqual(monitor.snapshot()['drive']['following']['session'], 1)
            current['input']['button_presses']['BTN_NORTH'] = 2
            until(lambda: not monitor.snapshot()['drive']['following']['enabled'])
            until(lambda: [c for c in commands if c['T'] == 1][-1] == dict(T=1, L=0., R=0.))
            time.sleep(.1)
            self.assertEqual(monitor.snapshot()['drive']['left_power'], 0)
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
