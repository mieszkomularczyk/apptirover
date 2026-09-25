import io
import subprocess
import sys
import time
import unittest
from unittest.mock import patch

from rich.console import Console

from apptirover.app import arguments
from apptirover.camera import CameraConfig, CameraMonitor, playback
from apptirover.status import render
from apptirover.telemetry import Telemetry


class CameraStateTests(unittest.TestCase):
    def live(self, boxes=None):
        monitor = CameraMonitor(CameraConfig(advertised_host='192.168.1.42'))
        now = time.monotonic()
        monitor.updated_at = now
        monitor.state.update(state='streaming', stream_ready=True, detection_state='ready',
                             last_frame_at=now, revision=0,
                             observation=dict(captured_at=now, completed_at=now, frame_sequence=3,
                                              boxes=boxes or [], inference_ms=22))
        return monitor

    def test_fresh_empty_detection_is_distinct_from_unavailable(self):
        monitor = self.live()
        self.assertEqual(monitor.snapshot()['people'], 0)
        self.assertTrue(monitor.snapshot()['observation_valid'])
        monitor.set_detection(False)
        self.assertIsNone(monitor.snapshot()['people'])
        self.assertFalse(monitor.snapshot()['observation_valid'])
        self.assertTrue(monitor.snapshot()['ready'])

    def test_old_results_cannot_reappear_during_disable_enable_race(self):
        monitor = self.live([[.1, .2, .4, .8, .9, 0]])
        old_state = monitor.state.copy()
        monitor.set_detection(False)
        monitor.set_detection(True)
        monitor.state.update(old_state)  # Late IPC from before the toggle.
        self.assertFalse(monitor.snapshot()['observation_valid'])
        monitor.state['revision'] = 2
        self.assertTrue(monitor.snapshot()['observation_valid'])

    def test_freshness_uses_source_age_not_arrival_or_worker_heartbeat(self):
        for field in ('observation', 'worker', 'video'):
            monitor = self.live([[0, 0, 1, 1, .9, 0]])
            if field == 'observation':
                monitor.state['observation']['captured_at'] -= .6
            elif field == 'worker':
                monitor.updated_at -= 2
            else:
                monitor.state['last_frame_at'] -= 2
            snapshot = monitor.snapshot()
            self.assertFalse(snapshot['observation_valid'])
            self.assertEqual(snapshot['boxes'], [])
            self.assertIsNone(snapshot['people'])

    def test_snapshots_do_not_share_mutable_observations(self):
        monitor = self.live([[0, 0, 1, 1, .9, 0]])
        monitor.snapshot()['boxes'].clear()
        self.assertEqual(monitor.snapshot()['people'], 1)

    def test_manual_import_has_no_video_or_ai_dependencies(self):
        result = subprocess.run([sys.executable, '-c',
            'import apptirover.app,sys; assert not ({"gi", "numpy", "cv2", "ncnn"} & sys.modules.keys())'],
            capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_cli_modes_and_validation(self):
        self.assertFalse(arguments(['--controller-only']).camera_config.enabled)
        self.assertTrue(arguments(['--camera-only']).camera_config.enabled)
        self.assertTrue(arguments(['--no-detection']).camera_config.no_detection)
        for flags in (['--camera-only', '--no-camera'], ['--camera-width', '641'],
                      ['--camera-fps', '0'], ['--confidence', 'nan'], ['--stream-host', '0.0.0.0'],
                      ['--rtsp-path', '/$(command)'], ['--rtsp-host', '']):
            with self.subTest(flags=flags), patch('sys.stderr', new=io.StringIO()), self.assertRaises(SystemExit):
                arguments(flags)

    def test_remote_address_and_configured_endpoint(self):
        url, command = playback(CameraConfig(advertised_host='192.168.1.42', port=8555, path='/rover'))
        self.assertEqual(url, 'rtsp://192.168.1.42:8555/rover')
        self.assertIn('"' + url + '"', command)
        self.assertEqual(playback(CameraConfig(advertised_host='fd00::1'))[0], 'rtsp://[fd00::1]:8554/camera')

    def test_full_ffplay_command_survives_small_terminal_and_disabled_camera(self):
        camera = CameraMonitor(CameraConfig(enabled=False, advertised_host='192.168.1.42')).snapshot()
        rover = Telemetry('/fake').snapshot(time.monotonic())
        host = dict(cpu_temperature_c=50, load_1m=.2, usb_cameras={})
        for width in (60, 80, 120):
            output = io.StringIO()
            Console(file=output, width=width, color_system=None).print(
                render(rover, host, 1, camera=camera, width=width, height=24))
            text = output.getvalue()
            compact = ''.join(text.split())
            self.assertIn(''.join(camera['ffplay_command'].split()), compact)
            self.assertIn('CAMERA', text)
            self.assertLessEqual(len(text.splitlines()), 24)


if __name__ == '__main__':
    unittest.main()
