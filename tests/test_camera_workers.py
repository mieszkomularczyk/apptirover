import errno
import json
import os
import pty
import termios
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from apptirover.camera import CameraConfig
from apptirover.terminal import TerminalKeys
from apptirover.vision.detection import DetectionWorker, _worker
from apptirover.vision.stream import CameraServer, Gst


class WorkerLifecycleTests(unittest.TestCase):
    def test_fresh_observation_is_published_before_the_status_heartbeat(self):
        server = CameraServer(CameraConfig())
        server.pipeline, server.worker, server.publish = Mock(), Mock(), Mock()
        server.worker.ready = True
        server.last_status = server.last_stats_time = 100
        server.worker.latest.return_value = dict(camera_generation=1, captured_at=99.98, boxes=[],
                                               completed_count=1, inference_ms=20)
        with patch('apptirover.vision.stream.time.monotonic', return_value=100):
            self.assertTrue(server.tick())
        server.publish.assert_called_once()
        server.publish.reset_mock()
        server.worker.latest.return_value = None
        with patch('apptirover.vision.stream.time.monotonic', return_value=100.01):
            self.assertTrue(server.tick())
        server.publish.assert_not_called()

    def test_duplicate_frame_notifications_do_not_repeat_inference_or_age_tracks(self):
        channel = Mock()
        channel.recv.side_effect = [b'frame', BlockingIOError(), b'frame', BlockingIOError(), b'stop']
        detector = Mock()
        detector.predict.return_value = [[.4, .1, .6, .9, .4, 0]]
        with patch('apptirover.vision.detection.Detector', return_value=detector), \
                patch('apptirover.vision.detection.select.select', return_value=([channel], [], [])), \
                patch('apptirover.vision.lifecycle.die_with_parent'), \
                patch('apptirover.vision.detection.signal.signal'):
            _worker('/unused', 2, .35, .45, bytearray(32 * 32 * 3), (32, 32, 3), Mock(acquire=Mock(return_value=True)),
                    SimpleNamespace(value=10.), SimpleNamespace(value=1), SimpleNamespace(value=1), channel, 1)
        self.assertEqual(detector.predict.call_count, 2)  # Warmup + one distinct frame.
        results = [json.loads(call.args[0]) for call in channel.send.call_args_list]
        self.assertFalse([packet for packet in results if packet['type'] == 'error'], results)
        results = [packet for packet in results if packet['type'] == 'result']
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0]['tracks'][0]['seen_at'], 10.)
        self.assertEqual(results[0]['completed_count'], 1)

    def test_dead_detector_socket_does_not_break_camera_cleanup(self):
        worker = object.__new__(DetectionWorker)
        worker.ready, worker.stop_at = True, None
        worker.channel = Mock()
        worker.channel.send.side_effect = OSError(errno.ENOTCONN, 'Transport endpoint is not connected')
        worker.begin_stop()
        self.assertFalse(worker.ready)
        self.assertIsNotNone(worker.stop_at)

    def test_capture_recovery_invalidates_results_and_has_bounded_budget(self):
        server = CameraServer(CameraConfig(no_detection=True))
        server.pipeline = Mock()
        server.pipeline.set_state.return_value = Gst.StateChangeReturn.SUCCESS
        server.fail = Mock()
        server.capture_started = server.last_video_time = 0
        server.observation = {'boxes': [[0, 0, 1, 1, .9, 0]]}
        with patch('apptirover.vision.stream.time.monotonic', return_value=4):
            server.watch_capture()
        server.pipeline.set_state.assert_not_called()
        with self.assertLogs('apptirover.vision.stream', level='WARNING'):
            for now in (10, 20, 30):
                with patch('apptirover.vision.stream.time.monotonic', return_value=now):
                    self.assertTrue(server.watch_capture())
            with patch('apptirover.vision.stream.time.monotonic', return_value=40), \
                    patch('apptirover.vision.stream.faulthandler.dump_traceback'):
                self.assertFalse(server.watch_capture())
        self.assertIsNone(server.observation)
        self.assertEqual(server.generation, 4)
        self.assertEqual(server.restarts, 3)
        server.fail.assert_called_once()

    def test_keyboard_toggle_needs_no_enter_and_restores_terminal(self):
        master, slave = pty.openpty()
        try:
            original = termios.tcgetattr(slave)
            with os.fdopen(os.dup(slave), 'r') as stdin, patch('sys.stdin', stdin):
                keys = TerminalKeys(True)
                try:
                    self.assertEqual(keys.read(), '')
                    os.write(master, b'D')
                    import select
                    self.assertTrue(select.select([slave], [], [], 1)[0])
                    self.assertEqual(keys.read(), 'd')
                finally:
                    keys.close()
            self.assertEqual(termios.tcgetattr(slave), original)
        finally:
            os.close(master)
            os.close(slave)


if __name__ == '__main__':
    unittest.main()
