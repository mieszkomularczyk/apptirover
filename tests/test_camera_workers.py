import errno
import os
import pty
import termios
import unittest
from unittest.mock import Mock, patch

from apptirover.camera import CameraConfig
from apptirover.terminal import TerminalKeys
from apptirover.vision.detection import DetectionWorker
from apptirover.vision.stream import CameraServer, Gst


class WorkerLifecycleTests(unittest.TestCase):
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
