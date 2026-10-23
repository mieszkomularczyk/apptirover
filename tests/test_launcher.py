import subprocess
import sys
import unittest
from unittest.mock import patch

from apptirover import launcher


class LauncherTests(unittest.TestCase):
    def test_source_and_frozen_camera_commands_preserve_literal_arguments(self):
        arguments = ('7', '{"image": "/tmp/a b.jpg"}', '123')
        with patch.object(sys, 'frozen', False, create=True):
            self.assertEqual(launcher.camera_worker_command(*arguments),
                             [sys.executable, '-m', 'apptirover.vision.worker', *arguments])
        with patch.object(sys, 'frozen', True, create=True):
            self.assertEqual(launcher.camera_worker_command(*arguments),
                             [sys.executable, '--camera-worker', *arguments])

    def test_freeze_support_runs_before_cli_dispatch(self):
        events = []
        with patch.object(sys, 'argv', ['apptirover', '--help']), \
                patch.object(launcher.multiprocessing, 'freeze_support', side_effect=lambda: events.append('freeze')), \
                patch('apptirover.app.main', side_effect=lambda: events.append('cli') or 0):
            self.assertEqual(launcher.main(), 0)
        self.assertEqual(events, ['freeze', 'cli'])

    def test_internal_worker_dispatch_never_starts_hardware_monitors(self):
        with patch.object(sys, 'argv', ['apptirover', '--camera-worker', '7', '{}', '123']), \
                patch.object(launcher.multiprocessing, 'freeze_support'), \
                patch('apptirover.vision.worker.main', return_value=0) as worker, \
                patch('apptirover.app.main') as app:
            self.assertEqual(launcher.main(), 0)
        worker.assert_called_once_with(['7', '{}', '123'])
        app.assert_not_called()

    def test_launcher_import_keeps_native_libraries_out_of_main(self):
        result = subprocess.run([sys.executable, '-c',
            'import apptirover.launcher,sys; assert not ({"gi", "numpy", "cv2", "ncnn"} & sys.modules.keys())'],
            capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__':
    unittest.main()
