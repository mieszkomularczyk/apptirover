"""Dispatch the CLI and isolated camera process in source and frozen builds."""

import multiprocessing
import sys


CAMERA_WORKER = '--camera-worker'


def camera_worker_command(*arguments):
    if getattr(sys, 'frozen', False):
        return [sys.executable, CAMERA_WORKER, *arguments]
    return [sys.executable, '-m', 'apptirover.vision.worker', *arguments]


def main():
    # A frozen multiprocessing child must be diverted before importing the app,
    # parsing CLI arguments or starting any hardware monitors.
    multiprocessing.freeze_support()
    if sys.argv[1:2] == [CAMERA_WORKER]:
        from .vision.worker import main as worker_main
        return worker_main(sys.argv[2:])
    from .app import main as app_main
    return app_main()
