"""Keep child processes from surviving their owner on this Linux rover."""

import ctypes
import os
import signal


def die_with_parent(expected_pid):
    libc = ctypes.CDLL(None, use_errno=True)
    if libc.prctl(1, signal.SIGKILL, 0, 0, 0) != 0:  # PR_SET_PDEATHSIG
        raise OSError(ctypes.get_errno(), 'Could not register parent-death signal')
    if os.getppid() != expected_pid:
        os.kill(os.getpid(), signal.SIGKILL)
