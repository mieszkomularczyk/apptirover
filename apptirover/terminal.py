"""Optional terminal keys; joystick input is handled separately by evdev."""

import os
import select
import sys
import termios
import tty


class TerminalKeys:
    def __init__(self, enabled):
        self.fd = None
        self.saved = None
        if enabled and sys.stdin.isatty():
            self.fd = sys.stdin.fileno()
            self.saved = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd, termios.TCSANOW)

    def read(self):
        if self.fd is not None and select.select([self.fd], [], [], 0)[0]:
            return os.read(self.fd, 64).decode(errors='ignore').lower()
        return ''

    def close(self):
        if self.saved is not None:
            termios.tcsetattr(self.fd, termios.TCSANOW, self.saved)
