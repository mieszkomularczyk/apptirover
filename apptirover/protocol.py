"""Bounded newline-delimited JSON decoding for the ESP32 serial connection."""

import json
import math


BASE_QUERY = b'{"T":130}\n'
IMU_QUERY = b'{"T":126}\n'
QUERY_TYPES = {126, 130}
CONTROL_TYPES = {1, 136}
TELEMETRY_TYPES = {1001, 1002}


def motor_command(left, right):
    if not all(type(value) in (int, float) and math.isfinite(value) and -1 <= value <= 1
               for value in (left, right)):
        raise ValueError('Motor power must be finite and within [-1, 1]')
    # General Driver multiplies T=1 values by 512; keep 8-bit PWM <=255.
    # Explicit floats are required by this firmware's JSON type checks, even at zero.
    return (json.dumps({'T': 1, 'L': round(float(left) * 255 / 512, 6),
                        'R': round(float(right) * 255 / 512, 6)}, separators=(',', ':')) + '\n').encode()


STOP_COMMAND = motor_command(0, 0)
WATCHDOG_COMMAND = b'{"T":136,"cmd":500}\n'


def finite_json(value):
    if isinstance(value, float):
        return math.isfinite(value)
    if isinstance(value, dict):
        return all(finite_json(item) for item in value.values())
    if isinstance(value, list):
        return all(finite_json(item) for item in value)
    return True


class JsonLines:
    """Preserve partial lines and discard an oversized line through its newline."""

    def __init__(self, max_line=8192):
        self.max_line = max_line
        self.buffer = bytearray()
        self.discarding = False
        self.invalid_lines = 0

    def feed(self, chunk):
        messages = []
        pieces = chunk.split(b"\n")
        for index, piece in enumerate(pieces):
            complete = index < len(pieces) - 1
            if not self.discarding:
                if len(self.buffer) + len(piece) > self.max_line:
                    self.invalid_lines += 1
                    self.buffer.clear()
                    self.discarding = True
                else:
                    self.buffer.extend(piece)
            if complete:
                if not self.discarding and self.buffer.strip():
                    try:
                        message = json.loads(self.buffer)
                        if not isinstance(message, dict) or not finite_json(message):
                            raise ValueError("Expected a finite JSON object")
                        if type(message.get("T")) is not int:
                            raise ValueError("Expected an integer message type")
                        messages.append(message)
                    except (ValueError, UnicodeError, RecursionError):
                        self.invalid_lines += 1
                self.buffer.clear()
                self.discarding = False
        return messages
