"""Single motion authority: faults, manual takeover, then expiring following."""

import math

from .following import Follower, PressToggle


def mix(throttle, turn, max_power):
    """Positive throttle goes forward; positive turn rotates the nose right."""
    if not all(type(value) in (int, float) and math.isfinite(value)
               for value in (throttle, turn, max_power)):
        raise ValueError('Drive values must be finite numbers')
    if not -1 <= throttle <= 1 or not -1 <= turn <= 1 or not 0 < max_power <= 1:
        raise ValueError('Drive values are outside their normalized ranges')
    left, right = throttle + turn, throttle - turn
    scale = max(1, abs(left), abs(right))
    return left / scale * max_power, right / scale * max_power


class DriveControl:
    def __init__(self, max_power=0.5, follow_config=None):
        mix(0, 0, max_power)
        self.max_power = max_power
        self.armed = False
        self.neutral_since = None
        self.session = None
        self.previous_at = None
        self.left = self.right = 0.0
        self.reason = 'waiting for controller and rover'
        self.throttle = self.turn = 0.0
        self.follower = Follower(follow_config)
        self.toggle = PressToggle(self.follower.config.button)
        self.now = None

    def stop(self, reason):
        if self.follower.active:
            self.follower.cancel(reason + '; X to re-enable')
        self.armed = False
        self.neutral_since = None
        self.left = self.right = 0.0
        self.throttle = self.turn = 0.0
        self.reason = reason

    def step(self, pad, rover, now, vision=None):
        self.now = now
        presses = self.toggle.update(pad)
        gap = now - self.previous_at if self.previous_at is not None else 0
        dt = min(0.1, max(0, gap))
        self.previous_at = now
        session = (rover.get('generation'), pad.get('input_generation'),
                   (pad.get('input') or {}).get('dropped', 0))
        if session != self.session:
            self.stop('center sticks after connection')
            self.session = session
        base = rover.get('packets', {}).get('1001', {})
        fault = None
        if gap > 0.25:
            fault = 'drive loop stalled; center sticks'
        elif not rover.get('port_open') or not base.get('fresh') or base.get('age_s', 999) > 1:
            fault = 'rover feedback unavailable'
        elif not pad.get('connected') or not pad.get('input_ready') or not pad.get('input'):
            fault = 'controller input unavailable'
        elif pad.get('worker_age_s') is None or pad['worker_age_s'] > 0.25:
            fault = 'controller worker stalled'
        elif pad.get('link_age_s') is None or pad['link_age_s'] > 0.75:
            fault = 'Bluetooth status stale'
        elif pad['input'].get('resyncing'):
            fault = 'input events lost; resynchronizing'
        if fault:
            self.stop(fault)
            return self.snapshot()
        try:
            sticks = pad['input']['sticks']
            throttle, turn = -sticks['left_y'], sticks['right_x']
            target_left, target_right = mix(throttle, turn, self.max_power)
        except (KeyError, TypeError, ValueError):
            self.stop('invalid joystick input')
            return self.snapshot()
        self.throttle, self.turn = throttle, turn
        neutral = throttle == 0 and turn == 0
        if not self.armed:
            if neutral:
                if self.neutral_since is None:
                    self.neutral_since = now
                self.armed = now - self.neutral_since >= 0.25
            else:
                self.neutral_since = None
            self.reason = 'ready' if self.armed else 'center sticks to enable drive'
            return self.snapshot()
        if presses and (self.follower.active or presses > 1):
            self.follower.cancel('X: following off', disabled=True)
            self.left = self.right = self.throttle = self.turn = 0.0
            self.reason = 'ready'
            return self.snapshot()
        if not neutral and self.follower.active:
            self.follower.cancel('manual takeover; X to re-enable')
            self.left = self.right = 0.0
        elif presses:
            if neutral:
                self.follower.activate(vision, now)
        if self.follower.active:
            throttle, turn = self.follower.step(vision, now)
            if self.follower.active:
                self.throttle, self.turn = throttle, turn
                target_left, target_right = mix(throttle, turn, min(self.max_power, self.follower.config.max_power))
                self.reason = self.follower.state
                self._output(target_left, target_right, dt)
                return self.snapshot()
            # A vision fault revokes its authority immediately, bypassing ramps.
            self.left = self.right = self.throttle = self.turn = 0.0
            self.reason = 'following stopped'
            return self.snapshot()
        if neutral:
            self.left = self.right = 0.0  # Stop bypasses all smoothing.
            self.reason = 'ready'
        else:
            # Limit changes to 200 percentage points of full power per second.
            # Both sides use the same interpolation factor, preserving the turn ratio.
            self._output(target_left, target_right, dt)
            self.reason = 'driving'
        return self.snapshot()

    def _output(self, left, right, dt):
        if left == right == 0:
            self.left = self.right = 0.0
            return
        delta = max(abs(left - self.left), abs(right - self.right))
        fraction = min(1, 2 * dt / delta) if delta else 1
        self.left += (left - self.left) * fraction
        self.right += (right - self.right) * fraction

    def snapshot(self):
        return dict(enabled=True, armed=self.armed, state=self.reason, max_power=self.max_power,
                    power_limit=min(self.max_power, self.follower.config.max_power) if self.follower.active else self.max_power,
                    throttle=self.throttle, turn=self.turn,
                    left_power=round(self.left, 6), right_power=round(self.right, 6),
                    authority='following' if self.follower.active else 'manual',
                    following=self.follower.snapshot(self.now))
