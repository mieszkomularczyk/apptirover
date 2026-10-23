"""Single motion authority: faults, manual takeover, then expiring following."""

import math

from .following import Follower, PressToggle

CONTROL_INTERVAL = .02


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


class FollowingOutput:
    """Slew average effort, then deliver small requests as short torque pulses.

    Credit accounts for actual on/off time: a 10% average request delivered at
    50% pulse power spends four times as long off as on. One control tick of
    credit absorbs timing quantization. No sleeps or queued motor commands.
    """

    def __init__(self, pulse_seconds):
        self.pulse_seconds = pulse_seconds
        self.reset()

    def reset(self):
        self.left = self.right = 0.
        self.credit = self.previous_peak = 0.
        self.at = self.until = None
        self.pulsed = self.active = False
        self.duty = 0.

    def update(self, throttle, turn, maximum, minimum, now, dt):
        left, right = mix(throttle, turn, maximum)
        if left == right == 0:
            self.reset()
            return 0., 0.
        if left * self.left < 0 or right * self.right < 0:
            # Do not deliver residual effort in the old direction. Give the
            # UART one explicit zero before starting a reversed correction.
            self.reset()
            return 0., 0.
        delta = max(abs(left - self.left), abs(right - self.right))
        fraction = min(1., 2 * dt / delta) if delta else 1.
        self.left += (left - self.left) * fraction
        self.right += (right - self.right) * fraction
        peak = max(abs(self.left), abs(self.right))
        floor = min(minimum, maximum)
        elapsed = max(0., now - self.at) if self.at is not None else 0.
        self.at = now
        # Above this duty there is no room for an off tick between short
        # pulses. Deliver the requested continuous PWM instead; it has already
        # passed through breakaway pulses during the average-effort ramp.
        self.pulsed = 0 < peak < floor * self.pulse_seconds / (self.pulse_seconds + CONTROL_INTERVAL)
        if not self.pulsed:
            self.credit = 0.
            self.until = None
            self.active = bool(peak)
            self.duty = 1. if peak else 0.
            self.previous_peak = peak
            return self.left, self.right
        self.duty = peak / floor
        self.credit = min(floor * CONTROL_INTERVAL, self.credit + (peak - self.previous_peak) * elapsed)
        if self.until is not None and now + 1e-9 < self.until:
            self.active = True
        elif self.previous_peak:
            # Every pulse has an explicit off edge, even if demand increased.
            self.until = None
            self.active = False
        elif self.credit >= -1e-9:
            self.until = now + self.pulse_seconds
            self.active = True
        else:
            self.active = False
        self.previous_peak = floor if self.active else 0.
        if not self.active:
            return 0., 0.
        return self.left * floor / peak, self.right * floor / peak

    def snapshot(self):
        return dict(average_left=round(self.left, 6), average_right=round(self.right, 6),
                    pulsed=self.pulsed, pulse_active=self.active, duty=round(self.duty, 6),
                    pulse_seconds=self.pulse_seconds)


class DriveControl:
    def __init__(self, max_power=1.0, follow_config=None, manual_power=.5):
        mix(0, 0, max_power)
        mix(0, 0, manual_power)
        self.max_power = max_power
        self.manual_power = min(manual_power, max_power)
        self.armed = False
        self.neutral_since = None
        self.session = None
        self.previous_at = None
        self.left = self.right = 0.0
        self.reason = 'waiting for controller and rover'
        self.throttle = self.turn = 0.0
        self.follower = Follower(follow_config)
        self.following_output = FollowingOutput(self.follower.config.pulse_seconds)
        self.toggle = PressToggle(self.follower.config.button)
        self.now = None

    def stop(self, reason):
        self.following_output.reset()
        if self.follower.active:
            self.follower.suspend(reason, reset=True)
        self.follower.manual_override = False
        self.armed = False
        self.neutral_since = None
        self.left = self.right = 0.0
        self.throttle = self.turn = 0.0
        self.reason = reason

    def step(self, pad, rover, now, vision=None):
        self.now = now
        presses = self.toggle.update(pad)
        toggled_off = False
        if presses % 2:
            if self.follower.enabled:
                self.follower.disable()
                toggled_off = True
            else:
                self.follower.activate(vision, now)
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
            target_left, target_right = mix(throttle, turn, self.manual_power)
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
        if toggled_off:
            self.following_output.reset()
            self.left = self.right = self.throttle = self.turn = 0.0
            self.reason = 'ready'
            return self.snapshot()
        if self.follower.active:
            override = not neutral
            if override != self.follower.manual_override:
                self.left = self.right = 0.0
                self.following_output.reset()
            self.follower.set_manual_override(override)
            throttle, turn = self.follower.step(vision, now)
            if not override:
                self.throttle, self.turn = throttle, turn
                self.left, self.right = self.following_output.update(
                    throttle, turn, min(self.max_power, self.follower.config.max_power),
                    self.follower.config.min_power, now, dt)
                self.reason = self.follower.state
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
                    manual_power=self.manual_power,
                    power_limit=min(self.max_power, self.follower.config.max_power)
                    if self.follower.active and not self.follower.manual_override else self.manual_power,
                    throttle=self.throttle, turn=self.turn,
                    left_power=round(self.left, 6), right_power=round(self.right, 6),
                    authority='following' if self.follower.active and not self.follower.manual_override and self.armed else 'manual',
                    following_output=self.following_output.snapshot(),
                    following=self.follower.snapshot(self.now))
