"""Small expiring motion requests; all neural inference/association runs elsewhere."""

from collections import deque
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class FollowConfig:
    button: str = 'BTN_NORTH'  # X / top face button on the Lite 2 in D mode.
    desired_width: float | None = None
    distance_scale: float = 1.25
    max_power: float = 1.
    min_power: float = .5
    max_age: float = .25
    acquire_seconds: float = 1.
    turn_limit: float = .25
    pulse_seconds: float = .06
    yaw_kp: float = .65
    yaw_ki: float = .45
    yaw_kd: float = .08
    distance_kp: float = .35
    distance_ki: float = .45
    distance_kd: float = .05
    reacquire_seconds: float = 1.

    def __post_init__(self):
        values = (self.distance_scale, self.turn_limit, self.pulse_seconds,
                  self.max_power, self.min_power, self.max_age, self.acquire_seconds,
                  self.yaw_kp, self.yaw_ki, self.yaw_kd, self.distance_kp, self.distance_ki,
                  self.distance_kd, self.reacquire_seconds)
        if (not all(type(v) in (int, float) and math.isfinite(v) for v in values)
                or (self.desired_width is not None and (type(self.desired_width) not in (int, float)
                    or not math.isfinite(self.desired_width) or not .03 <= self.desired_width <= .8))
                or not .05 <= self.max_power <= 1
                or not 1 <= self.distance_scale <= 2 or not .05 <= self.turn_limit <= 1
                or not .04 <= self.pulse_seconds <= .15
                or not 0 <= self.min_power <= 1 or not .1 <= self.reacquire_seconds <= 3
                or not .05 <= self.max_age <= .5 or not .1 <= self.acquire_seconds <= 3
                or min(self.yaw_kp, self.yaw_ki, self.yaw_kd, self.distance_kp, self.distance_ki, self.distance_kd) < 0):
            raise ValueError('Following configuration outside supported bounds')


class BoundedPID:
    """Source-time PID with bounded integral, conditional anti-windup and damping.

    A derivative estimate cannot cancel/reverse a sustained position error.
    Duplicate snapshots never integrate again. Reset whenever motion is blocked.
    """

    def __init__(self, kp, ki, kd, limit=1.):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.limit = limit
        self.reset()

    def reset(self):
        self.at = self.error = None
        self.integral = self.output = 0.

    def update(self, error, rate, at, deadband):
        if abs(error) <= deadband:
            self.reset()
            return 0.
        if self.at is not None and at <= self.at:
            return self.output
        dt = min(.2, at - self.at) if self.at is not None else 0.
        if self.error is not None and error * self.error < 0:
            self.integral = 0.
        elif self.error and abs(error) < abs(self.error):
            # Release stored effort as the rover closes the error, rather than
            # carrying a large integral into the target and overshooting it.
            self.integral *= abs(error / self.error)
        proportional = self.kp * error
        bound = min(.5, self.limit)
        candidate = max(-bound, min(bound, self.integral + self.ki * error * dt))
        damping_limit = abs(proportional + candidate) / 2
        damping = max(-damping_limit, min(damping_limit, self.kd * rate))
        demand = proportional + damping + candidate
        if not ((demand > self.limit and error > 0) or (demand < -self.limit and error < 0)):
            self.integral = candidate
        self.at, self.error = at, error
        demand = proportional + damping + self.integral
        self.output = max(0., min(self.limit, demand)) if error > 0 else min(0., max(-self.limit, demand))
        return self.output


class PressToggle:
    """Consume counted physical presses, including taps between control ticks."""

    def __init__(self, button):
        self.button = button
        self.session = self.count = self.held = None

    def update(self, pad):
        inputs = pad.get('input') or {}
        if not isinstance(inputs, dict) or not pad.get('input_ready') or inputs.get('resyncing'):
            self.session = None
            return 0
        session = (pad.get('input_generation'), inputs.get('dropped', 0))
        counts = inputs.get('button_presses') or {}
        count = counts.get(self.button, 0) if isinstance(counts, dict) else 0
        held = self.button in (inputs.get('buttons') or [])
        if session != self.session:
            self.session, self.count, self.held = session, count, held
            return 0
        delta = count - self.count if type(count) is int and type(self.count) is int else 0
        # Older/synthetic input sources can still expose ordinary button edges.
        if 'button_presses' not in inputs:
            delta = int(held and not self.held)
        self.count, self.held = count, held
        return max(0, delta)


class Follower:
    def __init__(self, config=None):
        self.config = config or FollowConfig()
        self.state = 'disabled'
        self.enabled = False
        self.manual_override = False
        self.reason = 'X to follow a person'
        self.session = 0
        self.epoch = None
        self.target_id = None
        self.last_key = None
        self.history = deque(maxlen=64)
        self.target = None
        self.source_at = None
        self.last_target = None
        self.throttle = self.turn = 0.
        self.desired_width = self.config.desired_width
        self.yaw_error = self.distance_error = None
        self.yaw_pid = BoundedPID(self.config.yaw_kp, self.config.yaw_ki, self.config.yaw_kd,
                                  limit=self.config.turn_limit)
        self.distance_pid = BoundedPID(self.config.distance_kp, self.config.distance_ki, self.config.distance_kd)

    def reset_pid(self):
        self.yaw_pid.reset()
        self.distance_pid.reset()

    def set_manual_override(self, override):
        if bool(override) != self.manual_override:
            self.reset_pid()
        self.manual_override = bool(override)

    @property
    def active(self):
        return self.enabled

    def suspend(self, reason, reset=False):
        """Revoke motion without changing the user's X-controlled mode."""
        self.state = 'waiting' if self.enabled else 'disabled'
        self.reason = reason
        self.history.clear()
        self.target = None
        self.throttle = self.turn = 0.
        self.yaw_error = self.distance_error = None
        self.reset_pid()
        if reset:
            self.target_id = self.last_target = self.source_at = self.last_key = self.epoch = None

    def disable(self):
        self.enabled = False
        self.manual_override = False
        self.desired_width = self.config.desired_width
        self.suspend('X: following off', reset=True)

    def _observation(self, vision, now):
        if not vision or not vision.get('enabled'):
            raise ValueError('camera disabled')
        if not vision.get('detection_enabled'):
            raise ValueError('detection disabled; press D first')
        observation = vision.get('observation')
        if not vision.get('observation_valid') or not vision.get('stream_ready') or not observation:
            raise ValueError('no fresh person observations')
        captured = observation['captured_at']
        completed = observation['completed_at']
        if not all(type(v) in (int, float) and math.isfinite(v) for v in (captured, completed)):
            raise ValueError('invalid observation timestamp')
        if not 0 <= now - captured <= self.config.max_age or not captured <= completed <= now:
            raise ValueError('person observation stale')
        epoch = (observation['process_generation'], observation['camera_generation'], observation['detection_generation'])
        if (epoch != (vision['generation'], vision['camera_generation'], vision['detection_generation'])
                or vision.get('revision') != vision.get('requested_revision')):
            raise ValueError('camera/detector session changed')
        sequence = observation['frame_sequence']
        if any(type(v) is not int or v < 0 for v in (*epoch, sequence)):
            raise ValueError('invalid observation generation')
        tracks = observation['tracks']
        if not isinstance(tracks, list) or len(tracks) > 32:
            raise ValueError('invalid tracking result')
        return observation, epoch, (*epoch, sequence)

    def activate(self, vision, now):
        self.enabled = True
        self.desired_width = self.config.desired_width
        self.suspend('acquiring a stable person', reset=True)
        self.session += 1
        self.state = 'acquiring'

    @staticmethod
    def _valid_track(track, captured):
        try:
            box, measured, velocity = track['box'], track['measurement'], track['velocity']
            return (type(track['id']) is int and track['id'] > 0 and len(box) == len(measured) == len(velocity) == 4
                    and all(type(v) in (int, float) and math.isfinite(v) for v in (*box, *measured, *velocity, track['score']))
                    and 0 <= track['score'] <= 1 and track['seen_at'] == captured
                    and track['measured'] is True and track['confirmed'] is True
                    and -.05 <= box[0] < box[2] <= 1.05 and -.05 <= box[1] < box[3] <= 1.05
                    and 0 <= measured[0] < measured[2] <= 1 and 0 <= measured[1] < measured[3] <= 1
                    and .02 <= box[2] - box[0] <= 1.1 and .05 <= box[3] - box[1] <= 1.1)
        except (KeyError, TypeError, ValueError):
            return False

    def step(self, vision, now):
        self.throttle = self.turn = 0.
        if not self.active:
            return 0., 0.
        try:
            observation, epoch, key = self._observation(vision, now)
            if epoch != self.epoch:
                self.suspend('acquiring after camera/detector session change', reset=True)
                self.epoch = epoch
            captured = observation['captured_at']
            if key == self.last_key and captured != self.source_at:
                raise ValueError('source timestamp changed without a new frame')
            if key != self.last_key:
                if self.last_key and (key[-1] <= self.last_key[-1] or captured <= self.source_at):
                    raise ValueError('tracking observation moved backwards')
                self.last_key, self.source_at = key, captured
                tracks = [track for track in observation['tracks'] if self._valid_track(track, captured)]
                ids = [track['id'] for track in tracks]
                if len(set(ids)) != len(ids):
                    raise ValueError('duplicate person IDs')
                if self.target_id is not None:
                    self._continue(tracks, captured)
                else:
                    self.state = 'acquiring'
                    self._acquire(tracks, captured)
            if self.state == 'following':
                self.throttle, self.turn = self._motion(self.target)
                self.reason = ('turning toward person' if self.turn and not self.throttle else
                               'approaching person' if self.throttle > 0 else
                               'backing away' if self.throttle < 0 else
                               'show full person width to set distance' if self.desired_width is None else 'holding position')
        except (KeyError, TypeError, ValueError, OverflowError) as error:
            self.suspend(str(error))
        return self.throttle, self.turn

    def _continue(self, tracks, now):
        chosen = next((track for track in tracks if track['id'] == self.target_id), None)
        if chosen is None and self.last_target is not None:
            reference = self.last_target
            dt = now - reference['seen_at']
            if dt <= self.config.reacquire_seconds:
                x1, y1, x2, y2 = reference['box']
                cx = (x1 + x2) / 2 + reference['velocity'][0] * dt
                cy = (y1 + y2) / 2 + reference['velocity'][1] * dt
                nearby = []
                for track in tracks:
                    a, b, c, d = track['box']
                    if (not track.get('ambiguous', True)
                            and math.hypot((a + c) / 2 - cx, (b + d) / 2 - cy) <= .12
                            and .5 <= (c - a) / (x2 - x1) <= 2
                            and .5 <= (d - b) / (y2 - y1) <= 2):
                        nearby.append(track)
                if len(nearby) == 1:
                    chosen = nearby[0]
        if chosen is not None and not chosen.get('ambiguous', True):
            if chosen['id'] != self.target_id:
                self.reset_pid()
            self.target = self.last_target = chosen
            self.target_id = chosen['id']
            self.state = 'following'
            self.history.clear()
            return
        self.target = None
        self.reset_pid()
        self.state = 'waiting'
        self.reason = 'target ambiguous; waiting' if chosen is not None else 'person lost; searching'
        if self.last_target and now - self.last_target['seen_at'] > self.config.reacquire_seconds:
            self.target_id = self.last_target = None
            self.state = 'acquiring'
            self._acquire(tracks, now)

    def _acquire(self, tracks, now):
        candidates = [track for track in tracks if not track.get('ambiguous', True)]
        self.history.append((now, {track['id']: track['score'] for track in candidates}))
        while self.history and now - self.history[0][0] > self.config.acquire_seconds + .1:
            self.history.popleft()
        if not self.history or now - self.history[0][0] < self.config.acquire_seconds:
            self.reason = 'acquiring a stable person' if candidates else 'no person; searching'
            return
        recent = [scores for at, scores in self.history if now - at <= self.config.acquire_seconds]
        ranked = []
        for track in candidates:
            scores = [scores[track['id']] for scores in recent if track['id'] in scores]
            if len(scores) >= 3 and len(scores) / len(recent) >= .8:
                ranked.append((len(scores), sum(scores) / len(scores), track))
        ranked.sort(key=lambda item: (item[0], item[1], -round(abs((item[2]['box'][0] + item[2]['box'][2]) / 2 - .5), 6),
                                      -item[2]['id']), reverse=True)
        if not ranked:
            self.reason = 'waiting for a persistent person'
            return
        self.target = ranked[0][2]
        self.last_target = self.target
        self.target_id = self.target['id']
        self.reset_pid()
        self.state, self.reason = 'following', 'following locked person'
        self.history.clear()

    def _motion(self, target):
        c = self.config
        x1, _, x2, _ = target['box']
        x1, x2 = max(0., x1), min(1., x2)
        center, width = (x1 + x2) / 2, x2 - x1
        clipped = target['measurement'][0] <= .005 or target['measurement'][2] >= .995
        if self.desired_width is None and not clipped and .03 <= width <= .8:
            # Capture once per X session. Reacquisition/manual override retain it.
            self.desired_width = min(.8, width * c.distance_scale)
        self.yaw_error = center - .5  # Rover positive turn is right.
        self.distance_error = math.log(self.desired_width / width) if self.desired_width is not None else None
        if self.manual_override:
            self.reset_pid()
            return 0., 0.
        turn = self.yaw_pid.update(self.yaw_error, target['velocity'][0], target['seen_at'], .03)
        throttle = 0.
        # Clipping can make a close person look narrow. Still turn to recenter,
        # and allow reverse, but never approach using that underestimated width.
        if (self.distance_error is not None and abs(self.yaw_error) < .3
                and not (clipped and self.distance_error > 0)):
            throttle = self.distance_pid.update(self.distance_error, -target['velocity'][2] / width,
                                                 target['seen_at'], math.log(1.1))
        else:
            self.distance_pid.reset()
        # Align before translating, in both forward and reverse.
        throttle *= max(0., 1 - abs(self.yaw_error) / .3)
        return max(-1., min(1., throttle)), max(-1., min(1., turn))

    def snapshot(self, now=None):
        box = self.target['box'] if self.target else None
        return dict(state='manual' if self.enabled and self.manual_override else self.state,
                    tracking_state=self.state, reason='joystick priority' if self.enabled and self.manual_override else self.reason,
                    enabled=self.enabled, active=self.active, manual_override=self.manual_override,
                    motion_active=self.enabled and self.state == 'following' and not self.manual_override
                    and bool(self.throttle or self.turn),
                    target_valid=self.state == 'following', tracking_reason=self.reason,
                    session=self.session, throttle=self.throttle, turn=self.turn,
                    target_id=self.target_id, desired_width=self.desired_width,
                    width_reference='automatic' if self.config.desired_width is None else 'fixed',
                    distance_scale=self.config.distance_scale if self.config.desired_width is None else 1.,
                    yaw_error=self.yaw_error, distance_error=self.distance_error,
                    yaw_integral=self.yaw_pid.integral, distance_integral=self.distance_pid.integral,
                    center_x=(box[0] + box[2]) / 2 if box else None,
                    width=box[2] - box[0] if box else None,
                    observation_age_s=now - self.source_at if now is not None and self.source_at is not None else None,
                    max_power=self.config.max_power, min_power=self.config.min_power,
                    turn_limit=self.config.turn_limit, button=self.config.button)
