"""Small expiring motion requests; all neural inference/association runs elsewhere."""

from collections import deque
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class FollowConfig:
    button: str = 'BTN_NORTH'  # X / top face button on the Lite 2 in D mode.
    desired_width: float = .15
    max_power: float = .25
    max_age: float = .25
    acquire_seconds: float = 1.
    yaw_kp: float = 2.5
    yaw_kd: float = .12
    distance_kp: float = .7
    distance_kd: float = .05

    def __post_init__(self):
        values = (self.desired_width, self.max_power, self.max_age, self.acquire_seconds,
                  self.yaw_kp, self.yaw_kd, self.distance_kp, self.distance_kd)
        if (not all(type(v) in (int, float) and math.isfinite(v) for v in values)
                or not .03 <= self.desired_width <= .8 or not .05 <= self.max_power <= 1
                or not .05 <= self.max_age <= .5 or not .1 <= self.acquire_seconds <= 3
                or min(self.yaw_kp, self.yaw_kd, self.distance_kp, self.distance_kd) < 0):
            raise ValueError('Following width/power outside supported bounds')


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
        self.reason = 'X to follow a person'
        self.session = 0
        self.epoch = None
        self.target_id = None
        self.last_key = None
        self.history = deque(maxlen=64)
        self.target = None
        self.source_at = None
        self.throttle = self.turn = 0.

    @property
    def active(self):
        return self.state in ('acquiring', 'following')

    def cancel(self, reason, disabled=False):
        self.state = 'disabled' if disabled else 'stopped'
        self.reason = reason
        self.history.clear()
        self.target_id = self.target = self.source_at = self.last_key = self.epoch = None
        self.throttle = self.turn = 0.

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
        self.cancel('acquiring a stable person')
        try:
            _, self.epoch, _ = self._observation(vision, now)
        except (KeyError, TypeError, ValueError) as error:
            self.reason = str(error)
            return
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
                    and 0 <= box[0] < box[2] <= 1 and 0 <= box[1] < box[3] <= 1
                    and .005 < measured[0] < measured[2] < .995 and 0 <= measured[1] < measured[3] <= 1
                    and box[2] - box[0] >= .02 and box[3] - box[1] >= .05)
        except (KeyError, TypeError, ValueError):
            return False

    def step(self, vision, now):
        self.throttle = self.turn = 0.
        if not self.active:
            return 0., 0.
        try:
            observation, epoch, key = self._observation(vision, now)
            if epoch != self.epoch:
                raise ValueError('camera/detector restarted; X to re-enable')
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
                if self.state == 'acquiring':
                    self._acquire(tracks, captured)
                else:
                    self.target = next((track for track in tracks if track['id'] == self.target_id), None)
                if self.state == 'following':
                    if self.target is None:
                        raise ValueError('target lost; X to acquire again')
                    if self.target.get('ambiguous', True):
                        raise ValueError('target identity ambiguous; X to acquire again')
            if self.state == 'following':
                self.throttle, self.turn = self._motion(self.target)
        except (KeyError, TypeError, ValueError, OverflowError) as error:
            self.cancel(str(error))
        return self.throttle, self.turn

    def _acquire(self, tracks, now):
        candidates = [track for track in tracks if not track.get('ambiguous', True)]
        self.history.append((now, {track['id']: track['score'] for track in candidates}))
        while self.history and now - self.history[0][0] > self.config.acquire_seconds + .1:
            self.history.popleft()
        if not self.history or now - self.history[0][0] < self.config.acquire_seconds:
            return
        recent = [scores for at, scores in self.history if now - at <= self.config.acquire_seconds]
        ranked = []
        for track in candidates:
            scores = [scores[track['id']] for scores in recent if track['id'] in scores]
            if len(scores) >= 3 and len(scores) / len(recent) >= .8:
                ranked.append((len(scores), sum(scores) / len(scores), track))
        ranked.sort(key=lambda item: (item[0], item[1]), reverse=True)
        if not ranked:
            self.reason = 'waiting for a persistent person'
            return
        if len(ranked) > 1 and ranked[0][0] == ranked[1][0] and ranked[0][1] - ranked[1][1] < .08:
            self.reason = 'multiple similar candidates; remain stopped'
            return
        self.target = ranked[0][2]
        self.target_id = self.target['id']
        self.state, self.reason = 'following', 'following locked person'
        self.history.clear()

    def _motion(self, target):
        c = self.config
        x1, _, x2, _ = target['box']
        center, width = (x1 + x2) / 2, x2 - x1
        yaw_error = center - .5  # Rover positive turn is right; ROS example uses the opposite sign.
        distance_error = math.log(c.desired_width / width)
        turn = c.yaw_kp * yaw_error + c.yaw_kd * target['velocity'][0] if abs(yaw_error) > .03 else 0.
        throttle = (c.distance_kp * distance_error - c.distance_kd * target['velocity'][2] / width
                    if abs(distance_error) > math.log(1.1) else 0.)
        # Align before translating, in both forward and reverse.
        throttle *= max(0., 1 - abs(yaw_error) / .3)
        return max(-1., min(1., throttle)), max(-1., min(1., turn))

    def snapshot(self, now=None):
        box = self.target['box'] if self.target else None
        return dict(state=self.state, reason=self.reason, active=self.active, session=self.session,
                    target_id=self.target_id, desired_width=self.config.desired_width,
                    center_x=(box[0] + box[2]) / 2 if box else None,
                    width=box[2] - box[0] if box else None,
                    observation_age_s=now - self.source_at if now is not None and self.source_at is not None else None,
                    max_power=self.config.max_power, button=self.config.button)
