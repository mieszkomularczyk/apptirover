"""Bounded person association, run in the inference process, never the UART loop.

ByteTrack-style two-stage matching: strong detections establish tracks; weaker
detections can continue an existing track. Four independent position/velocity
Kalman filters use source-frame time. This is a small standalone implementation,
not the Ultralytics tracker or an appearance/identity recognition model.
"""

from dataclasses import dataclass
import math


def box_iou(a, b):
    intersection = max(0, min(a[2], b[2]) - max(a[0], b[0])) * max(0, min(a[3], b[3]) - max(a[1], b[1]))
    union = max(0, a[2] - a[0]) * max(0, a[3] - a[1]) + max(0, b[2] - b[0]) * max(0, b[3] - b[1]) - intersection
    return intersection / union if union > 0 else 0.0


def assignment(costs):
    """Minimum-cost rectangular assignment; columns >= rows, including dummies.

    Successive shortest augmenting paths keep association global rather than
    letting a greedy first match steal another person's only plausible box.
    """
    if not costs:
        return []
    n, m = len(costs), len(costs[0])
    u, v, owner, previous = [0.] * (n + 1), [0.] * (m + 1), [0] * (m + 1), [0] * (m + 1)
    for row in range(1, n + 1):
        owner[0] = row
        column = 0
        distance, used = [math.inf] * (m + 1), [False] * (m + 1)
        while True:
            used[column] = True
            current = owner[column]
            delta, next_column = math.inf, 0
            for candidate in range(1, m + 1):
                if not used[candidate]:
                    cost = costs[current - 1][candidate - 1] - u[current] - v[candidate]
                    if cost < distance[candidate]:
                        distance[candidate], previous[candidate] = cost, column
                    if distance[candidate] < delta:
                        delta, next_column = distance[candidate], candidate
            for candidate in range(m + 1):
                if used[candidate]:
                    u[owner[candidate]] += delta
                    v[candidate] -= delta
                else:
                    distance[candidate] -= delta
            column = next_column
            if owner[column] == 0:
                break
        while column:
            owner[column] = owner[previous[column]]
            column = previous[column]
    return [(owner[column] - 1, column - 1) for column in range(1, m + 1) if owner[column]]


@dataclass
class AxisFilter:
    position: float
    velocity: float = 0.
    pp: float = .0004
    pv: float = 0.
    vv: float = .0625

    def predict(self, dt, acceleration):
        self.position += dt * self.velocity
        q = acceleration ** 2
        self.pp += 2 * dt * self.pv + dt * dt * self.vv + q * dt ** 4 / 4
        self.pv += dt * self.vv + q * dt ** 3 / 2
        self.vv += q * dt * dt

    def correct(self, value, variance):
        innovation = value - self.position
        total = self.pp + variance
        kp, kv = self.pp / total, self.pv / total
        self.position += kp * innovation
        self.velocity += kv * innovation
        pp, pv = self.pp, self.pv
        self.pp = max(1e-12, pp - kp * pp)
        self.pv = pv - kp * pv
        self.vv = max(1e-12, self.vv - kv * pv)


class Track:
    def __init__(self, track_id, row, now):
        x1, y1, x2, y2 = row[:4]
        self.id = track_id
        self.axes = [AxisFilter(value) for value in ((x1 + x2) / 2, (y1 + y2) / 2, x2 - x1, y2 - y1)]
        self.at = self.seen_at = now
        self.score, self.hits = row[4], 1
        self.measurement = list(row[:4])
        self.measured = True
        self.ambiguous = False

    @property
    def box(self):
        x, y, width, height = (axis.position for axis in self.axes)
        return [x - width / 2, y - height / 2, x + width / 2, y + height / 2]

    def predict(self, now):
        dt = max(0., now - self.at)
        for index, axis in enumerate(self.axes):
            axis.predict(dt, .8 if index < 2 else .35)
        self.at, self.measured, self.ambiguous = now, False, False

    def correct(self, row, now):
        x1, y1, x2, y2 = row[:4]
        for axis, value in zip(self.axes, ((x1 + x2) / 2, (y1 + y2) / 2, x2 - x1, y2 - y1)):
            axis.correct(value, .02 ** 2 / max(.2, row[4]))
        self.seen_at, self.score, self.measurement = now, row[4], list(row[:4])
        self.measured = True
        self.hits += 1

    def cost(self, row, weak):
        box = self.box
        overlap = box_iou(box, row)
        dx = (row[0] + row[2]) / 2 - self.axes[0].position
        dy = (row[1] + row[3]) / 2 - self.axes[1].position
        innovation = dx * dx / (self.axes[0].pp + .0004) + dy * dy / (self.axes[1].pp + .0004)
        width, height = row[2] - row[0], row[3] - row[1]
        if (overlap < (.4 if weak else .15) or innovation > 16
                or not .4 < width / max(.001, box[2] - box[0]) < 2.5
                or not .4 < height / max(.001, box[3] - box[1]) < 2.5):
            return 1000.
        return 1 - overlap if weak else .8 * (1 - overlap) + .2 * (1 - row[4])

    def snapshot(self):
        return dict(id=self.id, box=self.box, measurement=self.measurement, score=self.score,
                    velocity=[axis.velocity for axis in self.axes], seen_at=self.seen_at,
                    measured=self.measured, confirmed=self.hits >= 2, ambiguous=self.ambiguous)


class PersonTracker:
    def __init__(self, high_threshold=.35, low_threshold=.1, new_threshold=.5,
                 retention=.75, max_tracks=32):
        if not 0 < low_threshold < high_threshold <= new_threshold <= 1:
            raise ValueError('Tracking thresholds must satisfy 0 < low < high <= new <= 1')
        self.high, self.low, self.new = high_threshold, low_threshold, new_threshold
        self.retention, self.max_tracks = retention, max_tracks
        self.tracks = []
        self.next_id = 1
        self.at = None

    def reset(self):
        self.tracks.clear()
        self.at = None
        # IDs are never recycled inside the same detector process.

    def _match(self, tracks, detections, now, weak=False):
        if not tracks or not detections:
            return set(), set()
        costs = [[track.cost(row, weak) for row in detections] for track in tracks]
        matched_tracks, matched_detections = set(), set()
        for i, j in assignment([row + [.85] * len(tracks) for row in costs]):
            if j >= len(detections) or costs[i][j] >= .85:
                continue
            plausible = sorted(cost for cost in costs[i] if cost < .85)
            alternatives = sorted(row[j] for row in costs if row[j] < .85)
            tracks[i].ambiguous = ((len(plausible) > 1 and plausible[1] - plausible[0] < .12)
                                   or (len(alternatives) > 1 and alternatives[1] - alternatives[0] < .12))
            tracks[i].correct(detections[j], now)
            matched_tracks.add(tracks[i].id)
            matched_detections.add(j)
        return matched_tracks, matched_detections

    def update(self, rows, now):
        if not math.isfinite(now) or (self.at is not None and now <= self.at):
            return []
        rows = [row for row in rows if len(row) == 6 and all(math.isfinite(v) for v in row)
                and 0 <= row[0] < row[2] <= 1 and 0 <= row[1] < row[3] <= 1
                and self.low <= row[4] <= 1]
        rows = sorted(rows, key=lambda row: row[4], reverse=True)[:64]
        self.tracks = [track for track in self.tracks if now - track.seen_at <= self.retention]
        previously_measured = {track.id for track in self.tracks if track.measured}
        for track in self.tracks:
            track.predict(now)
        high = [row for row in rows if row[4] >= self.high]
        low = [row for row in rows if row[4] < self.high]
        matched, used = self._match(self.tracks, high, now)
        # Weak boxes continue active tracks; they cannot resurrect a lost identity.
        remaining = [track for track in self.tracks if track.id not in matched and track.id in previously_measured]
        self._match(remaining, low, now, weak=True)
        for index, row in enumerate(high):
            if index not in used and row[4] >= self.new and len(self.tracks) < self.max_tracks:
                self.tracks.append(Track(self.next_id, row, now))
                self.next_id += 1
        measured = [track for track in self.tracks if track.measured]
        for index, a in enumerate(measured):
            for b in measured[index + 1:]:
                if box_iou(a.measurement, b.measurement) > .35:
                    a.ambiguous = b.ambiguous = True
        self.at = now
        return [track.snapshot() for track in self.tracks]
