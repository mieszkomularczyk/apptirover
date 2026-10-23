#!/usr/bin/env python3
"""
Track-ID selection logic for the target tracker.

Chooses which ByteTrack persistent ID to follow out of a frame's detections:
while no target is locked, it accumulates per-ID (count, score) stats over a
rolling window so the strongest track can be picked once the caller confirms
a target is present. Once locked, it keeps following that ID and falls back
to the same-frame detection nearest the caller-supplied reference position
(the filtered/predicted position, not a raw last measurement) when the
tracker drops or renumbers the track, so a one-frame ID churn does not look
like the target vanishing.

Kept ROS-agnostic (plain values in/out) so it can be unit-tested in isolation.
"""

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple


@dataclass
class Candidate:
    """A single detection, normalized to [0, 1] image coordinates."""
    id: str
    score: float
    center_x: float
    center_y: float
    size_x: float
    size_y: float


class TargetSelector:
    """Picks and holds a track ID across frames of `Candidate` detections."""

    def __init__(self, candidate_window_sec: float, max_reacquire_distance: float):
        self.configure(candidate_window_sec, max_reacquire_distance)
        self.reset()

    def configure(self, candidate_window_sec: float, max_reacquire_distance: float):
        self.candidate_window_sec = candidate_window_sec
        self.max_reacquire_distance = max_reacquire_distance

    def reset(self):
        """Clear the lock and all accumulated candidate stats."""
        self.current_id: Optional[str] = None
        self._candidate_history: Dict[str, List[float]] = {}
        self._last_seen: Dict[str, float] = {}

    def select(
        self,
        candidates: List[Candidate],
        now: float,
        reference_center: Optional[Tuple[float, float]] = None,
    ) -> Optional[Candidate]:
        """Return this frame's chosen detection, or None if nothing qualifies.

        With no lock held, this also records per-ID stats for `lock_best_candidate`
        and returns the best-scoring detection regardless of ID (mirrors plain
        best-score selection until a target is confirmed present).

        `reference_center` is the caller's filtered/predicted position for "now"
        (e.g. a Kalman estimate), used only for churn reacquisition below -- a
        raw last-measurement position would lag behind a moving target and
        under-gate valid replacements or over-gate stale ones.
        """
        if self.current_id is None:
            self._record_candidates(candidates, now)
            return max(candidates, key=lambda c: c.score) if candidates else None

        for candidate in candidates:
            if candidate.id == self.current_id:
                return candidate

        replacement = self._find_nearby(candidates, reference_center)
        if replacement is not None:
            self.current_id = replacement.id
        return replacement

    def lock_best_candidate(self) -> Optional[str]:
        """Commit to the strongest accumulated candidate (most detections,
        tie-broken by score) and clear the accumulation. Called once the
        caller's own visibility hysteresis confirms a target is present.

        Returns the chosen ID, or None if no ID-bearing detections were seen
        (e.g. the tracker hasn't confirmed a track yet) -- the caller then
        keeps behaving as plain best-score-per-frame selection.
        """
        if not self._candidate_history:
            return None

        def metric(item):
            _, samples = item
            count = len(samples)
            avg_score = sum(samples) / count
            return (count, avg_score)

        best_id, _ = max(self._candidate_history.items(), key=metric)
        self.current_id = best_id
        self._candidate_history.clear()
        return best_id

    def _record_candidates(self, candidates: List[Candidate], now: float):
        for candidate in candidates:
            if not candidate.id:
                continue
            self._candidate_history.setdefault(candidate.id, []).append(candidate.score)
            self._last_seen[candidate.id] = now

        # Bound memory by dropping stats for IDs not seen within the window.
        cutoff = now - self.candidate_window_sec
        stale = [id_ for id_, t in self._last_seen.items() if t < cutoff]
        for id_ in stale:
            self._last_seen.pop(id_, None)
            self._candidate_history.pop(id_, None)

    def _find_nearby(
        self,
        candidates: List[Candidate],
        reference_center: Optional[Tuple[float, float]],
    ) -> Optional[Candidate]:
        if reference_center is None or not candidates:
            return None
        ref_x, ref_y = reference_center
        best = None
        best_dist = self.max_reacquire_distance
        for candidate in candidates:
            dist = math.hypot(candidate.center_x - ref_x, candidate.center_y - ref_y)
            if dist <= best_dist:
                best = candidate
                best_dist = dist
        return best
