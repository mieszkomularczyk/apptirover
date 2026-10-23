#!/usr/bin/env python3
"""
Bounding-box estimator for the target tracker.

Two constant-velocity Kalman filters run in parallel:
  - the bbox center  [cx, cy] (+ velocity), for horizontal/vertical control
  - the bbox size    [sx, sy] (+ rate),     for the area/throttle control loop

Both smooth noisy YOLO detections and coast their prediction forward through
short dropouts so downstream control does not run on frozen data. Outlier gating
is driven by the center (position is the reliable "same target?" discriminator);
when a measurement is accepted or the track is reset, both filters are updated
together so their states stay consistent.

Kept ROS-agnostic (plain floats in/out) so it can be unit-tested in isolation.
"""

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


@dataclass
class BBoxEstimate:
    """Filtered bbox estimate. Velocities are per second, in normalized units."""
    center_x: float
    center_y: float
    size_x: float
    size_y: float
    vel_x: float
    vel_y: float

    @property
    def area(self) -> float:
        return self.size_x * self.size_y


class _CVKalman2D:
    """Constant-velocity Kalman filter over a 2D quantity: state [px, py, vpx, vpy].

    Generic building block: used once for the bbox center and once for the bbox
    size. predict() and correct() are split so the owner can gate a measurement
    (after predicting) before deciding whether to correct.
    """

    def __init__(self, process_std: float, meas_std: float):
        self.configure(process_std, meas_std)
        self._x = np.zeros(4)
        self._P = np.eye(4)

    def configure(self, process_std: float, meas_std: float):
        self.process_std = process_std
        self.meas_std = meas_std

    def seed(self, px: float, py: float):
        """Initialise on a fresh measurement: trust position, leave rate open."""
        self._x = np.array([px, py, 0.0, 0.0])
        self._P = np.diag([self.meas_std ** 2, self.meas_std ** 2, 1.0, 1.0])

    def predict(self, dt: float) -> Tuple[float, float]:
        """Propagate state/covariance forward by dt; return the predicted value."""
        F = np.array([
            [1.0, 0.0, dt,  0.0],
            [0.0, 1.0, 0.0, dt ],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])
        # Continuous white-noise acceleration model.
        q = self.process_std ** 2
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt3 * dt
        Q = q * np.array([
            [dt4 / 4.0, 0.0,       dt3 / 2.0, 0.0      ],
            [0.0,       dt4 / 4.0, 0.0,       dt3 / 2.0],
            [dt3 / 2.0, 0.0,       dt2,       0.0      ],
            [0.0,       dt3 / 2.0, 0.0,       dt2      ],
        ])
        self._x = F @ self._x
        self._P = F @ self._P @ F.T + Q
        return float(self._x[0]), float(self._x[1])

    def correct(self, mx: float, my: float):
        """Fuse a measurement into the current (predicted) state."""
        H = np.array([[1.0, 0.0, 0.0, 0.0],
                      [0.0, 1.0, 0.0, 0.0]])
        R = np.diag([self.meas_std ** 2, self.meas_std ** 2])
        z = np.array([mx, my])
        y = z - H @ self._x
        S = H @ self._P @ H.T + R
        K = self._P @ H.T @ np.linalg.inv(S)
        self._x = self._x + K @ y
        self._P = (np.eye(4) - K @ H) @ self._P

    @property
    def value(self) -> Tuple[float, float]:
        return float(self._x[0]), float(self._x[1])

    @property
    def velocity(self) -> Tuple[float, float]:
        return float(self._x[2]), float(self._x[3])

    def peek(self, dt: float) -> Tuple[float, float]:
        """Position extrapolated by dt without mutating state (read-only)."""
        return float(self._x[0] + self._x[2] * dt), float(self._x[1] + self._x[3] * dt)


class BBoxEstimator:
    """Parallel constant-velocity Kalman filters on bbox center and size."""

    def __init__(self, process_std: float = 1.0, meas_std: float = 0.02,
                 size_process_std: float = 0.5, size_meas_std: float = 0.02,
                 gate_threshold: float = 0.2, max_reject_count: int = 5):
        self._center = _CVKalman2D(process_std, meas_std)
        self._size = _CVKalman2D(size_process_std, size_meas_std)
        self.configure(process_std, meas_std, size_process_std, size_meas_std,
                       gate_threshold, max_reject_count)
        self.reset()

    def configure(self, process_std: float, meas_std: float,
                  size_process_std: float, size_meas_std: float,
                  gate_threshold: float, max_reject_count: int):
        """Update the tuning gains (safe to call every cycle for live params).

        *_process_std are acceleration/rate noise (normalized units/s^2) driving
        the process covariance; *_meas_std are the detection noise (normalized
        units) used as the measurement covariance.
        """
        self._center.configure(process_std, meas_std)
        self._size.configure(size_process_std, size_meas_std)
        self.gate_threshold = gate_threshold
        self.max_reject_count = max_reject_count

    def reset(self):
        """Clear state so a fresh acquisition starts clean."""
        self._initialized = False
        self._last_time: Optional[float] = None
        self._reject_count = 0

    @property
    def initialized(self) -> bool:
        return self._initialized

    def _estimate(self) -> BBoxEstimate:
        cx, cy = self._center.value
        sx, sy = self._size.value
        vx, vy = self._center.velocity
        # Size cannot be negative; a long coast on a shrinking box could predict
        # past zero, which would poison the area signal downstream.
        return BBoxEstimate(cx, cy, max(0.0, sx), max(0.0, sy), vx, vy)

    def _seed(self, cx: float, cy: float, sx: float, sy: float, now: float):
        self._center.seed(cx, cy)
        self._size.seed(sx, sy)
        self._initialized = True
        self._last_time = now
        self._reject_count = 0

    def update(self, cx: float, cy: float, sx: float, sy: float, now: float) -> BBoxEstimate:
        """Correct the estimate with a new measurement (with outlier gating)."""
        if not self._initialized:
            self._seed(cx, cy, sx, sy, now)
            return self._estimate()

        dt = max(0.0, now - self._last_time) if self._last_time is not None else 0.0
        pred_cx, pred_cy = self._center.predict(dt)
        self._size.predict(dt)

        # Outlier gating on the center: reject an implausible jump and coast (both
        # filters already predicted), up to a limit.
        dist = math.hypot(cx - pred_cx, cy - pred_cy)
        if dist > self.gate_threshold:
            if self._reject_count < self.max_reject_count:
                self._reject_count += 1
                self._last_time = now
                return self._estimate()
            # Persistent divergence: target likely jumped/reacquired -> hard reset
            self._seed(cx, cy, sx, sy, now)
            return self._estimate()

        # Accept: fuse the measurement into both filters.
        self._reject_count = 0
        self._center.correct(cx, cy)
        self._size.correct(sx, sy)
        self._last_time = now
        return self._estimate()

    def peek_center(self, now: float) -> Optional[Tuple[float, float]]:
        """Predicted center position at `now`, without mutating filter state.

        For a caller that needs "where is the target right now" ahead of the
        update()/coast() call that will actually advance the filter this cycle.
        """
        if not self._initialized:
            return None
        dt = max(0.0, now - self._last_time) if self._last_time is not None else 0.0
        return self._center.peek(dt)

    def coast(self, now: float) -> Optional[BBoxEstimate]:
        """Predict forward through a dropout.

        Returns None if uninitialized.
        """
        if not self._initialized:
            return None
        dt = max(0.0, now - self._last_time) if self._last_time is not None else 0.0
        self._center.predict(dt)
        self._size.predict(dt)
        self._last_time = now
        return self._estimate()
