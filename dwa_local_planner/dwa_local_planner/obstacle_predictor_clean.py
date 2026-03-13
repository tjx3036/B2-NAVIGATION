#!/usr/bin/env python3
"""
Obstacle predictor (clean single implementation).

Pipeline:
- cluster points to obstacle centroids
- CV Kalman tracking
- Hungarian data association with distance + Mahalanobis gating
- dynamic/static state machine with hysteresis and static confirm time
"""

import math
import time
from typing import List, Optional, Tuple

import numpy as np


class ObstacleTrack:
    """Single tracked obstacle with state [x, y, vx, vy]."""

    def __init__(
        self,
        x: float,
        y: float,
        track_id: int,
        process_noise: float = 1.2,
        measurement_noise: float = 0.12,
        max_speed: float = 3.0,
    ):
        self.track_id = int(track_id)
        self.state = np.array([float(x), float(y), 0.0, 0.0], dtype=np.float64)
        self.P = np.diag([0.25, 0.25, 1.0, 1.0]).astype(np.float64)
        self.process_noise = float(max(0.05, process_noise))
        self.measurement_noise = float(max(0.01, measurement_noise))
        self.max_speed = float(max(0.2, max_speed))

        self.last_update_t = 0.0
        self.hit_count = 1
        self.age = 0.0

        self.is_dynamic = True
        self.dynamic_until = 0.0
        self.low_speed_since = 0.0

        self.prev_x: float = float(x)
        self.prev_y: float = float(y)

    @property
    def x(self) -> float:
        return float(self.state[0])

    @property
    def y(self) -> float:
        return float(self.state[1])

    @property
    def vx(self) -> float:
        return float(self.state[2])

    @property
    def vy(self) -> float:
        return float(self.state[3])

    def _clamp_speed(self) -> None:
        vx = float(self.state[2])
        vy = float(self.state[3])
        s = math.hypot(vx, vy)
        if s > self.max_speed and s > 1e-9:
            k = self.max_speed / s
            self.state[2] = vx * k
            self.state[3] = vy * k

    def _build_F_Q(self, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        dt = float(max(1e-4, dt))
        F = np.array(
            [
                [1.0, 0.0, dt, 0.0],
                [0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        a2 = self.process_noise * self.process_noise
        q11 = 0.25 * dt * dt * dt * dt * a2
        q13 = 0.5 * dt * dt * dt * a2
        q33 = dt * dt * a2
        Q = np.array(
            [
                [q11, 0.0, q13, 0.0],
                [0.0, q11, 0.0, q13],
                [q13, 0.0, q33, 0.0],
                [0.0, q13, 0.0, q33],
            ],
            dtype=np.float64,
        )
        return F, Q

    def predict_to(self, t: float) -> Tuple[float, float]:
        if self.last_update_t <= 0.0:
            return self.x, self.y
        dt = float(np.clip(t - self.last_update_t, 0.0, 0.7))
        if dt <= 1e-8:
            return self.x, self.y
        F, _ = self._build_F_Q(dt)
        s = F @ self.state
        return float(s[0]), float(s[1])

    def _predict_inplace(self, dt: float) -> None:
        dt = float(np.clip(dt, 0.0, 0.7))
        if dt <= 1e-8:
            return
        F, Q = self._build_F_Q(dt)
        self.state = F @ self.state
        self.P = F @ self.P @ F.T + Q
        self._clamp_speed()

    def mahalanobis_sq(self, x_meas: float, y_meas: float, t: float) -> float:
        if self.last_update_t <= 0.0:
            dx = float(x_meas) - self.x
            dy = float(y_meas) - self.y
            return dx * dx + dy * dy
        dt = float(np.clip(t - self.last_update_t, 0.0, 0.7))
        F, Q = self._build_F_Q(dt)
        x_pred = F @ self.state
        P_pred = F @ self.P @ F.T + Q
        H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]], dtype=np.float64)
        R = np.diag([self.measurement_noise * self.measurement_noise] * 2).astype(np.float64)
        z = np.array([float(x_meas), float(y_meas)], dtype=np.float64)
        y = z - (H @ x_pred)
        S = H @ P_pred @ H.T + R
        try:
            Sinv = np.linalg.inv(S)
            return float(y.T @ Sinv @ y)
        except np.linalg.LinAlgError:
            return float("inf")

    def update(self, x_meas: float, y_meas: float, t: float) -> None:
        if self.last_update_t <= 0.0:
            self.last_update_t = float(t)
            self.state[0] = float(x_meas)
            self.state[1] = float(y_meas)
            self.prev_x = float(x_meas)
            self.prev_y = float(y_meas)
            return

        self.prev_x = float(self.state[0])
        self.prev_y = float(self.state[1])
        dt = float(np.clip(t - self.last_update_t, 0.02, 0.50))
        self._predict_inplace(dt)

        H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]], dtype=np.float64)
        R = np.diag([self.measurement_noise * self.measurement_noise] * 2).astype(np.float64)
        z = np.array([float(x_meas), float(y_meas)], dtype=np.float64)
        y = z - (H @ self.state)
        S = H @ self.P @ H.T + R
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
            I = np.eye(4, dtype=np.float64)
            self.state = self.state + K @ y
            IKH = I - K @ H
            self.P = IKH @ self.P @ IKH.T + K @ R @ K.T
        except np.linalg.LinAlgError:
            self.state[0] = 0.65 * self.state[0] + 0.35 * z[0]
            self.state[1] = 0.65 * self.state[1] + 0.35 * z[1]

        self._clamp_speed()
        self.last_update_t = float(t)
        self.hit_count += 1

    def predict(self, dt: float) -> Tuple[float, float]:
        dt = float(max(0.0, dt))
        return self.x + self.vx * dt, self.y + self.vy * dt

    def predict_trajectory(self, dt_step: float, num_steps: int) -> List[Tuple[float, float]]:
        return [self.predict((i + 1) * dt_step) for i in range(num_steps)]


def _cluster_points(points: np.ndarray, eps: float, min_samples: int = 2) -> List[np.ndarray]:
    if points is None or points.shape[0] == 0:
        return []
    n = points.shape[0]
    if n == 1:
        return [points]
    visited = np.zeros(n, dtype=bool)
    clusters: List[np.ndarray] = []
    eps2 = float(eps * eps)
    for i in range(n):
        if visited[i]:
            continue
        visited[i] = True
        idxs = [i]
        q = [i]
        while q:
            qi = q.pop()
            px, py = points[qi, 0], points[qi, 1]
            for j in range(n):
                if visited[j]:
                    continue
                dx = points[j, 0] - px
                dy = points[j, 1] - py
                if dx * dx + dy * dy < eps2:
                    visited[j] = True
                    idxs.append(j)
                    q.append(j)
        if len(idxs) >= max(1, int(min_samples)):
            clusters.append(points[idxs])
    return clusters


def _centroid(cluster: np.ndarray) -> Tuple[float, float]:
    return float(np.mean(cluster[:, 0])), float(np.mean(cluster[:, 1]))


def _hungarian_square(cost: np.ndarray) -> List[Tuple[int, int]]:
    n = int(cost.shape[0])
    if n <= 0:
        return []
    u = np.zeros(n + 1, dtype=np.float64)
    v = np.zeros(n + 1, dtype=np.float64)
    p = np.zeros(n + 1, dtype=np.int64)
    way = np.zeros(n + 1, dtype=np.int64)
    for i in range(1, n + 1):
        p[0] = i
        j0 = 0
        minv = np.full(n + 1, np.inf, dtype=np.float64)
        used = np.zeros(n + 1, dtype=bool)
        while True:
            used[j0] = True
            i0 = int(p[j0])
            delta = np.inf
            j1 = 0
            for j in range(1, n + 1):
                if used[j]:
                    continue
                cur = cost[i0 - 1, j - 1] - u[i0] - v[j]
                if cur < minv[j]:
                    minv[j] = cur
                    way[j] = j0
                if minv[j] < delta:
                    delta = minv[j]
                    j1 = j
            for j in range(0, n + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minv[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        while True:
            j1 = int(way[j0])
            p[j0] = p[j1]
            j0 = j1
            if j0 == 0:
                break
    return [(int(p[j]) - 1, j - 1) for j in range(1, n + 1)]


def _assign_rectangular(cost: np.ndarray) -> List[Tuple[int, int]]:
    if cost.size == 0:
        return []
    r, c = int(cost.shape[0]), int(cost.shape[1])
    n = max(r, c)
    pad = float(np.max(cost) + 1e6)
    sq = np.full((n, n), pad, dtype=np.float64)
    sq[:r, :c] = cost
    pairs = _hungarian_square(sq)
    return [(i, j) for (i, j) in pairs if i < r and j < c]


class ObstaclePredictor:
    """Cluster -> track -> predict."""

    def __init__(
        self,
        cluster_eps: float = 0.18,
        min_cluster_points: int = 1,
        max_track_age: float = 1.0,
        min_hits_to_predict: int = 1,
        min_speed_to_predict: float = 0.08,
        dynamic_enter_speed: float = 0.30,
        dynamic_exit_speed: float = 0.10,
        dynamic_hold_time: float = 0.6,
        static_confirm_time: float = 0.5,
        association_dist: float = 0.5,
        predict_time: float = 0.7,
        predict_dt: float = 0.1,
        process_noise: float = 1.2,
        measurement_noise: float = 0.12,
        max_speed: float = 3.0,
    ):
        self.cluster_eps = float(cluster_eps)
        self.min_cluster_points = int(min_cluster_points)
        self.max_track_age = float(max_track_age)
        self.min_hits_to_predict = int(min_hits_to_predict)
        self.min_speed_to_predict = float(min_speed_to_predict)
        self.dynamic_enter_speed = max(float(dynamic_enter_speed), self.min_speed_to_predict)
        self.dynamic_exit_speed = min(float(dynamic_exit_speed), self.dynamic_enter_speed * 0.8)
        self.dynamic_hold_time = max(0.1, float(dynamic_hold_time))
        self.static_confirm_time = max(0.1, float(static_confirm_time))
        self.association_dist = float(association_dist)
        self.predict_time = float(predict_time)
        self.predict_dt = float(predict_dt)
        self.process_noise = float(process_noise)
        self.measurement_noise = float(measurement_noise)
        self.max_speed = float(max_speed)

        self._tracks: List[ObstacleTrack] = []
        self._next_id = 0
        self._last_t = 0.0
        self._max_association_mahalanobis_sq = 9.0

    def reset(self) -> None:
        self._tracks = []
        self._next_id = 0
        self._last_t = 0.0

    def update(self, obstacle_points: Optional[np.ndarray], timestamp: Optional[float] = None) -> None:
        t = float(timestamp if timestamp is not None else time.monotonic())
        if self._last_t > 0.0 and t <= self._last_t:
            t = self._last_t + 1e-3

        if obstacle_points is None or obstacle_points.shape[0] == 0:
            for tr in self._tracks:
                tr.age = t - tr.last_update_t
            self._prune_old_tracks(t)
            self._refresh_dynamic_states(t)
            self._last_t = t
            return

        arr = np.asarray(obstacle_points, dtype=np.float64)
        if arr.ndim != 2 or arr.shape[1] != 2:
            return

        clusters = _cluster_points(arr, self.cluster_eps, self.min_cluster_points)
        centroids = [_centroid(c) for c in clusters]
        for tr in self._tracks:
            tr.age = t - tr.last_update_t

        if not centroids:
            self._prune_old_tracks(t)
            self._refresh_dynamic_states(t)
            self._last_t = t
            return

        used = [False] * len(centroids)
        if self._tracks:
            gate2 = self.association_dist * self.association_dist
            huge = 1e9
            cost = np.full((len(self._tracks), len(centroids)), huge, dtype=np.float64)
            for ti, tr in enumerate(self._tracks):
                px, py = tr.predict_to(t)
                for ci, (cx, cy) in enumerate(centroids):
                    dx = cx - px
                    dy = cy - py
                    d2 = dx * dx + dy * dy
                    if d2 > gate2:
                        continue
                    md2 = tr.mahalanobis_sq(cx, cy, t)
                    if not math.isfinite(md2) or md2 > self._max_association_mahalanobis_sq:
                        continue
                    cost[ti, ci] = 0.35 * (d2 / max(gate2, 1e-6)) + 0.65 * (
                        md2 / self._max_association_mahalanobis_sq
                    )

            for ti, ci in _assign_rectangular(cost):
                if cost[ti, ci] >= huge * 0.5:
                    continue
                cx, cy = centroids[ci]
                self._tracks[ti].update(cx, cy, t)
                used[ci] = True

        for i, (cx, cy) in enumerate(centroids):
            if used[i]:
                continue
            tr = ObstacleTrack(
                cx, cy, self._next_id,
                self.process_noise, self.measurement_noise, self.max_speed,
            )
            tr.last_update_t = t
            self._tracks.append(tr)
            self._next_id += 1

        self._prune_old_tracks(t)
        self._refresh_dynamic_states(t)
        self._last_t = t

    def _prune_old_tracks(self, t: float) -> None:
        self._tracks = [tr for tr in self._tracks if (t - tr.last_update_t) <= self.max_track_age]

    def _refresh_dynamic_states(self, t: float) -> None:
        for tr in self._tracks:
            # 初期命中次数不足时，一律按“可能动态”处理，以免过早当静态抹掉
            if tr.hit_count < self.min_hits_to_predict:
                tr.is_dynamic = True
                continue

            # 速度估计带噪，先对小于阈值的抖动归零，避免小目标+激光抖动被误判为动态
            spd = math.hypot(tr.vx, tr.vy)
            if spd < 0.10:
                spd = 0.0

            # 进入动态：超过 dynamic_enter_speed，则视为动态，并保持至少 dynamic_hold_time
            if spd >= self.dynamic_enter_speed:
                tr.is_dynamic = True
                tr.dynamic_until = max(tr.dynamic_until, t + self.dynamic_hold_time)
                tr.low_speed_since = 0.0
                continue

            # 低速段：只有长期低于 dynamic_exit_speed，且超过 static_confirm_time，且已过 dynamic_until，才转为“静态”
            if spd <= self.dynamic_exit_speed:
                if tr.low_speed_since <= 0.0:
                    tr.low_speed_since = t
                if (t - tr.low_speed_since) >= self.static_confirm_time and t > tr.dynamic_until:
                    tr.is_dynamic = False
                else:
                    tr.is_dynamic = True
            else:
                # 中间速度：仍按动态处理，但不延长 low_speed_since 计时
                tr.is_dynamic = True
                tr.low_speed_since = 0.0

    def get_current_obstacle_points(self) -> np.ndarray:
        if not self._tracks:
            return np.empty((0, 2), dtype=np.float32)
        return np.array([[tr.x, tr.y] for tr in self._tracks], dtype=np.float32)

    def get_predicted_obstacle_points(
        self,
        predict_time: Optional[float] = None,
        dt_step: Optional[float] = None,
    ) -> np.ndarray:
        pt = float(predict_time if predict_time is not None else self.predict_time)
        dt = float(dt_step if dt_step is not None else self.predict_dt)
        n = max(1, int(pt / max(dt, 1e-3)))
        pts: List[List[float]] = []
        for tr in self._tracks:
            for px, py in tr.predict_trajectory(dt, n):
                pts.append([px, py])
        if not pts:
            return np.empty((0, 2), dtype=np.float32)
        return np.asarray(pts, dtype=np.float32)

    def get_obstacle_points_at_time(self, t_offset: float) -> np.ndarray:
        if not self._tracks:
            return np.empty((0, 2), dtype=np.float32)
        return np.asarray([tr.predict(float(t_offset)) for tr in self._tracks], dtype=np.float32)

    def get_combined_obstacle_points(
        self,
        predict_time: Optional[float] = None,
        dt_step: Optional[float] = None,
        include_current: bool = True,
    ) -> np.ndarray:
        curr = self.get_current_obstacle_points()
        pred = self.get_predicted_obstacle_points(predict_time, dt_step)
        if include_current and curr.shape[0] > 0:
            if pred.shape[0] > 0:
                return np.concatenate((curr, pred), axis=0)
            return curr
        return pred if pred.shape[0] > 0 else curr

    def get_track_count(self) -> int:
        return len(self._tracks)

    def get_dynamic_centroids(self, min_speed: Optional[float] = None) -> List[Tuple[float, float]]:
        out: List[Tuple[float, float]] = []
        for tr in self._tracks:
            if min_speed is None:
                keep = tr.is_dynamic
            else:
                if tr.hit_count < self.min_hits_to_predict:
                    continue
                keep = math.hypot(tr.vx, tr.vy) >= float(min_speed)
            if keep:
                out.append((tr.x, tr.y))
        return out

    def get_dynamic_trajectory_bands(
        self,
        min_speed: Optional[float] = None,
        predict_time: Optional[float] = None,
        dt_step: Optional[float] = None,
    ) -> List[List[Tuple[float, float]]]:
        pt = float(predict_time if predict_time is not None else self.predict_time)
        dt = float(dt_step if dt_step is not None else self.predict_dt)
        n = max(1, int(pt / max(dt, 1e-3)))
        out: List[List[Tuple[float, float]]] = []
        for tr in self._tracks:
            if min_speed is None:
                keep = tr.is_dynamic
            else:
                if tr.hit_count < self.min_hits_to_predict:
                    continue
                keep = math.hypot(tr.vx, tr.vy) >= float(min_speed)
            if not keep:
                continue
            band: List[Tuple[float, float]] = []
            if tr.hit_count >= 2 and (tr.prev_x != tr.x or tr.prev_y != tr.y):
                band.append((tr.prev_x, tr.prev_y))
            band.append((tr.x, tr.y))
            for i in range(n):
                band.append(tr.predict((i + 1) * dt))
            out.append(band)
        return out

    def get_confirmed_dynamic_for_costmap_filter(
        self,
    ) -> Tuple[List[Tuple[float, float]], List[List[Tuple[float, float]]], List[Tuple[float, float]]]:
        """
        For costmap filtering: return confirmed dynamic centroids, trajectory bands, and trail behind.
        Returns (centroids, trajectory_bands, trail_behind).
        """
        centroids = self.get_dynamic_centroids()
        bands = self.get_dynamic_trajectory_bands()
        trail: List[Tuple[float, float]] = []
        for tr in self._tracks:
            if not tr.is_dynamic or tr.hit_count < 2:
                continue
            px, py = tr.prev_x, tr.prev_y
            cx, cy = tr.x, tr.y
            dx, dy = cx - px, cy - py
            d = math.hypot(dx, dy)
            if d < 1e-6:
                trail.append((cx, cy))
            else:
                n_step = max(1, int(d / 0.06))
                for i in range(n_step + 1):
                    u = i / n_step
                    trail.append((px + u * dx, py + u * dy))
        return centroids, bands, trail

    def get_protect_centroids(self) -> List[Tuple[float, float]]:
        """Return centroids to protect from filtering (static + candidate static)."""
        out: List[Tuple[float, float]] = []
        for tr in self._tracks:
            if not tr.is_dynamic:
                out.append((tr.x, tr.y))
            elif tr.hit_count >= 2:
                spd = math.hypot(tr.vx, tr.vy)
                if spd < 0.03:
                    out.append((tr.x, tr.y))
        return out

    def get_predictions(
        self,
        predict_time: Optional[float] = None,
        dt_step: Optional[float] = None,
        range_origin_xy: Optional[Tuple[float, float]] = None,
        max_range: float = 4.0,
        max_preds: int = 30,
    ) -> List[dict]:
        pt = float(predict_time if predict_time is not None else self.predict_time)
        dt = float(dt_step if dt_step is not None else self.predict_dt)
        n = max(1, int(pt / max(dt, 1e-3)))
        preds: List[dict] = []
        for tr in self._tracks:
            if tr.hit_count < self.min_hits_to_predict:
                continue
            if not tr.is_dynamic:
                continue
            speed = math.hypot(tr.vx, tr.vy)
            if speed < self.min_speed_to_predict:
                continue
            if range_origin_xy is not None:
                dx = tr.x - float(range_origin_xy[0])
                dy = tr.y - float(range_origin_xy[1])
                if dx * dx + dy * dy > float(max_range * max_range):
                    continue
            future = [tr.predict((i + 1) * dt) for i in range(n)]
            preds.append(
                {
                    "position_xy": (tr.x, tr.y),
                    "velocity_xy": (tr.vx, tr.vy),
                    "future_xy": future,
                }
            )
        if range_origin_xy is not None and len(preds) > int(max_preds):
            ox, oy = float(range_origin_xy[0]), float(range_origin_xy[1])
            preds.sort(key=lambda p: (p["position_xy"][0] - ox) ** 2 + (p["position_xy"][1] - oy) ** 2)
            preds = preds[: int(max_preds)]
        return preds

    def get_tracks_debug(self) -> List[dict]:
        out: List[dict] = []
        for tr in self._tracks:
            speed = math.hypot(tr.vx, tr.vy)
            out.append(
                {
                    "id": int(tr.track_id),
                    "x": float(tr.x),
                    "y": float(tr.y),
                    "vx": float(tr.vx),
                    "vy": float(tr.vy),
                    "speed": float(speed),
                    "is_dynamic": bool(tr.is_dynamic),
                    "dynamic_until": float(max(0.0, tr.dynamic_until - self._last_t)),
                    "hits": int(tr.hit_count),
                    "age": float(tr.age),
                }
            )
        return out
