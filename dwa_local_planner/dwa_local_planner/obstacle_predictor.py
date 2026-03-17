#!/usr/bin/env python3
"""
Stable obstacle predictor for mobile robot / quadruped navigation.

目标：为 DWA 等局部规划提供 **稳定的动态障碍预测**，优先保证：
- 不乱闪、不乱跳；
- 少“凭空出现”的假障碍；
- 速度估计平滑可用；
- 接口与原来的 `ObstaclePredictor` 兼容：
  - __init__(...)
  - update(obstacle_points, timestamp=None)
  - get_dynamic_centroids(...)
  - get_dynamic_trajectory_bands(...)
  - get_predictions(...)
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np


# ==================== 聚类相关 ====================


def _cluster_points(points: np.ndarray, eps: float, min_samples: int = 3) -> List[np.ndarray]:
    """简单半径聚类：将近邻点聚成一簇即可，不追求严格 DBSCAN。"""
    if points is None or points.shape[0] == 0:
        return []
    n = int(points.shape[0])
    if n == 1:
        return [points]

    visited = np.zeros(n, dtype=bool)
    clusters: List[np.ndarray] = []
    r2 = float(eps * eps)

    for i in range(n):
        if visited[i]:
            continue
        visited[i] = True
        idxs = [i]
        queue = [i]
        while queue:
            qi = queue.pop()
            px = float(points[qi, 0])
            py = float(points[qi, 1])
            dx = points[:, 0] - px
            dy = points[:, 1] - py
            d2 = dx * dx + dy * dy
            neigh = np.where((~visited) & (d2 <= r2))[0]
            if neigh.size > 0:
                visited[neigh] = True
                idxs.extend(neigh.tolist())
                queue.extend(neigh.tolist())
        if len(idxs) >= int(min_samples):
            clusters.append(points[idxs])
    return clusters


def _centroid(cluster: np.ndarray) -> Tuple[float, float]:
    """簇质心。"""
    return float(np.mean(cluster[:, 0])), float(np.mean(cluster[:, 1]))


# ==================== 轨迹类型 ====================


def _velocity_from_position_history(
    history: List[Tuple[float, float, float]],
    min_frames: int,
    smooth_frames: int = 10,
) -> Tuple[float, float]:
    """
    用多帧 (t, x, y) 做线性回归得到 vx, vy；用最近 smooth_frames 帧，至少 min_frames 帧。
    """
    if len(history) < min_frames:
        return 0.0, 0.0
    use = history[-min(smooth_frames, len(history)):]
    if len(use) < min_frames:
        return 0.0, 0.0
    t = np.array([u[0] for u in use], dtype=np.float64)
    x = np.array([u[1] for u in use], dtype=np.float64)
    y = np.array([u[2] for u in use], dtype=np.float64)
    mt, mx, my = t.mean(), x.mean(), y.mean()
    tt = t - mt
    denom = np.dot(tt, tt) + 1e-12
    vx = float(np.dot(tt, x - mx) / denom)
    vy = float(np.dot(tt, y - my) / denom)
    return vx, vy


@dataclass
class ObstacleTrack:
    """单个障碍物轨迹：CV 状态 + 协方差 + 生命周期；速度由多帧位置回归得到。"""

    track_id: int
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0

    last_update_t: float = 0.0
    age: float = 0.0
    hit_count: int = 0
    missed_count: int = 0
    confirmed: bool = False

    # 多帧位置历史 (t, x, y)，用于回归计算速度
    position_history: List[Tuple[float, float, float]] = field(default_factory=list)

    # 输出用平滑位置，减少对外暴露的预测位置跳变（内部关联仍用 x, y）
    px_out: float = 0.0
    py_out: float = 0.0

    # 输出用平滑速度，减少对外暴露的速度抖动
    vx_out: float = 0.0
    vy_out: float = 0.0

    # EMA / 动态判定相关
    speed_ema: float = 0.0
    dyn_count: int = 0

    # 4x4 状态协方差
    P: np.ndarray = field(
        default_factory=lambda: np.diag([0.3, 0.3, 1.0, 1.0]).astype(np.float32)
    )

    def state_vector(self) -> np.ndarray:
        return np.array([[self.x], [self.y], [self.vx], [self.vy]], dtype=np.float32)

    def set_state_vector(self, vec: np.ndarray) -> None:
        self.x = float(vec[0, 0])
        self.y = float(vec[1, 0])
        self.vx = float(vec[2, 0])
        self.vy = float(vec[3, 0])

    # ---------- 预测 ----------

    def predict(self, dt: float, q_pos: float, q_vel: float, max_acc: float) -> None:
        """CV 预测一步：x <- F x, P <- F P F^T + Q，并限制速度突变。"""
        if dt <= 0.0:
            return
        dt = float(max(0.03, min(dt, 0.5)))

        F = np.array(
            [
                [1.0, 0.0, dt, 0.0],
                [0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )
        Q = np.diag(
            [
                q_pos * dt * dt,
                q_pos * dt * dt,
                q_vel * dt,
                q_vel * dt,
            ]
        ).astype(np.float32)

        x_vec = self.state_vector()
        x_pred = F @ x_vec

        # 限制速度变化（防止单步发散）
        dvx = float(x_pred[2, 0] - self.vx)
        dvy = float(x_pred[3, 0] - self.vy)
        dv = math.hypot(dvx, dvy)
        dv_max = float(max_acc * dt)
        if dv > dv_max > 0.0:
            scale = dv_max / dv
            x_pred[2, 0] = float(self.vx + dvx * scale)
            x_pred[3, 0] = float(self.vy + dvy * scale)

        P_pred = F @ self.P @ F.T + Q

        self.set_state_vector(x_pred)
        self.P = P_pred
        self.age += dt

        # 漏检时对速度做衰减，避免转弯/减速时预测跑太远导致下一帧关联不上
        if self.missed_count > 0:
            decay = 0.88
            self.vx *= decay
            self.vy *= decay

    # ---------- 门控与更新 ----------

    def maha_distance2(self, z: Tuple[float, float], meas_var: float) -> float:
        """当前位置观测 z 相对当前状态的 Mahalanobis 距离平方。"""
        zx, zy = float(z[0]), float(z[1])
        H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]], dtype=np.float32)
        R = np.diag([meas_var, meas_var]).astype(np.float32)

        x_vec = self.state_vector()
        z_pred = H @ x_vec
        y = np.array([[zx], [zy]], dtype=np.float32) - z_pred
        S = H @ self.P @ H.T + R
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return float("inf")
        d2 = float((y.T @ S_inv @ y)[0, 0])
        return d2

    def update(
        self,
        z: Tuple[float, float],
        meas_var: float,
        speed_ema_alpha: float,
        now_t: float,
        vel_ema_alpha: float,
        velocity_smooth_frames: int = 8,
        velocity_min_frames: int = 4,
    ) -> None:
        """Kalman 更新 + 多帧位置回归速度与 EMA 混合，避免相邻两帧噪声。"""
        zx, zy = float(z[0]), float(z[1])
        H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]], dtype=np.float32)
        R = np.diag([meas_var, meas_var]).astype(np.float32)

        x_vec = self.state_vector()
        z_vec = np.array([[zx], [zy]], dtype=np.float32)
        y = z_vec - H @ x_vec
        S = H @ self.P @ H.T + R
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return

        x_new = x_vec + K @ y
        I = np.eye(4, dtype=np.float32)
        P_new = (I - K @ H) @ self.P

        prev_vx, prev_vy = float(self.vx), float(self.vy)
        self.set_state_vector(x_new)
        self.P = P_new

        # 位置输出 EMA，减少对外暴露的位置跳变（内部关联仍使用 self.x, self.y）
        pos_alpha = 0.35
        if self.hit_count == 0 and self.last_update_t == 0.0:
            # 第一次命中，直接对齐
            self.px_out = float(self.x)
            self.py_out = float(self.y)
        else:
            self.px_out = (1.0 - pos_alpha) * self.px_out + pos_alpha * float(self.x)
            self.py_out = (1.0 - pos_alpha) * self.py_out + pos_alpha * float(self.y)

        # 多帧位置历史：当前时刻 + 更新后位置，用于回归算速度
        self.position_history.append((now_t, float(self.x), float(self.y)))
        if velocity_smooth_frames > 0:
            while len(self.position_history) > velocity_smooth_frames:
                self.position_history.pop(0)
        vx_reg, vy_reg = _velocity_from_position_history(
            self.position_history, velocity_min_frames, velocity_smooth_frames
        )
        alpha_v = float(vel_ema_alpha)
        if len(self.position_history) >= velocity_min_frames:
            self.vx = (1.0 - alpha_v) * prev_vx + alpha_v * vx_reg
            self.vy = (1.0 - alpha_v) * prev_vy + alpha_v * vy_reg
        else:
            self.vx = (1.0 - alpha_v) * prev_vx + alpha_v * self.vx
            self.vy = (1.0 - alpha_v) * prev_vy + alpha_v * self.vy

        # 输出速度二次平滑，减轻对外暴露速度的抖动
        out_alpha = 0.28
        self.vx_out = (1.0 - out_alpha) * self.vx_out + out_alpha * self.vx
        self.vy_out = (1.0 - out_alpha) * self.vy_out + out_alpha * self.vy

        self.last_update_t = now_t
        self.hit_count += 1
        self.missed_count = 0

        inst_speed = math.hypot(self.vx, self.vy)
        alpha_s = float(speed_ema_alpha)
        self.speed_ema = (1.0 - alpha_s) * self.speed_ema + alpha_s * inst_speed


# ==================== 预测器主体 ====================


class ObstaclePredictor:
    """稳定版多目标 CV 预测器（偏移动机器人场景）。"""

    def __init__(
        self,
        # 聚类 / cluster 过滤
        cluster_eps: float = 0.3,
        min_cluster_points: int = 6,
        max_cluster_points: int = 400,
        # 轨迹生命周期
        max_track_age: float = 2.5,
        max_missed: int = 12,
        min_confirm_hits: int = 3,
        # 动态判定 / 预测
        min_hits_to_predict: int = 3,
        min_speed_to_predict: float = 0.35,
        dynamic_speed_threshold: float = 0.25,
        dynamic_required_count: int = 3,
        # 关联 / gating
        association_dist: float = 0.8,
        gate_mahalanobis: float = 3.0,
        meas_pos_var: float = 0.10,
        # 过程噪声 / 速度约束
        process_noise_pos: float = 0.25,
        process_noise_vel: float = 0.4,
        max_acc: float = 3.0,
        # EMA 系数
        speed_ema_alpha: float = 0.25,
        vel_ema_alpha: float = 0.5,
        # 预测参数
        predict_time: float = 2.5,
        predict_dt: float = 0.25,
        # 多帧速度：用最近 N 帧位置回归得到速度，避免相邻两帧噪声
        velocity_smooth_frames: int = 8,
        velocity_min_frames: int = 4,
    ):
        self.cluster_eps = float(cluster_eps)
        self.min_cluster_points = int(min_cluster_points)
        self.max_cluster_points = int(max_cluster_points)

        self.max_track_age = float(max_track_age)
        self.max_missed = int(max_missed)
        self.min_confirm_hits = int(min_confirm_hits)

        self.min_hits_to_predict = int(min_hits_to_predict)
        self.min_speed_to_predict = float(min_speed_to_predict)
        self.dynamic_speed_threshold = float(dynamic_speed_threshold)
        self.dynamic_required_count = int(dynamic_required_count)

        self.association_dist = float(association_dist)
        self.gate_mahalanobis = float(gate_mahalanobis)
        self.meas_pos_var = float(meas_pos_var)

        self.process_noise_pos = float(process_noise_pos)
        self.process_noise_vel = float(process_noise_vel)
        self.max_acc = float(max_acc)

        self.speed_ema_alpha = float(speed_ema_alpha)
        self.vel_ema_alpha = float(vel_ema_alpha)

        self.predict_time = float(predict_time)
        self.predict_dt = float(predict_dt)
        self.velocity_smooth_frames = int(velocity_smooth_frames)
        self.velocity_min_frames = int(velocity_min_frames)

        # 栅格量化：20cm 分辨率，吃掉厘米级抖动
        self._pos_grid: float = 0.2

        self._tracks: List[ObstacleTrack] = []
        self._next_id: int = 0

    # ---------- 轨迹管理 ----------

    def _prune_old_tracks(self, now_t: float) -> None:
        kept: List[ObstacleTrack] = []
        for tr in self._tracks:
            if tr.last_update_t <= 0.0:
                continue
            if now_t - tr.last_update_t > self.max_track_age:
                continue
            if tr.missed_count > self.max_missed:
                continue
            kept.append(tr)
        self._tracks = kept

    # ---------- 主更新接口 ----------

    def update(self, obstacle_points: Optional[np.ndarray], timestamp: Optional[float] = None) -> None:
        """用新一帧 2D 点云更新所有轨迹。"""
        t = float(timestamp if timestamp is not None else time.monotonic())

        if obstacle_points is None or obstacle_points.shape[0] == 0:
            # 只有预测 + missed_count++
            for tr in self._tracks:
                dt = t - tr.last_update_t if tr.last_update_t > 0.0 else 0.0
                if dt > 0.0:
                    tr.predict(dt, self.process_noise_pos, self.process_noise_vel, self.max_acc)
                    tr.missed_count += 1
            self._prune_old_tracks(t)
            return

        pts = np.asarray(obstacle_points, dtype=np.float32)
        if pts.ndim != 2 or pts.shape[1] != 2:
            return

        # ---------- 聚类 + cluster 过滤 ----------
        clusters = _cluster_points(pts, self.cluster_eps, self.min_cluster_points)
        centroids: List[Tuple[float, float]] = []
        for c in clusters:
            if c.shape[0] > self.max_cluster_points:
                continue
            cx, cy = _centroid(c)
            centroids.append((cx, cy))

        if centroids:
            centroids_arr = np.array(centroids, dtype=np.float32)
            # 位置栅格化：全部量化到 0.2m 网格，20cm 内抖动直接吞掉
            g = self._pos_grid
            if g > 0.0:
                centroids_arr = np.round(centroids_arr / g) * g
        else:
            centroids_arr = np.empty((0, 2), dtype=np.float32)

        # ---------- 先对所有轨迹预测到当前时刻 ----------
        for tr in self._tracks:
            dt = t - tr.last_update_t if tr.last_update_t > 0.0 else 0.0
            if dt > 0.0:
                tr.predict(dt, self.process_noise_pos, self.process_noise_vel, self.max_acc)

        n_tr = len(self._tracks)
        n_meas = int(centroids_arr.shape[0])

        # ---------- 构造代价矩阵 + 门控（简单贪心 GNN） ----------
        pairs: List[Tuple[int, int]] = []
        if n_tr > 0 and n_meas > 0:
            cost = np.full((n_tr, n_meas), np.inf, dtype=np.float32)
            gate_d2 = self.association_dist * self.association_dist
            maha_gate2 = self.gate_mahalanobis * self.gate_mahalanobis

            for ti, tr in enumerate(self._tracks):
                for mi in range(n_meas):
                    cx = float(centroids_arr[mi, 0])
                    cy = float(centroids_arr[mi, 1])
                    dx = cx - tr.x
                    dy = cy - tr.y
                    d2_eu = dx * dx + dy * dy
                    if d2_eu > gate_d2:
                        continue
                    d2_maha = tr.maha_distance2((cx, cy), self.meas_pos_var)
                    if d2_maha <= maha_gate2:
                        # 欧氏项避免相近时错配导致位置跳变，优先选离预测更近的观测
                        cost[ti, mi] = d2_maha + 0.03 * (d2_eu / (gate_d2 + 1e-6))

            flat_idx = np.argsort(cost, axis=None)
            used_tr = np.zeros(n_tr, dtype=bool)
            used_meas = np.zeros(n_meas, dtype=bool)
            for idx in flat_idx:
                c = float(cost.flat[idx])
                if not math.isfinite(c):
                    break
                ti = int(idx // n_meas)
                mi = int(idx % n_meas)
                if used_tr[ti] or used_meas[mi]:
                    continue
                used_tr[ti] = True
                used_meas[mi] = True
                pairs.append((ti, mi))
        else:
            used_tr = np.zeros(n_tr, dtype=bool)
            used_meas = np.zeros(n_meas, dtype=bool)

        # ---------- 对匹配的轨迹执行更新（用原始质心，保证速度回归不受 0.2m 量化影响）----------
        for ti, mi in pairs:
            tr = self._tracks[ti]
            cx, cy = float(centroids[mi][0]), float(centroids[mi][1])
            tr.update(
                (cx, cy),
                self.meas_pos_var,
                self.speed_ema_alpha,
                t,
                self.vel_ema_alpha,
                self.velocity_smooth_frames,
                self.velocity_min_frames,
            )
            if not tr.confirmed and tr.hit_count >= self.min_confirm_hits:
                tr.confirmed = True

            # 动态判定：加速计数，减速时缓慢衰减，避免转弯/减速瞬间从动态列表消失
            spd_eff = max(math.hypot(tr.vx, tr.vy), tr.speed_ema)
            if spd_eff > self.dynamic_speed_threshold:
                tr.dyn_count += 1
            else:
                tr.dyn_count = max(0, tr.dyn_count - 1)

        # ---------- 未命中的轨迹：只预测 + missed_count++ ----------
        for ti, tr in enumerate(self._tracks):
            if n_tr > 0 and ti < used_tr.size and used_tr[ti]:
                continue
            tr.missed_count += 1

        # ---------- 未匹配到轨迹的观测：新建 tentative 轨迹（用原始质心）----------
        for mi in range(n_meas):
            if n_meas > 0 and mi < used_meas.size and used_meas[mi]:
                continue
            cx, cy = float(centroids[mi][0]), float(centroids[mi][1])
            tr = ObstacleTrack(track_id=self._next_id, x=cx, y=cy)
            tr.last_update_t = t
            tr.hit_count = 1
            self._tracks.append(tr)
            self._next_id += 1

        # ---------- 删除长期未更新的轨迹 ----------
        self._prune_old_tracks(t)

    # ==================== 输出接口 ====================

    def get_current_obstacle_points(self) -> np.ndarray:
        """所有存活轨迹的位置 (M, 2)。"""
        if not self._tracks:
            return np.empty((0, 2), dtype=np.float32)
        return np.array([[tr.x, tr.y] for tr in self._tracks], dtype=np.float32)

    def _iter_dynamic_tracks(self, min_speed: float) -> List[ObstacleTrack]:
        v_thr = max(float(min_speed), self.min_speed_to_predict)
        dyn_tracks: List[ObstacleTrack] = []
        for tr in self._tracks:
            if not tr.confirmed:
                continue
            if tr.hit_count < self.min_hits_to_predict:
                continue
            if tr.missed_count != 0:
                continue
            spd_eff = max(math.hypot(tr.vx, tr.vy), tr.speed_ema)
            if spd_eff < v_thr:
                continue
            if tr.dyn_count < self.dynamic_required_count:
                continue
            dyn_tracks.append(tr)
        return dyn_tracks

    def get_dynamic_centroids(self, min_speed: float = 0.08) -> List[Tuple[float, float]]:
        """返回被认为是“动态”的轨迹当前质心，用于动静分离。"""
        dyn_tracks = self._iter_dynamic_tracks(min_speed)
        # 用平滑位置，避免对外暴露/动静分离出现跳变
        return [(tr.px_out if tr.px_out != 0.0 or tr.py_out != 0.0 else tr.x,
                 tr.py_out if tr.px_out != 0.0 or tr.py_out != 0.0 else tr.y)
                for tr in dyn_tracks]

    def get_dynamic_trajectory_bands(
        self,
        min_speed: float = 0.08,
        predict_time: Optional[float] = None,
        dt_step: Optional[float] = None,
    ) -> List[List[Tuple[float, float]]]:
        """返回动态障碍的轨迹带：[当前点, 未来若干点]，用于 scan_filter."""
        pt = float(predict_time if predict_time is not None else self.predict_time)
        dt = float(dt_step if dt_step is not None else self.predict_dt)
        num_steps = max(1, int(pt / dt))

        dyn_tracks = self._iter_dynamic_tracks(min_speed)
        out: List[List[Tuple[float, float]]] = []
        for tr in dyn_tracks:
            # 起点和未来点都用平滑位置 + 平滑速度
            px0 = tr.px_out if tr.px_out != 0.0 or tr.py_out != 0.0 else tr.x
            py0 = tr.py_out if tr.px_out != 0.0 or tr.py_out != 0.0 else tr.y
            band: List[Tuple[float, float]] = [(px0, py0)]
            vx, vy = tr.vx_out, tr.vy_out
            for i in range(num_steps):
                dt_i = (i + 1) * dt
                px = px0 + vx * dt_i
                py = py0 + vy * dt_i
                band.append((px, py))
            out.append(band)
        return out

    def get_predictions(
        self,
        predict_time: Optional[float] = None,
        dt_step: Optional[float] = None,
        range_origin_xy: Optional[Tuple[float, float]] = None,
        max_range: float = 4.0,
        max_preds: int = 30,
    ) -> List[dict]:
        """返回结构化预测，供 DWA predictive_score 使用。"""
        pt = float(predict_time if predict_time is not None else self.predict_time)
        dt = float(dt_step if dt_step is not None else self.predict_dt)
        num_steps = max(1, int(pt / dt))

        dyn_tracks = self._iter_dynamic_tracks(self.min_speed_to_predict)

        preds: List[dict] = []
        for tr in dyn_tracks:
            px0 = tr.px_out if tr.px_out != 0.0 or tr.py_out != 0.0 else tr.x
            py0 = tr.py_out if tr.px_out != 0.0 or tr.py_out != 0.0 else tr.y
            if range_origin_xy is not None:
                dx = px0 - float(range_origin_xy[0])
                dy = py0 - float(range_origin_xy[1])
                if dx * dx + dy * dy > max_range * max_range:
                    continue
            vx, vy = tr.vx_out, tr.vy_out
            future: List[Tuple[float, float]] = []
            for i in range(num_steps):
                dt_i = (i + 1) * dt
                px = px0 + vx * dt_i
                py = py0 + vy * dt_i
                future.append((px, py))
            preds.append(
                {
                    "position_xy": (px0, py0),
                    "velocity_xy": (vx, vy),
                    "speed": math.hypot(vx, vy),
                    "future_xy": future,
                }
            )

        if range_origin_xy is not None and len(preds) > max_preds:
            ox, oy = float(range_origin_xy[0]), float(range_origin_xy[1])
            preds.sort(
                key=lambda p: (p["position_xy"][0] - ox) ** 2 + (p["position_xy"][1] - oy) ** 2
            )
            preds = preds[:max_preds]
        return preds

#!/usr/bin/env python3
"""
多目标障碍物轨迹预测模块 (Multi-Target Obstacle Trajectory Predictor)

核心思想（结合我们讨论 + AA/Stone-Soup 风格）：
- 输入：sac_dwa_node 中已经筛选并投影到世界坐标系的 2D 点云 (N, 2)。
- 聚类：简单 DBSCAN 风格，将近邻点聚为障碍物簇。
- 跟踪：对每个障碍物簇维护 Constant Velocity (CV) 轨迹 [x, y, vx, vy]，带 4x4 协方差。
- 关联：使用全局最近邻 (GNN) + Mahalanobis gate 做多目标数据关联，减少 ID 交换和速度突变。
- 漏检处理：轨迹允许短时间只预测不更新 (missed_count)，避免轨迹一帧没测量就瞬间消失。
- 速度估计：基于滤波后的 vx, vy 和其 EMA（速度模长），不放大厘米级抖动；20cm 以内移动视为静止。

对外接口与旧版兼容：
- ObstaclePredictor.__init__(cluster_eps, max_track_age, association_dist, predict_time, predict_dt, ...)
- update(obstacle_points, timestamp=None)
- get_dynamic_centroids(...)
- get_dynamic_trajectory_bands(...)
- get_predictions(...)
"""

# 第二份实现残留的 future import，必须移除以避免 SyntaxError

import math
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np


# ========== 聚类相关 ==========

def _cluster_points(points: np.ndarray, eps: float, min_samples: int = 2) -> List[np.ndarray]:
    """非常简单的 DBSCAN 风格聚类：距离 < eps 视为同一簇。"""
    if points is None or points.shape[0] == 0:
        return []
    n = points.shape[0]
    if n == 1:
        return [points]

    visited = np.zeros(n, dtype=bool)
    clusters: List[np.ndarray] = []

    for i in range(n):
        if visited[i]:
            continue
        visited[i] = True
        idxs = [i]
        queue = [i]
        while queue:
            qi = queue.pop()
            px, py = float(points[qi, 0]), float(points[qi, 1])
            dx = points[:, 0] - px
            dy = points[:, 1] - py
            d2 = dx * dx + dy * dy
            neigh = np.where((~visited) & (d2 < eps * eps))[0]
            if neigh.size > 0:
                visited[neigh] = True
                idxs.extend(neigh.tolist())
                queue.extend(neigh.tolist())
        if len(idxs) >= max(1, int(min_samples)):
            clusters.append(points[idxs])
    return clusters


def _centroid(cluster: np.ndarray) -> Tuple[float, float]:
    """簇质心。"""
    return float(np.mean(cluster[:, 0])), float(np.mean(cluster[:, 1]))


# ========== 轨迹类型 ==========

@dataclass
class ObstacleTrack:
    """单个障碍物轨迹：CV Kalman 状态 + 协方差 + 生命周期信息。"""

    track_id: int
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0
    last_update_t: float = 0.0
    hit_count: int = 0
    age: float = 0.0
    missed_count: int = 0
    confirmed: bool = False

    # 状态协方差 (4x4)
    P: np.ndarray = field(default_factory=lambda: np.diag([0.5, 0.5, 1.0, 1.0]).astype(np.float32))
    # 速度模长 EMA，用于动静判定抑制单帧抖动
    speed_ema: float = 0.0

    def state_vector(self) -> np.ndarray:
        return np.array([[self.x], [self.y], [self.vx], [self.vy]], dtype=np.float32)

    def set_state_vector(self, vec: np.ndarray) -> None:
        self.x = float(vec[0, 0])
        self.y = float(vec[1, 0])
        self.vx = float(vec[2, 0])
        self.vy = float(vec[3, 0])

    def predict(self, dt: float, q_pos: float, q_vel: float) -> None:
        """CV 预测一步：x <- F x, P <- F P F^T + Q。"""
        if dt <= 0.0:
            return
        dt = float(max(0.03, min(dt, 0.5)))

        F = np.array(
            [
                [1.0, 0.0, dt, 0.0],
                [0.0, 1.0, 0.0, dt],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=np.float32,
        )
        Q = np.diag(
            [
                q_pos * dt * dt,
                q_pos * dt * dt,
                q_vel * dt,
                q_vel * dt,
            ]
        ).astype(np.float32)

        x_vec = self.state_vector()
        x_pred = F @ x_vec
        P_pred = F @ self.P @ F.T + Q

        self.set_state_vector(x_pred)
        self.P = P_pred
        self.age += dt

    def maha_distance2(self, z: Tuple[float, float], meas_var: float) -> float:
        """计算测量 z 相对当前状态的 Mahalanobis 距离平方。"""
        zx, zy = float(z[0]), float(z[1])
        H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]], dtype=np.float32)
        R = np.diag([meas_var, meas_var]).astype(np.float32)

        x_vec = self.state_vector()
        z_pred = H @ x_vec
        y = np.array([[zx], [zy]], dtype=np.float32) - z_pred
        S = H @ self.P @ H.T + R
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return float("inf")
        d2 = float((y.T @ S_inv @ y)[0, 0])
        return d2

    def update(
        self,
        z: Tuple[float, float],
        meas_var: float,
        speed_ema_alpha: float,
        now_t: float,
    ) -> None:
        """对位置测量执行 Kalman 更新，并更新速度 EMA。"""
        zx, zy = float(z[0]), float(z[1])
        H = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]], dtype=np.float32)
        R = np.diag([meas_var, meas_var]).astype(np.float32)

        x_vec = self.state_vector()
        z_vec = np.array([[zx], [zy]], dtype=np.float32)
        y = z_vec - H @ x_vec
        S = H @ self.P @ H.T + R
        try:
            K = self.P @ H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return

        x_new = x_vec + K @ y
        I = np.eye(4, dtype=np.float32)
        P_new = (I - K @ H) @ self.P

        self.set_state_vector(x_new)
        self.P = P_new
        self.last_update_t = now_t
        self.hit_count += 1
        self.missed_count = 0

        # 速度 EMA：平滑当前速度模长，抑制单帧抖动
        vx_now, vy_now = float(self.vx), float(self.vy)
        inst_speed = math.hypot(vx_now, vy_now)
        alpha = float(speed_ema_alpha)
        self.speed_ema = (1.0 - alpha) * self.speed_ema + alpha * inst_speed


# ========== 预测器主体 ==========


class ObstaclePredictor:
    """多目标 CV 预测器：聚类 -> GNN 关联 -> CV Kalman 更新 -> 轨迹预测。"""

    def __init__(
        self,
        cluster_eps: float = 0.35,
        min_cluster_points: int = 3,
        max_track_age: float = 2.0,
        min_hits_to_predict: int = 3,
        min_speed_to_predict: float = 0.5,
        association_dist: float = 0.8,
        predict_time: float = 2.5,
        predict_dt: float = 0.25,
        gate_mahalanobis: float = 3.0,
        meas_pos_var: float = 0.06,
        process_noise_pos: float = 0.25,
        process_noise_vel: float = 0.45,
        speed_ema_alpha: float = 0.2,
        max_missed: int = 5,
        min_confirm_hits: int = 3,
    ):
        """
        cluster_eps: 聚类半径 (m)。
        min_cluster_points: 最小簇点数。
        max_track_age: 轨迹最长保留时间 (s)。
        min_hits_to_predict: 至少命中几次才输出预测。
        min_speed_to_predict: 最低速度阈值 (m/s)，低于不视为动态。
        association_dist: 粗欧氏距离 gate (m)。
        predict_time: 预测时长 (s)。
        predict_dt: 预测时间步长 (s)。
        gate_mahalanobis: Mahalanobis 距离门控 sigma。
        meas_pos_var: 位置测量噪声方差。
        process_noise_pos / process_noise_vel: CV 过程噪声。
        speed_ema_alpha: 速度 EMA 平滑系数。
        max_missed: 连续未命中帧数超过后删除轨迹。
        min_confirm_hits: 从 tentative 变 confirmed 所需的最少命中次数。
        """
        self.cluster_eps = float(cluster_eps)
        self.min_cluster_points = int(min_cluster_points)
        self.max_track_age = float(max_track_age)
        self.min_hits_to_predict = int(min_hits_to_predict)
        self.min_speed_to_predict = float(min_speed_to_predict)
        self.association_dist = float(association_dist)
        self.predict_time = float(predict_time)
        self.predict_dt = float(predict_dt)
        self.gate_mahalanobis = float(gate_mahalanobis)
        self.meas_pos_var = float(meas_pos_var)
        self.process_noise_pos = float(process_noise_pos)
        self.process_noise_vel = float(process_noise_vel)
        self.speed_ema_alpha = float(speed_ema_alpha)
        self.max_missed = int(max_missed)
        self.min_confirm_hits = int(min_confirm_hits)

        # 20cm 级分辨率：位置栅格化 + 速度小于该阈值视为静止
        self._pos_grid: float = 0.2          # m
        self.static_speed_epsilon: float = 0.2  # m/s

        self._tracks: List[ObstacleTrack] = []
        self._next_id: int = 0

    # ---- 轨迹管理 ----

    def _prune_old_tracks(self, now_t: float) -> None:
        kept: List[ObstacleTrack] = []
        for tr in self._tracks:
            if (now_t - tr.last_update_t) > self.max_track_age:
                continue
            if tr.missed_count > self.max_missed:
                continue
            kept.append(tr)
        self._tracks = kept

    # ---- 主更新接口 ----

    def update(self, obstacle_points: Optional[np.ndarray], timestamp: Optional[float] = None) -> None:
        """用新一帧 2D 障碍点更新所有轨迹。"""
        t = float(timestamp if timestamp is not None else time.monotonic())

        if obstacle_points is None or obstacle_points.shape[0] == 0:
            # 没有新观测：仅预测 + 累积 missed_count
            for tr in self._tracks:
                dt = t - tr.last_update_t if tr.last_update_t > 0.0 else 0.0
                if dt > 0.0:
                    tr.predict(dt, self.process_noise_pos, self.process_noise_vel)
                    tr.missed_count += 1
            self._prune_old_tracks(t)
            return

        pts = np.asarray(obstacle_points, dtype=np.float32)
        if pts.ndim != 2 or pts.shape[1] != 2:
            return

        clusters = _cluster_points(pts, self.cluster_eps, self.min_cluster_points)
        if clusters:
            centroids = np.array([_centroid(c) for c in clusters], dtype=np.float32)
            g = self._pos_grid
            if g > 0.0:
                centroids = np.round(centroids / g) * g
        else:
            centroids = np.empty((0, 2), dtype=np.float32)

        # 1) 所有轨迹先预测到当前时刻
        for tr in self._tracks:
            dt = t - tr.last_update_t if tr.last_update_t > 0.0 else 0.0
            if dt > 0.0:
                tr.predict(dt, self.process_noise_pos, self.process_noise_vel)

        n_tr = len(self._tracks)
        n_meas = centroids.shape[0]

        # 2) 构造代价矩阵（Mahalanobis 距离），只对粗欧氏距离 gate 内的 pair 计算
        pairs: List[Tuple[int, int]] = []
        if n_tr > 0 and n_meas > 0:
            cost = np.full((n_tr, n_meas), np.inf, dtype=np.float32)
            for ti, tr in enumerate(self._tracks):
                for mi in range(n_meas):
                    cx, cy = float(centroids[mi, 0]), float(centroids[mi, 1])
                    dx = cx - tr.x
                    dy = cy - tr.y
                    if dx * dx + dy * dy > self.association_dist * self.association_dist:
                        continue
                    d2 = tr.maha_distance2((cx, cy), self.meas_pos_var)
                    if d2 <= self.gate_mahalanobis * self.gate_mahalanobis:
                        cost[ti, mi] = d2

            flat_idx = np.argsort(cost, axis=None)
            used_tr = np.zeros(n_tr, dtype=bool)
            used_meas = np.zeros(n_meas, dtype=bool)
            for idx in flat_idx:
                c = float(cost.flat[idx])
                if not math.isfinite(c):
                    break
                ti = int(idx // n_meas)
                mi = int(idx % n_meas)
                if used_tr[ti] or used_meas[mi]:
                    continue
                used_tr[ti] = True
                used_meas[mi] = True
                pairs.append((ti, mi))
        else:
            used_tr = np.zeros(n_tr, dtype=bool)
            used_meas = np.zeros(n_meas, dtype=bool)

        # 3) 对匹配的轨迹执行 Kalman 更新
        for ti, mi in pairs:
            tr = self._tracks[ti]
            cx, cy = float(centroids[mi, 0]), float(centroids[mi, 1])
            tr.update((cx, cy), self.meas_pos_var, self.speed_ema_alpha, t)
            if not tr.confirmed and tr.hit_count >= self.min_confirm_hits:
                tr.confirmed = True

        # 4) 对未命中的轨迹，仅预测并增加 missed_count（预测已在步骤 1 完成）
        for ti, tr in enumerate(self._tracks):
            if n_tr > 0 and ti < used_tr.size and used_tr[ti]:
                continue
            tr.missed_count += 1

        # 5) 未匹配到任何轨迹的观测 -> 新轨迹
        for mi in range(n_meas):
            if n_meas > 0 and mi < used_meas.size and used_meas[mi]:
                continue
            cx, cy = float(centroids[mi, 0]), float(centroids[mi, 1])
            tr = ObstacleTrack(track_id=self._next_id, x=cx, y=cy)
            tr.last_update_t = t
            tr.hit_count = 1
            self._tracks.append(tr)
            self._next_id += 1

        # 6) 删除长期未更新或过老的轨迹
        self._prune_old_tracks(t)

    # ========= 输出接口 =========

    def get_current_obstacle_points(self) -> np.ndarray:
        """当前所有轨迹的位置 (M, 2)。"""
        if not self._tracks:
            return np.empty((0, 2), dtype=np.float32)
        return np.array([[tr.x, tr.y] for tr in self._tracks], dtype=np.float32)

    def get_dynamic_centroids(self, min_speed: float = 0.08) -> List[Tuple[float, float]]:
        """
        返回被认为是“动态”的轨迹当前质心，用于动静分离。

        判定：
        - 轨迹已 confirmed；
        - 命中次数 >= min_hits_to_predict；
        - 当前未 missed；
        - 速度模长 / EMA >= max(min_speed, min_speed_to_predict, static_speed_epsilon)。
        """
        out: List[Tuple[float, float]] = []
        v_min = max(float(min_speed), self.min_speed_to_predict, self.static_speed_epsilon)
        for tr in self._tracks:
            if not tr.confirmed:
                continue
            if tr.hit_count < self.min_hits_to_predict:
                continue
            if tr.missed_count != 0:
                continue
            spd = math.hypot(tr.vx, tr.vy)
            spd_eff = max(spd, tr.speed_ema)
            if spd_eff < v_min:
                continue
            out.append((tr.x, tr.y))
        return out

    def get_dynamic_trajectory_bands(
        self,
        min_speed: float = 0.08,
        predict_time: Optional[float] = None,
        dt_step: Optional[float] = None,
    ) -> List[List[Tuple[float, float]]]:
        """
        返回动态障碍物的轨迹带（当前 + 未来预测），用于 scan_filter。
        每条 band = [position_xy, future_xy[0], future_xy[1], ...]。
        """
        pt = float(predict_time if predict_time is not None else self.predict_time)
        dt = float(dt_step if dt_step is not None else self.predict_dt)
        num_steps = max(1, int(pt / dt))
        v_min = max(float(min_speed), self.min_speed_to_predict, self.static_speed_epsilon)

        out: List[List[Tuple[float, float]]] = []
        for tr in self._tracks:
            if not tr.confirmed:
                continue
            if tr.hit_count < self.min_hits_to_predict:
                continue
            spd = math.hypot(tr.vx, tr.vy)
            spd_eff = max(spd, tr.speed_ema)
            if spd_eff < v_min:
                continue
            band: List[Tuple[float, float]] = [(tr.x, tr.y)]
            for i in range(num_steps):
                dt_i = (i + 1) * dt
                px = tr.x + tr.vx * dt_i
                py = tr.y + tr.vy * dt_i
                band.append((px, py))
            out.append(band)
        return out

    def get_predictions(
        self,
        predict_time: Optional[float] = None,
        dt_step: Optional[float] = None,
        range_origin_xy: Optional[Tuple[float, float]] = None,
        max_range: float = 4.0,
        max_preds: int = 30,
    ) -> List[dict]:
        """
        返回结构化预测，供 DWA predictive_score 使用。

        每个目标:
        {
            "position_xy": (x, y),
            "velocity_xy": (vx, vy),
            "future_xy": [(x1, y1), (x2, y2), ...],
        }
        """
        pt = float(predict_time if predict_time is not None else self.predict_time)
        dt = float(dt_step if dt_step is not None else self.predict_dt)
        num_steps = max(1, int(pt / dt))
        v_min = max(self.min_speed_to_predict, self.static_speed_epsilon)

        preds: List[dict] = []
        for tr in self._tracks:
            if not tr.confirmed:
                continue
            if tr.hit_count < self.min_hits_to_predict:
                continue
            spd = math.hypot(tr.vx, tr.vy)
            spd_eff = max(spd, tr.speed_ema)
            if spd_eff < v_min:
                continue
            if range_origin_xy is not None:
                dx = tr.x - float(range_origin_xy[0])
                dy = tr.y - float(range_origin_xy[1])
                if dx * dx + dy * dy > max_range * max_range:
                    continue
            future: List[Tuple[float, float]] = []
            for i in range(num_steps):
                dt_i = (i + 1) * dt
                px = tr.x + tr.vx * dt_i
                py = tr.y + tr.vy * dt_i
                future.append((px, py))
            preds.append(
                {
                    "position_xy": (tr.x, tr.y),
                    "velocity_xy": (tr.vx, tr.vy),
                    "future_xy": future,
                }
            )

        if range_origin_xy is not None and len(preds) > max_preds:
            ox, oy = float(range_origin_xy[0]), float(range_origin_xy[1])
            preds.sort(
                key=lambda p: (p["position_xy"][0] - ox) ** 2 + (p["position_xy"][1] - oy) ** 2
            )
            preds = preds[:max_preds]
        return preds
