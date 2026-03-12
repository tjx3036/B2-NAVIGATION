#!/usr/bin/env python3
"""
障碍物轨迹预测与跟踪模块 (Obstacle Trajectory Prediction & Tracking)

基于 AA/Stone-Soup 中的 Constant Velocity (CV) 模型与 Kalman 预测思想，实现轻量级前瞻性避障：
- 障碍物聚类：将点云聚类为离散障碍物
- 轨迹跟踪：对每个障碍物维护状态 (x, y, vx, vy)，使用 alpha-beta 滤波估计速度
- 轨迹预测：按 CV 模型预测未来时刻的障碍物位置，供 DWA 前瞻性评分使用

参考：Stone-Soup ConstantVelocity 与 KalmanPredictor
作者：dong (adapted from AA/Stone-Soup)
版本：1.0
"""

import math
import time
import numpy as np
from typing import List, Optional, Tuple


class ObstacleTrack:
    """单个障碍物轨迹，状态 [x, y, vx, vy]，使用 alpha-beta 滤波平滑跟踪"""

    def __init__(self, x: float, y: float, track_id: int, alpha: float = 0.7, beta: float = 0.5):
        self.track_id = track_id
        self.x = float(x)
        self.y = float(y)
        self.vx = 0.0
        self.vy = 0.0
        self.alpha = alpha  # 位置平滑系数
        self.beta = beta    # 速度平滑系数
        self.last_update_t = 0.0
        self.hit_count = 1
        self.age = 0.0

    def update(self, x: float, y: float, t: float) -> None:
        """用新观测更新轨迹，估计速度"""
        dt = t - self.last_update_t
        if dt <= 0:
            dt = 0.05  # 最小步长
        if self.last_update_t <= 0:
            self.last_update_t = t
            self.x = x
            self.y = y
            return
        # 速度估计：(新位置 - 旧位置) / dt
        vx_raw = (x - self.x) / dt
        vy_raw = (y - self.y) / dt
        self.vx = self.vx + self.beta * (vx_raw - self.vx)
        self.vy = self.vy + self.beta * (vy_raw - self.vy)
        self.x = self.x + self.alpha * (x - self.x)
        self.y = self.y + self.alpha * (y - self.y)
        self.last_update_t = t
        self.hit_count += 1

    def predict(self, dt: float) -> Tuple[float, float]:
        """CV 模型预测：pos + velocity * dt"""
        return (
            self.x + self.vx * dt,
            self.y + self.vy * dt,
        )

    def predict_trajectory(self, dt_step: float, num_steps: int) -> List[Tuple[float, float]]:
        """预测未来一段轨迹上的位置序列"""
        pts = []
        for i in range(num_steps):
            t = (i + 1) * dt_step
            pts.append(self.predict(t))
        return pts


def _cluster_points(points: np.ndarray, eps: float, min_samples: int = 2) -> List[np.ndarray]:
    """
    简单 DBSCAN 风格聚类：距离 < eps 的点归为同一障碍物
    points: (N, 2) 世界坐标
    """
    if points is None or points.shape[0] == 0:
        return []
    n = points.shape[0]
    if n == 1:
        return [points]
    visited = np.zeros(n, dtype=bool)
    clusters = []

    for i in range(n):
        if visited[i]:
            continue
        visited[i] = True
        cluster_indices = [i]
        queue = [i]
        while queue:
            qi = queue.pop()
            px, py = points[qi, 0], points[qi, 1]
            for j in range(n):
                if visited[j]:
                    continue
                dx = points[j, 0] - px
                dy = points[j, 1] - py
                if dx * dx + dy * dy < eps * eps:
                    visited[j] = True
                    cluster_indices.append(j)
                    queue.append(j)
        # 最小簇点数过滤：抑制孤立噪声点被当作障碍物轨迹
        if len(cluster_indices) >= max(1, int(min_samples)):
            clusters.append(points[cluster_indices])
    return clusters


def _centroid(cluster: np.ndarray) -> Tuple[float, float]:
    """簇的质心"""
    return (float(np.mean(cluster[:, 0])), float(np.mean(cluster[:, 1])))


class ObstaclePredictor:
    """
    障碍物预测器：聚类 -> 跟踪 -> 预测

    输入：世界坐标下的障碍点 (scan/costmap 投影)
    输出：当前 + 预测时段内的障碍物位置点集，供 DWA 前瞻性避障
    """

    def __init__(
        self,
        cluster_eps: float = 0.25,
        min_cluster_points: int = 1,
        max_track_age: float = 1.5,
        min_hits_to_predict: int = 1,
        min_speed_to_predict: float = 0.08,
        association_dist: float = 0.5,
        predict_time: float = 1.4,
        predict_dt: float = 0.1,
    ):
        """
        cluster_eps: 聚类半径 (m)，小于此距离的点归为同一障碍
        min_cluster_points: 最小簇点数
        max_track_age: 轨迹最大年龄 (s)，超时删除
        min_hits_to_predict: 至少更新几次才参与速度预测
        association_dist: 观测-轨迹关联距离阈值 (m)
        predict_time: 预测时长 (s)
        predict_dt: 预测步长 (s)
        """
        self.cluster_eps = cluster_eps
        self.min_cluster_points = min_cluster_points
        self.max_track_age = max_track_age
        self.min_hits_to_predict = min_hits_to_predict
        self.min_speed_to_predict = min_speed_to_predict
        self.association_dist = association_dist
        self.predict_time = predict_time
        self.predict_dt = predict_dt

        self._tracks: List[ObstacleTrack] = []
        self._next_id = 0
        self._last_t = 0.0

    def update(self, obstacle_points: Optional[np.ndarray], timestamp: Optional[float] = None) -> None:
        """
        用新一帧障碍点更新预测器
        obstacle_points: (N, 2) 世界坐标，None 表示无数据
        timestamp: 可选，默认用 time.monotonic()
        """
        t = timestamp if timestamp is not None else time.monotonic()
        if obstacle_points is None or obstacle_points.shape[0] == 0:
            self._prune_old_tracks(t)
            return

        arr = np.asarray(obstacle_points, dtype=np.float64)
        if arr.ndim != 2 or arr.shape[1] != 2:
            return

        clusters = _cluster_points(arr, self.cluster_eps, self.min_cluster_points)
        centroids = [_centroid(c) for c in clusters]

        # 关联：新观测与已有轨迹
        used = [False] * len(centroids)
        for tr in self._tracks:
            tr.age = t - tr.last_update_t
            best_i = -1
            best_d2 = self.association_dist * self.association_dist
            for i, (cx, cy) in enumerate(centroids):
                if used[i]:
                    continue
                dx = cx - tr.x
                dy = cy - tr.y
                d2 = dx * dx + dy * dy
                if d2 < best_d2:
                    best_d2 = d2
                    best_i = i
            if best_i >= 0:
                used[best_i] = True
                tr.update(centroids[best_i][0], centroids[best_i][1], t)

        # 未关联的观测 -> 新轨迹（初始化 last_update_t 避免被 prune 误删）
        for i, (cx, cy) in enumerate(centroids):
            if not used[i]:
                tr = ObstacleTrack(cx, cy, self._next_id)
                tr.last_update_t = t
                self._tracks.append(tr)
                self._next_id += 1

        self._prune_old_tracks(t)
        self._last_t = t

    def _prune_old_tracks(self, t: float) -> None:
        """删除超龄或长期未更新的轨迹"""
        self._tracks = [
            tr for tr in self._tracks
            if (t - tr.last_update_t) <= self.max_track_age
        ]

    def get_current_obstacle_points(self) -> np.ndarray:
        """当前时刻所有轨迹的位置，形状 (M, 2)"""
        if not self._tracks:
            return np.empty((0, 2), dtype=np.float32)
        return np.array([[tr.x, tr.y] for tr in self._tracks], dtype=np.float32)

    def get_predicted_obstacle_points(
        self,
        predict_time: Optional[float] = None,
        dt_step: Optional[float] = None,
    ) -> np.ndarray:
        """
        返回预测时段内所有障碍物的预测位置点 (用于 DWA 前瞻性障碍评分)
        在 [dt_step, 2*dt_step, ..., predict_time] 各时刻采样
        """
        pt = predict_time if predict_time is not None else self.predict_time
        dt = dt_step if dt_step is not None else self.predict_dt
        num_steps = max(1, int(pt / dt))

        pts = []
        for tr in self._tracks:
            for (px, py) in tr.predict_trajectory(dt, num_steps):
                pts.append([px, py])
        if not pts:
            return np.empty((0, 2), dtype=np.float32)
        return np.array(pts, dtype=np.float32)

    def get_obstacle_points_at_time(self, t_offset: float) -> np.ndarray:
        """
        返回在 t_offset 时刻（相对最近一次 update 时刻）的障碍物位置。
        用于轨迹-障碍物时间对准：轨迹点 t_i 仅与 t_i 时刻的障碍物位置比较。
        t_offset=0 表示当前时刻。
        """
        if not self._tracks:
            return np.empty((0, 2), dtype=np.float32)
        pts = []
        for tr in self._tracks:
            px, py = tr.predict(t_offset)
            pts.append([px, py])
        return np.array(pts, dtype=np.float32)

    def get_combined_obstacle_points(
        self,
        predict_time: Optional[float] = None,
        dt_step: Optional[float] = None,
        include_current: bool = True,
    ) -> np.ndarray:
        """
        合并当前 + 预测点，供 DWA 统一用于障碍距离计算
        include_current: 是否包含当前时刻位置
        """
        curr = self.get_current_obstacle_points()
        predicted = self.get_predicted_obstacle_points(predict_time, dt_step)
        if include_current and curr.shape[0] > 0:
            if predicted.shape[0] > 0:
                return np.concatenate([curr, predicted], axis=0)
            return curr
        return predicted if predicted.shape[0] > 0 else curr

    def get_track_count(self) -> int:
        """当前跟踪的障碍物数量"""
        return len(self._tracks)

    def get_dynamic_centroids(self, min_speed: float = 0.08) -> List[Tuple[float, float]]:
        """
        返回速度超过阈值的轨迹质心，用于动静分离：这些点附近的 scan 点视为动态，不送入 obstacle_score。
        min_speed: 速度模长阈值 (m/s)，超过则视为动态障碍物
        """
        out = []
        for tr in self._tracks:
            if tr.hit_count < self.min_hits_to_predict:
                continue
            spd = math.sqrt(tr.vx * tr.vx + tr.vy * tr.vy)
            if spd >= min_speed:
                out.append((tr.x, tr.y))
        return out

    def get_dynamic_trajectory_bands(
        self,
        min_speed: float = 0.08,
        predict_time: Optional[float] = None,
        dt_step: Optional[float] = None,
    ) -> List[List[Tuple[float, float]]]:
        """
        返回动态障碍物的轨迹带（当前 + 预测未来点），供 RH-Map 风格 scan 过滤使用。
        每条 band = [position_xy, future_xy[0], future_xy[1], ...]
        """
        pt = predict_time if predict_time is not None else self.predict_time
        dt = dt_step if dt_step is not None else self.predict_dt
        num_steps = max(1, int(pt / dt))

        out = []
        for tr in self._tracks:
            if tr.hit_count < self.min_hits_to_predict:
                continue
            spd = math.sqrt(tr.vx * tr.vx + tr.vy * tr.vy)
            if spd < min_speed:
                continue
            band = [(tr.x, tr.y)]
            for i in range(num_steps):
                px, py = tr.predict((i + 1) * dt)
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
        返回结构化预测，供 DWA predictive_score（TTC + 时空距离）使用。
        每个目标: position_xy, velocity_xy, future_xy
        range_origin_xy: 机器人位置，用于量程过滤，避免世界坐标下远处点误参与
        max_range: 只保留距离 range_origin 小于此值的障碍 (m)
        max_preds: 最多保留的预测目标数（取距离最近的），避免 pred_cnt 暴增导致震荡
        """
        pt = predict_time if predict_time is not None else self.predict_time
        dt = dt_step if dt_step is not None else self.predict_dt
        num_steps = max(1, int(pt / dt))

        preds = []
        for tr in self._tracks:
            if tr.hit_count < self.min_hits_to_predict:
                continue
            speed = math.sqrt(tr.vx * tr.vx + tr.vy * tr.vy)
            if speed < self.min_speed_to_predict:
                continue
            if range_origin_xy is not None:
                dx = tr.x - range_origin_xy[0]
                dy = tr.y - range_origin_xy[1]
                if dx * dx + dy * dy > max_range * max_range:
                    continue
            future = [tr.predict((i + 1) * dt) for i in range(num_steps)]
            preds.append({
                "position_xy": (tr.x, tr.y),
                "velocity_xy": (tr.vx, tr.vy),
                "future_xy": future,
            })

        if range_origin_xy is not None and len(preds) > max_preds:
            preds.sort(
                key=lambda p: (p["position_xy"][0] - range_origin_xy[0]) ** 2
                + (p["position_xy"][1] - range_origin_xy[1]) ** 2,
            )
            preds = preds[:max_preds]
        return preds
