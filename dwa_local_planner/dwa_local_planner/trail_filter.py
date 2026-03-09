#!/usr/bin/env python3
"""
动态障碍物轨迹拖影过滤模块 (Trail Filter for Dynamic Obstacles)

参考 RH-Map (RA-L 2023) 的 region-wise 思想与 AA/Stone-Soup DistanceGater：
- 区域化：轨迹点按 region 粒度合并，减少冗余（类似 RH-Map 的 cube/region 结构）
- 距离门控：相邻点过密时降采样，避免膨胀圆过度重叠
- 拖影消除：过滤后的轨迹带供 scan_filter 排除，配合 costmap clearing 消除 ghost trail

RH-Map: Online Map Construction Framework of Dynamic Objects Removal
Based on 3D Region-wise Hash Map Structure
"""


import math
import numpy as np
from typing import List, Optional, Tuple


def _point_dist(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    return math.sqrt(dx * dx + dy * dy)


class TrailFilter:
    """
    轨迹拖影过滤器：对动态障碍物的 future_xy 轨迹带做降采样/合并，
    避免膨胀圆过度重叠导致的冗余与计算负担。
    参考 AA DistanceGater：用距离阈值过滤。
    """

    def __init__(
        self,
        min_step_dist: float = 0.15,
        inflation_radius: float = 0.25,
        max_points_per_trail: int = 15,
    ):
        """
        min_step_dist: 相邻保留点最小间距 (m)，小于此距则合并/跳过
        inflation_radius: 轨迹点膨胀半径 (m)，用于 overlap 判断
        max_points_per_trail: 单条轨迹最多保留点数
        """
        self.min_step_dist = min_step_dist
        self.inflation_radius = inflation_radius
        self.max_points_per_trail = max_points_per_trail

    def filter_trail(self, points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        对单条轨迹做拖影过滤：Douglas-Peucker 风格 + 距离门控。
        保留首尾及满足 min_step_dist 的中间点。
        """
        if not points or len(points) <= 2:
            return list(points)

        out = [points[0]]
        last = points[0]
        for i in range(1, len(points)):
            p = points[i]
            if _point_dist(last, p) >= self.min_step_dist:
                out.append(p)
                last = p
        if points[-1] != last and _point_dist(out[-1], points[-1]) > 1e-6:
            out.append(points[-1])

        if len(out) > self.max_points_per_trail:
            step = (len(out) - 1) / (self.max_points_per_trail - 1)
            idx = [int(round(i * step)) for i in range(self.max_points_per_trail)]
            idx[-1] = len(out) - 1
            out = [out[i] for i in idx]
        return out

    def filter_prediction(
        self,
        pred: dict,
        inflation_radius: Optional[float] = None,
    ) -> dict:
        """
        对单条预测的 future_xy 做拖影过滤，返回过滤后的预测（含 position_xy, velocity_xy, future_xy）。
        """
        r = inflation_radius if inflation_radius is not None else self.inflation_radius
        pos = pred.get("position_xy", (0, 0))
        vel = pred.get("velocity_xy", (0, 0))
        future = pred.get("future_xy", [])

        full_trail = [pos] + [tuple(p) for p in future]
        filtered = self.filter_trail(full_trail)
        if not filtered:
            return pred
        new_pos = filtered[0]
        new_future = filtered[1:] if len(filtered) > 1 else []

        return {
            "position_xy": new_pos,
            "velocity_xy": vel,
            "future_xy": new_future,
            "inflation_radius": r,
        }

    def filter_predictions(
        self,
        predictions: List[dict],
        inflation_radius: Optional[float] = None,
    ) -> List[dict]:
        """对预测列表批量应用拖影过滤"""
        return [
            self.filter_prediction(p, inflation_radius)
            for p in predictions
        ]


def separate_dynamic_static_points(
    points: np.ndarray,
    dynamic_centroids: List[Tuple[float, float]],
    exclude_radius: float = 0.5,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    将障碍点分离为动态与静态。
    落在 dynamic_centroids 某点 exclude_radius 内的点视为动态，其余为静态。

    Returns:
        (static_points, dynamic_points)
    """
    if points is None or points.shape[0] == 0:
        return np.empty((0, 2), dtype=np.float32), np.empty((0, 2), dtype=np.float32)

    static_mask = np.ones(points.shape[0], dtype=bool)
    for cx, cy in dynamic_centroids:
        dx = points[:, 0] - cx
        dy = points[:, 1] - cy
        d2 = dx * dx + dy * dy
        static_mask &= d2 > exclude_radius * exclude_radius

    static_pts = points[static_mask]
    dynamic_pts = points[~static_mask]
    return static_pts.astype(np.float32), dynamic_pts.astype(np.float32)
