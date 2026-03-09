#!/usr/bin/env python3
"""
Costmap 用 LaserScan 过滤模块（参考 RH-Map）

参考 RH-Map (Region-wise Hash Map, RA-L 2023) 的 S2M-R (Scan-to-Map Removal)：
- 动态区域检测：排除落在动态障碍物轨迹带内的 scan 点
- 轨迹带 = 当前质心 + 预测未来位置（region-wise 覆盖整条轨迹）
- 设为 range_max 使 costmap 不标记、并沿射线 clearing 消除拖影

RH-Map: Online Map Construction Framework of Dynamic Objects Removal
Based on 3D Region-wise Hash Map Structure
"""

import math
import copy
import numpy as np
from typing import List, Optional, Tuple

from sensor_msgs.msg import LaserScan

# 轨迹带: 每条为 [当前点, 未来点1, 未来点2, ...]
TrajectoryBands = List[List[Tuple[float, float]]]


def _collect_exclude_points(
    dynamic_centroids: List[Tuple[float, float]],
    trajectory_bands: List[List[Tuple[float, float]]],
    region_size: float,
) -> List[Tuple[float, float]]:
    """
    收集需排除的区域中心点。参考 RH-Map 的 region-wise 结构。
    trajectory_bands: 每条为 [pos, f1, f2, ...]，经 trail_filter 降采样后的轨迹带
    region_size: 区域粒度 (m)，同一 region 内点合并减少计算
    """
    pts = list(dynamic_centroids)
    if trajectory_bands:
        for band in trajectory_bands:
            pts.extend(band)
    if not pts or region_size <= 0:
        return pts

    # 简单去重：距离 < region_size 的点视为同区域，只保留代表点
    kept = [pts[0]]
    for p in pts[1:]:
        too_close = False
        for k in kept:
            dx, dy = p[0] - k[0], p[1] - k[1]
            if dx * dx + dy * dy < region_size * region_size:
                too_close = True
                break
        if not too_close:
            kept.append(p)
    return kept


def _distance_sq_to_segment(
    px: np.ndarray,
    py: np.ndarray,
    ax: float,
    ay: float,
    bx: float,
    by: float,
) -> np.ndarray:
    """计算点集到线段 AB 的平方距离（向量化）。"""
    abx = bx - ax
    aby = by - ay
    ab2 = abx * abx + aby * aby
    if ab2 <= 1e-12:
        dx = px - ax
        dy = py - ay
        return dx * dx + dy * dy
    apx = px - ax
    apy = py - ay
    t = (apx * abx + apy * aby) / ab2
    t = np.clip(t, 0.0, 1.0)
    qx = ax + t * abx
    qy = ay + t * aby
    dx = px - qx
    dy = py - qy
    return dx * dx + dy * dy


def filter_scan_exclude_dynamic(
    scan: LaserScan,
    dynamic_centroids: List[Tuple[float, float]],
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    exclude_radius: float = 0.5,
    trajectory_bands: Optional[TrajectoryBands] = None,
    region_size: float = 0.2,
) -> LaserScan:
    """
    过滤 LaserScan：落在动态轨迹带内的射线设为 range_max。
    参考 RH-Map S2M-R：排除动态区域，实现 Scan-to-Map 的 clearing，消除 ghost trail。

    dynamic_centroids: 动态质心（当前）
    trajectory_bands: 动态轨迹带 [pos, f1, f2, ...]，覆盖整条预测路径
    region_size: 区域合并粒度 (m)，同 region 内点合并
    """
    out = copy.deepcopy(scan)

    exclude_pts = _collect_exclude_points(
        dynamic_centroids,
        trajectory_bands or [],
        region_size,
    )
    if not exclude_pts:
        return out

    ranges_raw = np.asarray(out.ranges, dtype=np.float64)
    valid_min = max(float(scan.range_min), 0.03)
    finite = np.isfinite(ranges_raw)
    valid = finite & (ranges_raw >= valid_min) & (ranges_raw <= scan.range_max)
    # 非法值（nan/inf）不参与几何计算，并在输出中替换为 range_max。
    ranges = np.where(finite, ranges_raw, float(scan.range_max))

    angles = scan.angle_min + np.arange(len(ranges), dtype=np.float64) * scan.angle_increment
    cos_a = np.cos(angles)
    sin_a = np.sin(angles)

    c = math.cos(robot_yaw)
    s = math.sin(robot_yaw)
    safe_ranges = np.where(valid, ranges, 0.0)
    wx = robot_x + (safe_ranges * cos_a) * c - (safe_ranges * sin_a) * s
    wy = robot_y + (safe_ranges * cos_a) * s + (safe_ranges * sin_a) * c

    dynamic_mask = np.zeros(len(ranges), dtype=bool)
    for cx, cy in exclude_pts:
        dx = wx - cx
        dy = wy - cy
        d2 = dx * dx + dy * dy
        dynamic_mask |= (valid & (d2 <= exclude_radius * exclude_radius))

    # 轨迹“走廊”过滤：不仅剔除离散轨迹点，还剔除轨迹段附近射线，减少偶发拖影残留。
    if trajectory_bands:
        corridor_r2 = (exclude_radius * 1.15) * (exclude_radius * 1.15)
        for band in trajectory_bands:
            if len(band) < 2:
                continue
            for i in range(len(band) - 1):
                ax, ay = band[i]
                bx, by = band[i + 1]
                d2_seg = _distance_sq_to_segment(wx, wy, ax, ay, bx, by)
                dynamic_mask |= (valid & (d2_seg <= corridor_r2))

    new_ranges = [float(r) for r in ranges]
    for i in range(len(new_ranges)):
        if dynamic_mask[i]:
            new_ranges[i] = float(scan.range_max)

    out.ranges = new_ranges
    return out
