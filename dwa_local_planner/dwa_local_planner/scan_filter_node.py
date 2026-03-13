#!/usr/bin/env python3
"""
Scan Filter Node：在 scan 进入 costmap 之前过滤动态障碍物及其轨迹。

订阅 /scan，用 ObstaclePredictor 检测动态障碍物，将动态区域内的点设为 inf（无障碍），
发布 /scan_filtered 供 Nav2 obstacle_layer 使用，从源头消除 ghost 膨胀带。

数据流: /scan -> scan_filter_node -> /scan_filtered -> Nav2 obstacle_layer
"""

import math
import time
from typing import List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def _point_in_regions(x: float, y: float, regions: List[Tuple[float, float]], radius: float) -> bool:
    """点 (x,y) 是否在任一 region 质心的 radius 范围内"""
    for cx, cy in regions:
        if (x - cx) ** 2 + (y - cy) ** 2 <= radius * radius:
            return True
    return False


def _point_near_polyline(x: float, y: float, polyline: List[Tuple[float, float]], radius: float) -> bool:
    """点 (x,y) 是否在折线 polyline 的 radius 范围内"""
    for i in range(len(polyline) - 1):
        ax, ay = polyline[i]
        bx, by = polyline[i + 1]
        # 点到线段的最短距离
        abx, aby = bx - ax, by - ay
        apx, apy = x - ax, y - ay
        t = max(0.0, min(1.0, (apx * abx + apy * aby) / max(1e-12, abx * abx + aby * aby)))
        px = ax + t * abx
        py = ay + t * aby
        d2 = (x - px) ** 2 + (y - py) ** 2
        if d2 <= radius * radius:
            return True
    if len(polyline) == 1:
        cx, cy = polyline[0]
        if (x - cx) ** 2 + (y - cy) ** 2 <= radius * radius:
            return True
    return False


class ScanFilterNode(Node):
    def __init__(self):
        super().__init__("scan_filter_node")

        from dwa_local_planner.obstacle_predictor_clean import ObstaclePredictor

        # 小截面静态目标（站立行人、杆子）需要尽快被视为静态；
        # 同时抑制由激光抖动导致的小速度误判为动态。
        static_confirm = self.declare_parameter("static_confirm_time", 0.7).value
        cluster_eps = self.declare_parameter("cluster_eps", 0.18).value
        self.predictor = ObstaclePredictor(
            # 动态进入/退出速度阈值：抑制 <0.3m/s 的抖动被当成动态
            dynamic_enter_speed=0.30,
            dynamic_exit_speed=0.15,
            # 持续时间：真实动态即便短暂停顿，仍保持一小段时间动态状态，避免拖影残留
            dynamic_hold_time=1.0,
            static_confirm_time=float(static_confirm),
            cluster_eps=float(cluster_eps),
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 过滤半径：动态质心/轨迹/拖影的邻域，在此范围内的 scan 点将被过滤
        en = self.declare_parameter("enabled", True).value
        self.enabled = en if isinstance(en, bool) else str(en).lower() in ("true", "1", "yes")
        # 过滤半径略小一点，减少多人场景下大面积“黑洞”，但仍能覆盖动态目标本体
        self.filter_radius = self.declare_parameter("filter_radius", 0.45).value
        # 保护半径：仅保护静态障碍核心，不宜过大否则拖影在静态附近无法过滤
        self.protect_radius = self.declare_parameter("protect_radius", 0.36).value
        self.target_frame = self.declare_parameter("target_frame", "odom").value
        self.scan_topic = self.declare_parameter("scan_topic", "/scan").value
        self.output_topic = self.declare_parameter("output_topic", "/scan_filtered").value

        self.sub = self.create_subscription(
            LaserScan, self.scan_topic, self.on_scan, 20
        )
        self.pub = self.create_publisher(LaserScan, self.output_topic, 20)

        self._last_log_t = 0.0
        self.get_logger().info(
            f"Scan filter started: {self.scan_topic} -> {self.output_topic}, "
            f"filter_radius={self.filter_radius}, protect_radius={self.protect_radius}"
        )

    def on_scan(self, msg: LaserScan):
        if not self.enabled:
            self.pub.publish(msg)
            return
        t = time.monotonic()
        try:
            trans = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.08),
            )
        except Exception:
            self.pub.publish(msg)
            return

        ranges = np.asarray(msg.ranges, dtype=np.float64)
        valid_min = max(float(msg.range_min), 0.03)
        valid = np.isfinite(ranges) & (ranges > valid_min) & (ranges < msg.range_max)
        angles = msg.angle_min + np.arange(len(ranges), dtype=np.float64) * msg.angle_increment

        # 转换到 target_frame 的障碍点
        rx = trans.transform.translation.x
        ry = trans.transform.translation.y
        q = trans.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        c, s = math.cos(yaw), math.sin(yaw)

        pts_world: List[Tuple[float, float]] = []
        idx_valid: List[int] = []
        for i in np.where(valid)[0]:
            r = ranges[i]
            a = angles[i]
            lx = r * math.cos(a)
            ly = r * math.sin(a)
            wx = rx + c * lx - s * ly
            wy = ry + s * lx + c * ly
            pts_world.append((wx, wy))
            idx_valid.append(i)

        if pts_world:
            arr = np.array(pts_world, dtype=np.float64)
            self.predictor.update(arr, t)

        centroids, bands, trail = self.predictor.get_confirmed_dynamic_for_costmap_filter()
        protect = self.predictor.get_protect_centroids()

        out_ranges = np.array(msg.ranges, dtype=np.float32)
        filtered_count = 0

        for i, (wx, wy) in zip(idx_valid, pts_world):
            in_dynamic = (
                _point_in_regions(wx, wy, centroids, self.filter_radius)
                or any(_point_near_polyline(wx, wy, band, self.filter_radius) for band in bands)
                or _point_in_regions(wx, wy, trail, self.filter_radius)
            )
            in_protect = _point_in_regions(wx, wy, protect, self.protect_radius)
            if in_dynamic and not in_protect:
                out_ranges[i] = float("inf")
                filtered_count += 1

        out_msg = LaserScan()
        out_msg.header = msg.header
        out_msg.angle_min = msg.angle_min
        out_msg.angle_max = msg.angle_max
        out_msg.angle_increment = msg.angle_increment
        out_msg.time_increment = msg.time_increment
        out_msg.scan_time = msg.scan_time
        out_msg.range_min = msg.range_min
        out_msg.range_max = msg.range_max
        out_msg.ranges = out_ranges.tolist()
        self.pub.publish(out_msg)

        if t - self._last_log_t > 5.0:
            self._last_log_t = t
            n_dyn = len(centroids) + len(bands)
            self.get_logger().info(
                f"Filter: {filtered_count} pts filtered, {n_dyn} dynamic tracks"
            )


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
