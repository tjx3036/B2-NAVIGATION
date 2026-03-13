#!/usr/bin/env python3
"""
SAC-DWA 局部规划器桥接节点，用于 RCI 四足仿真。

使用自定义 DWA 算法替代 Nav2 的 RegulatedPurePursuitController 作为局部规划器。

输入:
- /odom (nav_msgs/Odometry)
- /plan (nav_msgs/Path) - Nav2 planner_server 发布的全局路径
- /scan (sensor_msgs/LaserScan)
- /local_costmap/costmap (nav_msgs/OccupancyGrid)

输出:
- /cmd_vel (geometry_msgs/Twist) - 机器人速度命令，rl_sim 订阅此话题
"""

import math
import time
import threading
from typing import Optional

import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from gazebo_msgs.msg import ModelStates
from nav2_msgs.action import FollowPath
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster, Buffer, TransformListener

from dwa_local_planner.dwa_planner import DWAPlanner
from dwa_local_planner.obstacle_predictor import ObstaclePredictor
from dwa_local_planner.trail_filter import TrailFilter, separate_dynamic_static_points
from dwa_local_planner.scan_filter import filter_scan_exclude_dynamic


class SACDWANode(Node):
    def __init__(self):
        super().__init__("sac_dwa_local_planner")
        self.declare_parameter("alias_map_to_odom", False)
        self._alias_map_to_odom = bool(self.get_parameter("alias_map_to_odom").value)
        # 控制环与动作服务分组，避免 timer 计算占满线程导致 action 超时
        self._ctrl_group = MutuallyExclusiveCallbackGroup()
        self._action_group = ReentrantCallbackGroup()
        self.planner = DWAPlanner()
        self.planner.set_debug(False)

        self.control_hz = 8.0           # 控制频率：提高控制连续性，减少“走走停停”
        self.planner.control_interval = 1.0 / self.control_hz

        # 前瞻性避障：障碍物轨迹预测与跟踪（cluster_eps 稍大避免点云过细分簇）
        self.obstacle_predictor = ObstaclePredictor(
            cluster_eps=0.4,
            max_track_age=1.5,
            association_dist=0.5,
            predict_time=self.planner.predict_time,
            predict_dt=self.planner.control_interval,
        )
        # 拖影过滤：对动态障碍物轨迹带做降采样，参考 AA DistanceGater
        self.trail_filter = TrailFilter(
            min_step_dist=0.15,
            inflation_radius=0.25,
            max_points_per_trail=15,
        )
        self.dynamic_exclude_radius = 0.6  # 动静分离：略增大半径，减少动态拖影残留

        # 参考 ros2-robot-navigation-exploration: lookahead 0.5, 前方角度范围
        self.lookahead_dist = 0.9       # 前瞻距离 (m)
        self.lookahead_angle_range = math.pi * 2 / 3  # 前方 120° 内选点（exploration 60°偏严，弯道易无点）
        # 仅用于“硬急停”，阈值不宜过大，否则会在未触及膨胀边界前就频繁停住
        self.emergency_stop_dist = 0.18
        self.min_cmd_linear_speed = 0.12
        self.goal_stop_tolerance = 0.25  # 到达判定
        self.goal_slow_dist = 1.0       # 该距离内减速
        self.slowing_factor = 2.0
        # 近体盲区：抑制传感器自体回波/近场噪声把机器人“自己当障碍”
        self.self_filter_radius = 0.30

        self._scan_min_dist = 10.0
        self._scan_points_xy = np.empty((0, 2), dtype=np.float32)
        self._pc_points_xy = np.empty((0, 2), dtype=np.float32)
        self.scan_timeout_s = 1.2
        self.costmap_timeout_s = 1.0
        self._last_scan_t = 0.0
        self._last_points_t = 0.0
        self._last_costmap_t = 0.0
        self._last_plan: Optional[Path] = None
        self._plan_frame = "map"
        self._odom_frame = "odom"
        self._has_odom = False
        self._has_real_odom = False
        self._robot_model_candidates = ["robot_model", "b2_gazebo", "b2"]
        self._final_goal_xy: Optional[tuple[float, float]] = None
        self._goal_token = 0
        self._goal_lock = threading.Lock()
        self._last_goal_accept_log_t = 0.0
        self._last_frame_warn_t = 0.0
        self._last_diag_log_t = 0.0
        self._progress_anchor_t = 0.0
        self._progress_anchor_xy: Optional[tuple[float, float]] = None
        self._recovery_until_t = 0.0
        self._recovery_turn_sign = 1.0
        self._no_feasible_boost_until_t = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 最近一次用于 DWA 的“纯静态障碍点”，供 GUI 直接读取算法内部动静分离结果
        self._last_static_pts = np.empty((0, 2), dtype=np.float32)

        self.sub_odom = self.create_subscription(
            Odometry, "/odom", self.on_odom, 20, callback_group=self._ctrl_group
        )
        self.sub_model_states = self.create_subscription(
            ModelStates, "/gazebo/model_states", self.on_model_states, 20, callback_group=self._ctrl_group
        )
        self.sub_plan = self.create_subscription(
            Path, "/plan", self.on_plan, 10, callback_group=self._ctrl_group
        )
        self.sub_scan = self.create_subscription(
            LaserScan, "/scan", self.on_scan, 20, callback_group=self._ctrl_group
        )
        self.sub_points = self.create_subscription(
            PointCloud2, "/velodyne_points", self.on_points, 10, callback_group=self._ctrl_group
        )
        self.sub_costmap = self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self.on_costmap, 10, callback_group=self._ctrl_group
        )

        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 20)
        self.pub_odom = self.create_publisher(Odometry, "/odom", 20)
        self.pub_scan_for_costmap = self.create_publisher(LaserScan, "/scan_for_costmap", 10)
        # 仅用于可视化/调试：发布 DWA 内部的动态/静态障碍物信息
        self.pub_dynamic_info = self.create_publisher(Float32MultiArray, "/dwa_dynamic_obstacles_info", 10)
        self.pub_static_info = self.create_publisher(Float32MultiArray, "/dwa_static_obstacles_info", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / self.control_hz, self.on_timer, callback_group=self._ctrl_group)
        self.follow_path_server = ActionServer(
            self,
            FollowPath,
            "follow_path",
            execute_callback=self._execute_follow_path,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self._action_group,
        )
        self.clear_local_costmap_srv = self.create_service(
            ClearEntireCostmap,
            "/local_costmap/clear_entirely_local_costmap",
            self._on_clear_costmap,
            callback_group=self._action_group,
        )
        self.clear_global_costmap_srv = self.create_service(
            ClearEntireCostmap,
            "/global_costmap/clear_entirely_global_costmap",
            self._on_clear_costmap,
            callback_group=self._action_group,
        )
        self.get_logger().info("DWA local planner started (replacing Nav2 controller for /cmd_vel).")
        self.get_logger().info("FollowPath action server is active in sac_dwa_node.")
        if self._alias_map_to_odom:
            self.get_logger().warn("alias_map_to_odom enabled: treating map and odom as identical frames.")

    def _on_goal(self, goal_request: FollowPath.Goal):
        if len(goal_request.path.poses) == 0:
            self.get_logger().warn("Reject FollowPath goal: empty path.")
            return GoalResponse.REJECT
        with self._goal_lock:
            self._goal_token += 1
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle):
        self._last_plan = None
        self._final_goal_xy = None
        with self._goal_lock:
            self._goal_token += 1
        self.pub_cmd.publish(Twist())
        return CancelResponse.ACCEPT

    def _on_clear_costmap(self, _request, response):
        # Placeholder for BT recovery compatibility in pure-DWA mode.
        return response

    def _execute_follow_path(self, goal_handle):
        with self._goal_lock:
            my_token = self._goal_token
        req = goal_handle.request
        self._last_plan = req.path
        self._plan_frame = req.path.header.frame_id if req.path.header.frame_id else "map"
        last_pose = req.path.poses[-1].pose.position
        gx, gy, ok = self._transform_xy(float(last_pose.x), float(last_pose.y), self._plan_frame, self._odom_frame)
        if not ok:
            self.get_logger().warn(
                f"Reject FollowPath goal: cannot transform final goal {self._plan_frame}->{self._odom_frame}."
            )
            self._last_plan = None
            self._final_goal_xy = None
            self.pub_cmd.publish(Twist())
            if goal_handle.is_active:
                goal_handle.abort()
            return FollowPath.Result()
        self._final_goal_xy = (gx, gy)
        now_t = time.time()
        if now_t - self._last_goal_accept_log_t > 1.0:
            self.get_logger().info(f"FollowPath accepted: {len(req.path.poses)} poses.")
            self._last_goal_accept_log_t = now_t

        feedback = FollowPath.Feedback()
        result = FollowPath.Result()
        while rclpy.ok():
            with self._goal_lock:
                if my_token != self._goal_token:
                    # 抢占不是客户端 cancel，避免非法状态迁移
                    if goal_handle.is_active:
                        goal_handle.abort()
                    return result
            if goal_handle.is_cancel_requested:
                self._last_plan = None
                self._final_goal_xy = None
                self.pub_cmd.publish(Twist())
                if goal_handle.is_active:
                    goal_handle.canceled()
                return result

            if self.planner.current_pose is not None and self._final_goal_xy is not None:
                dx = self._final_goal_xy[0] - float(self.planner.current_pose.position.x)
                dy = self._final_goal_xy[1] - float(self.planner.current_pose.position.y)
                dist = math.hypot(dx, dy)
                feedback.distance_to_goal = float(dist)
                feedback.speed = float(abs(self.planner.current_linear_speed))
                try:
                    goal_handle.publish_feedback(feedback)
                except Exception:
                    # Shutdown or preemption race
                    pass
                if dist <= self.goal_stop_tolerance:
                    self._last_plan = None
                    self._final_goal_xy = None
                    self.pub_cmd.publish(Twist())
                    if goal_handle.is_active:
                        goal_handle.succeed()
                    return result
            time.sleep(0.05)
        if rclpy.ok() and goal_handle.is_active:
            goal_handle.abort()
        return result

    def on_odom(self, msg: Odometry):
        if msg.header.frame_id:
            self._odom_frame = msg.header.frame_id
        self._publish_odom_tf(msg)
        self.planner.update_robot_state(msg)
        self._has_odom = True
        self._has_real_odom = True

    def on_model_states(self, msg: ModelStates):
        if self._has_real_odom:
            return
        if not msg.name:
            return

        idx = -1
        for candidate in self._robot_model_candidates:
            if candidate in msg.name:
                idx = msg.name.index(candidate)
                break
        if idx < 0:
            return

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose = msg.pose[idx]
        odom.twist.twist = msg.twist[idx]
        self._publish_odom_tf(odom)
        self.planner.update_robot_state(odom)
        self._has_odom = True
        self.pub_odom.publish(odom)

    def _try_tf_fallback(self) -> bool:
        """当 /odom 和 /gazebo/model_states 都无数据时，从 TF 获取 map->base_link 位姿"""
        for target_frame, source_frame in [("map", "base_link"), ("odom", "base_link")]:
            try:
                t = self.tf_buffer.lookup_transform(
                    target_frame, source_frame, rclpy.time.Time(), rclpy.duration.Duration(seconds=0.5)
                )
                odom = Odometry()
                odom.header.stamp = self.get_clock().now().to_msg()
                odom.header.frame_id = target_frame
                odom.child_frame_id = "base_link"
                odom.pose.pose.position.x = t.transform.translation.x
                odom.pose.pose.position.y = t.transform.translation.y
                odom.pose.pose.position.z = t.transform.translation.z
                odom.pose.pose.orientation = t.transform.rotation
                odom.twist.twist.linear.x = 0.0
                odom.twist.twist.linear.y = 0.0
                odom.twist.twist.angular.z = 0.0
                self.planner.update_robot_state(odom)
                return True
            except Exception:
                continue
        return False

    def _publish_odom_tf(self, odom: Odometry):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = odom.header.stamp
        tf_msg.header.frame_id = odom.header.frame_id if odom.header.frame_id else "odom"
        tf_msg.child_frame_id = odom.child_frame_id if odom.child_frame_id else "base_link"
        tf_msg.transform.translation.x = float(odom.pose.pose.position.x)
        tf_msg.transform.translation.y = float(odom.pose.pose.position.y)
        tf_msg.transform.translation.z = float(odom.pose.pose.position.z)
        tf_msg.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)

    def on_plan(self, msg: Path):
        self._last_plan = msg
        if msg.header.frame_id:
            self._plan_frame = msg.header.frame_id
        self._update_target_from_path()

    def on_scan(self, msg: LaserScan):
        self._last_scan_t = time.monotonic()
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        valid_min = max(float(msg.range_min), 0.03)
        valid = np.isfinite(ranges) & (ranges > valid_min) & (ranges < msg.range_max)
        angles = msg.angle_min + np.arange(ranges.shape[0], dtype=np.float32) * msg.angle_increment
        forward_sector = np.abs(angles) <= 0.70
        front_valid = valid & forward_sector
        # 用低分位替代绝对最小值，抑制单点噪声/自体回波导致的“提前刹停”
        if np.any(front_valid):
            self._scan_min_dist = float(np.percentile(ranges[front_valid], 10.0))
        elif np.any(valid):
            self._scan_min_dist = float(np.percentile(ranges[valid], 10.0))
        else:
            self._scan_min_dist = 10.0

        # 将激光点转换到世界坐标，作为 DWA 的 costmap 失效兜底障碍输入
        if self.planner.current_pose is not None and np.any(valid):
            # 适度下采样，降低计算负担
            idx = np.where(valid)[0][::4]
            if idx.size > 0:
                rr = ranges[idx]
                aa = angles[idx]
                # 过滤机器人近体回波，避免 obstacle_score 被“自体障碍”拉成不可行
                rr_mask = rr > self.self_filter_radius
                rr = rr[rr_mask]
                aa = aa[rr_mask]
            if idx.size > 0 and rr.size > 0:
                yaw = self.planner.quaternion_to_yaw(self.planner.current_pose.orientation)
                rx = float(self.planner.current_pose.position.x)
                ry = float(self.planner.current_pose.position.y)
                wx = rx + rr * np.cos(yaw + aa)
                wy = ry + rr * np.sin(yaw + aa)
                pts = np.stack((wx, wy), axis=1)
                self._scan_points_xy = pts
            else:
                self._scan_points_xy = np.empty((0, 2), dtype=np.float32)
        else:
            self._scan_points_xy = np.empty((0, 2), dtype=np.float32)
        ts = self._ros_time_to_float(msg.header.stamp) if msg.header.stamp else None
        self._push_obstacle_points_to_planner(timestamp=ts)

        dynamic_centroids = self.obstacle_predictor.get_dynamic_centroids(min_speed=0.05)
        trajectory_bands = self.obstacle_predictor.get_dynamic_trajectory_bands(
            min_speed=0.05,
            predict_time=self.planner.predict_time,
            dt_step=self.planner.control_interval,
        )
        bands_filtered = [
            self.trail_filter.filter_trail(band)
            for band in trajectory_bands
        ]
        if self.planner.current_pose is not None:
            rx = float(self.planner.current_pose.position.x)
            ry = float(self.planner.current_pose.position.y)
            yaw = self.planner.quaternion_to_yaw(self.planner.current_pose.orientation)
            filtered = filter_scan_exclude_dynamic(
                msg, dynamic_centroids, rx, ry, yaw, self.dynamic_exclude_radius,
                trajectory_bands=bands_filtered, region_size=0.2,
            )
        else:
            filtered = msg
        self.pub_scan_for_costmap.publish(filtered)

    def on_points(self, msg: PointCloud2):
        self._last_points_t = time.monotonic()
        ts = self._ros_time_to_float(msg.header.stamp) if msg.header.stamp else None
        if self.planner.current_pose is None:
            self._pc_points_xy = np.empty((0, 2), dtype=np.float32)
            self._push_obstacle_points_to_planner(timestamp=ts)
            return

        try:
            pts_iter = point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True
            )
            pts = np.asarray(list(pts_iter), dtype=np.float32)
        except Exception:
            pts = np.empty((0, 3), dtype=np.float32)

        if pts.size == 0:
            self._pc_points_xy = np.empty((0, 2), dtype=np.float32)
            self._push_obstacle_points_to_planner(timestamp=ts)
            return

        # 仅保留可能构成障碍的高度与距离，抑制地面和远点噪声
        x = pts[:, 0]
        y = pts[:, 1]
        z = pts[:, 2]
        r2 = x * x + y * y
        mask = (z > 0.05) & (z < 1.6) & (r2 > self.self_filter_radius * self.self_filter_radius) & (r2 < 6.0 * 6.0)
        pts = pts[mask]
        if pts.shape[0] == 0:
            self._pc_points_xy = np.empty((0, 2), dtype=np.float32)
            self._push_obstacle_points_to_planner(timestamp=ts)
            return

        # 下采样降低计算负担
        pts = pts[::6]
        rx = float(self.planner.current_pose.position.x)
        ry = float(self.planner.current_pose.position.y)
        yaw = self.planner.quaternion_to_yaw(self.planner.current_pose.orientation)
        c = math.cos(yaw)
        s = math.sin(yaw)
        px = pts[:, 0]
        py = pts[:, 1]
        wx = rx + c * px - s * py
        wy = ry + s * px + c * py
        self._pc_points_xy = np.stack((wx, wy), axis=1).astype(np.float32)
        self._push_obstacle_points_to_planner(timestamp=ts)

    def _push_obstacle_points_to_planner(self, timestamp: Optional[float] = None):
        """
        更新障碍点并推送给 planner 与预测器。动静分离：
        - 静态点 -> planner.update_scan_points（obstacle_score 用）
        - 全部点 -> obstacle_predictor（跟踪与预测，动态由 predictive_score 处理）
        动态质心附近的点从 scan_points 中排除，避免重复计入 obstacle_score。
        """
        now_mono = time.monotonic()
        scan_fresh = (now_mono - self._last_scan_t) <= self.scan_timeout_s
        points_fresh = (now_mono - self._last_points_t) <= self.scan_timeout_s

        scan_pts = self._scan_points_xy if scan_fresh else np.empty((0, 2), dtype=np.float32)
        pc_pts = self._pc_points_xy if points_fresh else np.empty((0, 2), dtype=np.float32)

        # 传感器超时后快速清空近障碍估计，避免“旧障碍”导致空旷区域误停。
        if not scan_fresh:
            self._scan_min_dist = 10.0

        if pc_pts.shape[0] > 0 and scan_pts.shape[0] > 0:
            pts = np.concatenate((pc_pts, scan_pts), axis=0)
        elif pc_pts.shape[0] > 0:
            pts = pc_pts
        else:
            pts = scan_pts

        if pts.shape[0] > 0:
            self.obstacle_predictor.update(
                pts, timestamp=timestamp if timestamp is not None else self._now_ros_time()
            )
            dynamic_centroids = self.obstacle_predictor.get_dynamic_centroids(min_speed=0.05)
            static_pts, _ = separate_dynamic_static_points(
                pts, dynamic_centroids, exclude_radius=self.dynamic_exclude_radius
            )
            # 缓存算法内部真正用于 DWA 的“静态障碍点”，GUI 直接可视化这一份
            self._last_static_pts = (
                static_pts.astype(np.float32) if static_pts is not None else np.empty((0, 2), dtype=np.float32)
            )
            self.planner.update_scan_points(static_pts)
        else:
            self._last_static_pts = np.empty((0, 2), dtype=np.float32)
            self.planner.update_scan_points(None)
            self.obstacle_predictor.update(
                None, timestamp=timestamp if timestamp is not None else self._now_ros_time()
            )

    def _ros_time_to_float(self, stamp) -> float:
        """将 ROS Time 转为 float 秒，用于预测器时间对准"""
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _now_ros_time(self) -> float:
        now_msg = self.get_clock().now().to_msg()
        return float(now_msg.sec) + float(now_msg.nanosec) * 1e-9

    def on_costmap(self, msg: OccupancyGrid):
        self._last_costmap_t = time.monotonic()
        h = msg.info.height
        w = msg.info.width
        if h == 0 or w == 0:
            return
        arr = np.asarray(msg.data, dtype=np.int16).reshape((h, w))
        origin = (float(msg.info.origin.position.x), float(msg.info.origin.position.y))
        self.planner.update_map(arr, float(msg.info.resolution), origin)

    def _update_target_from_path(self):
        """参考 exploration 的 find_local_target_on_path：先找前方最近点，再取 lookahead 外的点"""
        if self._last_plan is None or len(self._last_plan.poses) == 0:
            return
        if self.planner.current_pose is None:
            return

        rx = float(self.planner.current_pose.position.x)
        ry = float(self.planner.current_pose.position.y)
        robot_angle = self.planner.quaternion_to_yaw(self.planner.current_pose.orientation)

        pts = []
        tf_ok = True
        for p in self._last_plan.poses:
            x, y, ok = self._transform_xy(
                float(p.pose.position.x), float(p.pose.position.y), self._plan_frame, self._odom_frame
            )
            if not ok:
                tf_ok = False
                break
            pts.append((x, y))
        if not tf_ok:
            now_t = time.time()
            if now_t - self._last_frame_warn_t > 2.0:
                self.get_logger().warn(
                    f"Plan/odom frame transform unavailable: {self._plan_frame} -> {self._odom_frame}. "
                    "DWA target update skipped."
                )
                self._last_frame_warn_t = now_t
            return
        if len(pts) < 2:
            self.planner.set_target(pts[0][0], pts[0][1])
            return

        # 1. 找机器人前方 lookahead_angle_range 内最近的点
        closest_idx = -1
        min_dist_in_range = float('inf')
        for i, (px, py) in enumerate(pts):
            dist = math.hypot(px - rx, py - ry)
            pt_angle = math.atan2(py - ry, px - rx)
            angle_diff = abs(self.planner.normalize_angle(pt_angle - robot_angle))
            if angle_diff <= self.lookahead_angle_range and dist < min_dist_in_range:
                min_dist_in_range = dist
                closest_idx = i

        if closest_idx < 0:
            # 没有前方点时，不直接追终点；改为路径最近点，避免跨障碍“硬切”
            closest_idx = min(
                range(len(pts)),
                key=lambda i: math.hypot(pts[i][0] - rx, pts[i][1] - ry)
            )

        # 2. 使用固定前瞻，避免近障碍时目标点过近导致速度反复掉到 0
        effective_lookahead = self.lookahead_dist

        # 3. 从最近点沿路径向前，找第一个距离 >= lookahead_dist 的点
        for i in range(closest_idx, len(pts)):
            px, py = pts[i]
            dist = math.hypot(px - rx, py - ry)
            # 避免选择离机器人过近的局部目标点（会导致“原地犹豫”）
            if dist >= effective_lookahead and dist >= 0.45:
                self.planner.set_target(px, py)
                return

        # 4. 没有足够远的点，用路径终点
        tx, ty = pts[-1]
        self.planner.set_target(tx, ty)

    def _update_adaptive_weights(self, pred_count: int = 0):
        """按障碍距离 + 动态目标数量自适应：有人横穿时显著提高 predictive，抑制盲目前推。"""
        d = self._scan_min_dist
        has_dynamic = pred_count > 0
        if has_dynamic and d < 1.2:
            # 动态风险优先：降低速度偏好，提升前瞻项，给“绕行/减速”留空间
            self.planner.set_weights(direction=0.20, obstacle=0.24, velocity=0.18, predictive=0.38)
        elif has_dynamic:
            self.planner.set_weights(direction=0.23, obstacle=0.20, velocity=0.24, predictive=0.33)
        elif d < 0.45:
            self.planner.set_weights(direction=0.22, obstacle=0.38, velocity=0.20, predictive=0.20)
        elif d < 1.0:
            self.planner.set_weights(direction=0.26, obstacle=0.30, velocity=0.28, predictive=0.16)
        else:
            self.planner.set_weights(direction=0.28, obstacle=0.16, velocity=0.42, predictive=0.14)

    def _heading_error_to_target(self) -> Optional[float]:
        """机器人朝向与目标方向的偏差 (rad)，[-pi, pi]"""
        if self.planner.current_pose is None:
            return None
        dx = float(self.planner.target_x) - float(self.planner.current_pose.position.x)
        dy = float(self.planner.target_y) - float(self.planner.current_pose.position.y)
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return 0.0
        target_yaw = math.atan2(dy, dx)
        cur_yaw = self.planner.quaternion_to_yaw(self.planner.current_pose.orientation)
        return self.planner.normalize_angle(target_yaw - cur_yaw)

    def _distance_to_target(self) -> Optional[float]:
        if self.planner.current_pose is None:
            return None
        dx = float(self.planner.target_x) - float(self.planner.current_pose.position.x)
        dy = float(self.planner.target_y) - float(self.planner.current_pose.position.y)
        return math.hypot(dx, dy)

    def _distance_to_final_goal(self) -> Optional[float]:
        if self.planner.current_pose is None or self._final_goal_xy is None:
            return None
        dx = float(self._final_goal_xy[0]) - float(self.planner.current_pose.position.x)
        dy = float(self._final_goal_xy[1]) - float(self.planner.current_pose.position.y)
        return math.hypot(dx, dy)

    def _transform_xy(self, x: float, y: float, from_frame: str, to_frame: str) -> tuple[float, float, bool]:
        if self._alias_map_to_odom:
            f = from_frame.strip("/")
            t = to_frame.strip("/")
            if (f == "map" and t == "odom") or (f == "odom" and t == "map"):
                return x, y, True
        if from_frame == to_frame:
            return x, y, True
        try:
            t = self.tf_buffer.lookup_transform(
                to_frame, from_frame, rclpy.time.Time(), rclpy.duration.Duration(seconds=0.2)
            )
            tx = float(t.transform.translation.x)
            ty = float(t.transform.translation.y)
            yaw = self.planner.quaternion_to_yaw(t.transform.rotation)
            cx = math.cos(yaw)
            sx = math.sin(yaw)
            nx = tx + cx * x - sx * y
            ny = ty + sx * x + cx * y
            return nx, ny, True
        except Exception:
            return x, y, False

    def on_timer(self):
        if not self._has_odom:
            if not self._try_tf_fallback():
                now_t = time.monotonic()
                if now_t - self._last_diag_log_t > 0.5:
                    self.get_logger().info("[DWA_DIAG] return=no_odom")
                    self._last_diag_log_t = now_t
                return

        # 仅用上次动静分离结果刷新 planner 的障碍点；预测器只在 on_scan/on_points 里更新，
        # 避免定时器用同一份缓存点云+新时间戳反复 update，导致多帧回归速度被冲成 0、停下后无法跟踪动态目标。
        self.planner.update_scan_points(
            self._last_static_pts if self._last_static_pts.size > 0 else None
        )

        # costmap 过期时清空地图输入，避免历史障碍残留长期压低 obstacle_score。
        if (time.monotonic() - self._last_costmap_t) > self.costmap_timeout_s:
            self.planner.map = None

        if self._plan_frame != self._odom_frame and not self._alias_map_to_odom:
            now_t = time.time()
            if now_t - self._last_frame_warn_t > 2.0:
                self.get_logger().warn(
                    f"Frame mismatch detected: plan={self._plan_frame}, odom={self._odom_frame}. "
                    "Target points will be transformed before DWA scoring."
                )
                self._last_frame_warn_t = now_t

        self._update_target_from_path()

        # 无全局路径时不发速度，避免朝错误方向移动
        if self._last_plan is None or len(self._last_plan.poses) == 0:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.pub_cmd.publish(cmd)
            now_t = time.monotonic()
            if now_t - self._last_diag_log_t > 0.5:
                self.get_logger().info("[DWA_DIAG] return=no_plan")
                self._last_diag_log_t = now_t
            return

        # 到达最终 goal 时停止（不要用前瞻点，否则会“到点不停”）
        final_goal_dist = self._distance_to_final_goal()
        if final_goal_dist is not None and final_goal_dist <= self.goal_stop_tolerance:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.pub_cmd.publish(cmd)
            now_t = time.monotonic()
            if now_t - self._last_diag_log_t > 0.5:
                self.get_logger().info(
                    f"[DWA_DIAG] return=goal_reached final_goal_dist={final_goal_dist:.3f}"
                )
                self._last_diag_log_t = now_t
            return

        # 进展监控：2s 内位移过小则触发短时原地转向脱困
        now_t = time.monotonic()
        if self.planner.current_pose is not None:
            cur_xy = (
                float(self.planner.current_pose.position.x),
                float(self.planner.current_pose.position.y),
            )
            if self._progress_anchor_xy is None:
                self._progress_anchor_xy = cur_xy
                self._progress_anchor_t = now_t
            elif now_t - self._progress_anchor_t >= 2.0:
                moved = math.hypot(cur_xy[0] - self._progress_anchor_xy[0], cur_xy[1] - self._progress_anchor_xy[1])
                # 距终点仍较远且几乎无位移，判定为局部受困
                if (final_goal_dist is None or final_goal_dist > self.goal_stop_tolerance + 0.4) and moved < 0.05:
                    heading_err = self._heading_error_to_target()
                    self._recovery_turn_sign = 1.0 if (heading_err is None or heading_err >= 0.0) else -1.0
                    self._recovery_until_t = now_t + 1.2
                self._progress_anchor_xy = cur_xy
                self._progress_anchor_t = now_t

        if now_t < self._recovery_until_t:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.7 * self._recovery_turn_sign
            self.pub_cmd.publish(cmd)
            if now_t - self._last_diag_log_t > 0.5:
                self.get_logger().info(
                    f"[DWA_DIAG] return=recovery_spin w={cmd.angular.z:.3f} "
                    f"scan_min={self._scan_min_dist:.3f} "
                    f"goal_dist={final_goal_dist if final_goal_dist is not None else -1.0:.3f}"
                )
                self._last_diag_log_t = now_t
            return

        # 前瞻预测：以机器人为参考做量程过滤，避免 pred_cnt 暴增导致左右震荡
        range_origin = None
        if self.planner.current_pose is not None:
            range_origin = (
                float(self.planner.current_pose.position.x),
                float(self.planner.current_pose.position.y),
            )
        preds = self.obstacle_predictor.get_predictions(
            predict_time=self.planner.predict_time,
            dt_step=self.planner.control_interval,
            range_origin_xy=range_origin,
            max_range=8.0,
            max_preds=30,
        )
        preds = self.trail_filter.filter_predictions(preds)
        # 在更新 planner / 生成速度命令之前，将这一刻的预测结果和静态障碍点发布出去，
        # 供 GUI 节点直接展示“算法真实使用的动/静态障碍物”。
        try:
            dyn_msg = Float32MultiArray()
            dyn_data = []
            for p in preds:
                pos = p.get("position_xy", (0.0, 0.0))
                vel = p.get("velocity_xy", (0.0, 0.0))
                vx = float(vel[0])
                vy = float(vel[1])
                speed = math.hypot(vx, vy)
                dyn_data.extend([float(pos[0]), float(pos[1]), vx, vy, speed])
            dyn_msg.data = dyn_data
            self.pub_dynamic_info.publish(dyn_msg)

            static_msg = Float32MultiArray()
            if self._last_static_pts is not None and self._last_static_pts.size > 0:
                static_msg.data = self._last_static_pts.astype(np.float32).ravel().tolist()
            else:
                static_msg.data = []
            self.pub_static_info.publish(static_msg)
        except Exception:
            # 可视化信息发布失败不影响控制主循环
            pass

        self.planner.update_predicted_obstacles(preds)
        self._update_adaptive_weights(pred_count=len(preds))

        cmd = self.planner.generate_velocity_command()
        dbg = self.planner.last_debug if isinstance(self.planner.last_debug, dict) else {}

        # 反停滞：环境判定安全但 DWA 选到极小线速度时，给一个小前推避免“原地犹豫”
        anti_stall_applied = False
        status = str(dbg.get("status", "na"))
        safe_by_score = float(dbg.get("obs_max", 0.0)) >= 0.75
        low_v = abs(float(cmd.linear.x)) <= 0.03
        heading_not_too_large = abs(float(cmd.angular.z)) <= 0.45
        pred_safe = float(dbg.get("pred_min", 1.0)) > 0.55 and int(dbg.get("pred_cnt", 0)) <= 2
        if safe_by_score and pred_safe and low_v and heading_not_too_large and self._scan_min_dist > 0.75:
            cmd.linear.x = 0.22
            anti_stall_applied = True

        # no_feasible_rotate 下的温和兜底：
        # 触发后保持一个短时前推窗口，避免“这一帧推一下、下一帧又停住”的锯齿节奏。
        if status == "no_feasible_rotate" and self._scan_min_dist > 0.95:
            heading_err = self._heading_error_to_target()
            if heading_err is None or abs(heading_err) < 0.35:
                self._no_feasible_boost_until_t = now_t + 0.8
                cmd.linear.x = max(float(cmd.linear.x), 0.18)
                cmd.angular.z = float(np.clip(float(cmd.angular.z), -0.32, 0.32))
                anti_stall_applied = True
        elif now_t < self._no_feasible_boost_until_t and self._scan_min_dist > 0.90:
            cmd.linear.x = max(float(cmd.linear.x), 0.18)
            cmd.angular.z = float(np.clip(float(cmd.angular.z), -0.32, 0.32))
            anti_stall_applied = True

        # 安全且在前进模式下，避免输出过小线速度导致“在动但很慢”
        if (
            float(cmd.linear.x) > 0.0
            and abs(float(cmd.angular.z)) < 0.40
            and self._scan_min_dist > 0.90
            and float(cmd.linear.x) < self.min_cmd_linear_speed
        ):
            cmd.linear.x = self.min_cmd_linear_speed

        self.pub_cmd.publish(cmd)
        now_t = time.monotonic()
        if now_t - self._last_diag_log_t > 0.5:
            self.get_logger().info(
                "[DWA_DIAG] return=publish_cmd "
                f"scan_min={self._scan_min_dist:.3f} "
                f"goal_dist={final_goal_dist if final_goal_dist is not None else -1.0:.3f} "
                f"target=({self.planner.target_x:.2f},{self.planner.target_y:.2f}) "
                f"cmd=({float(cmd.linear.x):.3f},{float(cmd.angular.z):.3f}) "
                f"dwa_status={dbg.get('status', 'na')} "
                f"best_v={float(dbg.get('best_v', 0.0)):.3f} "
                f"best_w={float(dbg.get('best_w', 0.0)):.3f} "
                f"best_score={float(dbg.get('best_score', -1.0)):.3f} "
                f"obs_min={float(dbg.get('obs_min', -1.0)):.3f} "
                f"obs_max={float(dbg.get('obs_max', -1.0)):.3f} "
                f"pred_cnt={int(dbg.get('pred_cnt', 0))} "
                f"pred_min={float(dbg.get('pred_min', 1.0)):.3f} "
                f"pred_max={float(dbg.get('pred_max', 1.0)):.3f} "
                f"forced_min_progress={int(bool(dbg.get('forced_min_progress', False)))} "
                f"anti_stall={int(anti_stall_applied)}"
            )
            self._last_diag_log_t = now_t


def main():
    rclpy.init()
    node = SACDWANode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
