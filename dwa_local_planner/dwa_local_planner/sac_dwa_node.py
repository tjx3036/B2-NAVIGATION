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
from tf2_ros import TransformBroadcaster, Buffer, TransformListener

from dwa_local_planner.dwa_planner import DWAPlanner


class SACDWANode(Node):
    def __init__(self):
        super().__init__("sac_dwa_local_planner")
        # 控制环与动作服务分组，避免 timer 计算占满线程导致 action 超时
        self._ctrl_group = MutuallyExclusiveCallbackGroup()
        self._action_group = ReentrantCallbackGroup()
        self.planner = DWAPlanner()
        self.planner.set_debug(False)

        self.control_hz = 5.0           # 控制频率（进一步降负载，保证Action响应）
        self.planner.control_interval = 1.0 / self.control_hz

        # 参考 ros2-robot-navigation-exploration: lookahead 0.5, 前方角度范围
        self.lookahead_dist = 0.9       # 前瞻距离 (m)
        self.lookahead_angle_range = math.pi * 2 / 3  # 前方 120° 内选点（exploration 60°偏严，弯道易无点）
        self.emergency_stop_dist = 0.26
        self.min_cmd_linear_speed = 0.08
        self.goal_stop_tolerance = 0.25  # 到达判定
        self.goal_slow_dist = 1.0       # 该距离内减速
        self.slowing_factor = 2.0

        self._scan_min_dist = 10.0
        self._scan_points_xy = np.empty((0, 2), dtype=np.float32)
        self._pc_points_xy = np.empty((0, 2), dtype=np.float32)
        self.scan_timeout_s = 0.8
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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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
        if np.any(front_valid):
            self._scan_min_dist = float(np.min(ranges[front_valid]))
        elif np.any(valid):
            self._scan_min_dist = float(np.min(ranges[valid]))
        else:
            self._scan_min_dist = 10.0

        # 将激光点转换到世界坐标，作为 DWA 的 costmap 失效兜底障碍输入
        if self.planner.current_pose is not None and np.any(valid):
            # 适度下采样，降低计算负担
            idx = np.where(valid)[0][::4]
            if idx.size > 0:
                rr = ranges[idx]
                aa = angles[idx]
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
        self._push_obstacle_points_to_planner()

    def on_points(self, msg: PointCloud2):
        self._last_points_t = time.monotonic()
        if self.planner.current_pose is None:
            self._pc_points_xy = np.empty((0, 2), dtype=np.float32)
            self._push_obstacle_points_to_planner()
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
            self._push_obstacle_points_to_planner()
            return

        # 仅保留可能构成障碍的高度与距离，抑制地面和远点噪声
        x = pts[:, 0]
        y = pts[:, 1]
        z = pts[:, 2]
        r2 = x * x + y * y
        mask = (z > 0.05) & (z < 1.6) & (r2 > 0.05 * 0.05) & (r2 < 6.0 * 6.0)
        pts = pts[mask]
        if pts.shape[0] == 0:
            self._pc_points_xy = np.empty((0, 2), dtype=np.float32)
            self._push_obstacle_points_to_planner()
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
        self._push_obstacle_points_to_planner()

    def _push_obstacle_points_to_planner(self):
        if self._pc_points_xy.shape[0] > 0 and self._scan_points_xy.shape[0] > 0:
            pts = np.concatenate((self._pc_points_xy, self._scan_points_xy), axis=0)
        elif self._pc_points_xy.shape[0] > 0:
            pts = self._pc_points_xy
        else:
            pts = self._scan_points_xy

        if pts.shape[0] > 0:
            self.planner.update_scan_points(pts)
        else:
            self.planner.update_scan_points(None)

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

        # 2. 近障碍时缩短前瞻，优先先绕过眼前障碍再追远端目标
        effective_lookahead = self.lookahead_dist
        if self._scan_min_dist < 0.7:
            effective_lookahead = 0.50
        elif self._scan_min_dist < 1.0:
            effective_lookahead = 0.65

        # 3. 从最近点沿路径向前，找第一个距离 >= lookahead_dist 的点
        for i in range(closest_idx, len(pts)):
            px, py = pts[i]
            dist = math.hypot(px - rx, py - ry)
            if dist >= effective_lookahead:
                self.planner.set_target(px, py)
                return

        # 4. 没有足够远的点，用路径终点
        tx, ty = pts[-1]
        self.planner.set_target(tx, ty)

    def _update_adaptive_weights(self):
        """按障碍距离自适应：本算法仅一个障碍项，近时须提高 obstacle"""
        d = self._scan_min_dist
        if d < 0.45:
            self.planner.set_weights(direction=0.30, obstacle=0.52, velocity=0.18)   # 很近障碍，强避障
        elif d < 1.0:
            self.planner.set_weights(direction=0.34, obstacle=0.44, velocity=0.22)
        else:
            self.planner.set_weights(direction=0.40, obstacle=0.34, velocity=0.26)

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
                return

        if self._plan_frame != self._odom_frame:
            now_t = time.time()
            if now_t - self._last_frame_warn_t > 2.0:
                self.get_logger().warn(
                    f"Frame mismatch detected: plan={self._plan_frame}, odom={self._odom_frame}. "
                    "Target points will be transformed before DWA scoring."
                )
                self._last_frame_warn_t = now_t

        self._update_target_from_path()
        self._update_adaptive_weights()

        now_mono = time.monotonic()
        scan_fresh = (now_mono - self._last_scan_t) <= self.scan_timeout_s
        costmap_fresh = (now_mono - self._last_costmap_t) <= self.costmap_timeout_s
        if not scan_fresh or not costmap_fresh:
            cmd = Twist()
            cmd.linear.x = 0.0
            # 输入失效时仅允许很小角速度搜索，不允许盲目前进
            cmd.angular.z = 0.25 if self._last_plan is not None else 0.0
            self.pub_cmd.publish(cmd)
            return

        # 无全局路径时不发速度，避免朝错误方向移动
        if self._last_plan is None or len(self._last_plan.poses) == 0:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.pub_cmd.publish(cmd)
            return

        if self._scan_min_dist <= self.emergency_stop_dist:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.pub_cmd.publish(cmd)
            return

        # 到达最终 goal 时停止（不要用前瞻点，否则会“到点不停”）
        final_goal_dist = self._distance_to_final_goal()
        target_dist = self._distance_to_target()
        if final_goal_dist is not None and final_goal_dist <= self.goal_stop_tolerance:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.pub_cmd.publish(cmd)
            return

        cmd = self.planner.generate_velocity_command()
        heading_err = self._heading_error_to_target()

        # 接近目标减速（RotateToGoal 思路）
        if (
            final_goal_dist is not None
            and final_goal_dist < self.goal_slow_dist
            and final_goal_dist > self.goal_stop_tolerance
        ):
            scale = final_goal_dist / self.goal_slow_dist
            scale = max(1.0 / self.slowing_factor, scale)
            cmd.linear.x = float(cmd.linear.x) * scale

        # 大角度先转向（exploration 无此逻辑，但目标已限定前方，保留保险）
        if heading_err is not None:
            if abs(heading_err) > 2.4:    # 偏差极大时才原地转，避免“原地打圈”
                cmd.linear.x = 0.0
                if abs(float(cmd.angular.z)) < 0.5:
                    cmd.angular.z = 0.5 * (1.0 if heading_err > 0 else -1.0)
            elif abs(heading_err) > 1.2:
                # 大偏差时减速前进，允许边走边转
                cmd.linear.x = min(float(cmd.linear.x), 0.12)

        # 近障碍时优先限制前进速度，避免“继续顶着障碍走”
        if self._scan_min_dist < 0.45:
            cmd.linear.x = min(float(cmd.linear.x), 0.06)
        elif self._scan_min_dist < 0.70:
            cmd.linear.x = min(float(cmd.linear.x), 0.10)

        # 朝向基本对准时强制最小前进速度（仅在前方障碍较远时启用）
        if (
            final_goal_dist is not None
            and final_goal_dist > self.goal_stop_tolerance
            and self._scan_min_dist > 0.90
            and (heading_err is None or abs(heading_err) < 0.55)
            and abs(float(cmd.linear.x)) < self.min_cmd_linear_speed
        ):
            cmd.linear.x = self.min_cmd_linear_speed
        self.pub_cmd.publish(cmd)


def main():
    rclpy.init()
    node = SACDWANode()
    try:
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        executor.spin()
    finally:
        try:
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
