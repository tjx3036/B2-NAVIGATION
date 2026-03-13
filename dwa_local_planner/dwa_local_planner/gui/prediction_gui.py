#!/usr/bin/env python3
"""
基于 rclpy + matplotlib 的 GUI，可视化 SAC-DWA 使用的环境信息（不修改任何控制逻辑）：
- 动态障碍物的预测位置、速度方向/大小、数量
- 静态障碍物点云
- 机器人自身的位置、朝向、速度

只“拿数据”，不修改原有节点：
- 订阅已有话题：/odom, /scan, /velodyne_points
- 在 GUI 节点内部复用 ObstaclePredictor + TrailFilter 做动静分离和轨迹预测
"""

import math
import threading
import time
from typing import List, Tuple, Optional

import numpy as np
import rclpy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformListener

from dwa_local_planner.obstacle_predictor import ObstaclePredictor
from dwa_local_planner.trail_filter import TrailFilter, separate_dynamic_static_points


class PredictionGuiNode(Node):
    def __init__(self):
        super().__init__("dwa_prediction_gui")

        self._lock = threading.Lock()

        # 机器人状态
        self.robot_pos = np.array([0.0, 0.0], dtype=np.float32)
        self.robot_yaw = 0.0
        # 机体系线速度模长（用于文本显示）
        self.robot_lin_speed = 0.0
        self.robot_ang_speed = 0.0
        # 世界坐标系下的线速度向量 (vx, vy)，用于显示“实际移动方向”
        self.robot_vel_world = np.array([0.0, 0.0], dtype=np.float32)

        # GUI 用的障碍数据（默认来自 SAC-DWA 节点的 debug 话题，若缺失再本地推断）
        self.dynamic_obstacles: List[Tuple[float, float, float, float, float]] = []
        self.static_points: np.ndarray = np.empty((0, 2), dtype=np.float32)
        self._dynamic_from_node: List[Tuple[float, float, float, float, float]] = []
        self._static_from_node: np.ndarray = np.empty((0, 2), dtype=np.float32)

        # 时间戳
        self.last_dyn_stamp = 0.0
        self.last_static_stamp = 0.0
        self.last_odom_stamp = 0.0

        # 与 sac_dwa_node 保持一致的预测器配置（仅用于本地 fallback，不影响真实算法）
        self.control_hz = 8.0
        self.control_dt = 1.0 / self.control_hz
        self.predict_time = 1.4

        self.obstacle_predictor = ObstaclePredictor(
            cluster_eps=0.4,
            max_track_age=1.5,
            association_dist=0.5,
            predict_time=self.predict_time,
            predict_dt=self.control_dt,
        )
        self.trail_filter = TrailFilter(
            min_step_dist=0.15,
            inflation_radius=0.25,
            max_points_per_trail=15,
        )

        self.dynamic_exclude_radius = 0.6
        self.self_filter_radius = 0.30
        self.scan_timeout_s = 1.2

        # 来自 scan / 点云的世界坐标点
        self._scan_points_xy = np.empty((0, 2), dtype=np.float32)
        self._pc_points_xy = np.empty((0, 2), dtype=np.float32)
        self._last_scan_t = 0.0
        self._last_points_t = 0.0

        # 记录是否收到过 /odom；若一直没有，则优先使用 /gazebo/model_states / TF 作为机器人位姿来源
        self._has_odom = False
        self._robot_model_candidates = ["robot_model", "b2_gazebo", "b2"]

        # 通过 TF 获取机器人位姿的 world_frame / robot_frame，可通过参数重映射
        self.declare_parameter("world_frame", "odom")
        self.declare_parameter("robot_frame", "base_link")
        self.world_frame: str = str(self.get_parameter("world_frame").value)
        self.robot_frame: str = str(self.get_parameter("robot_frame").value)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ROS 订阅
        self.sub_odom = self.create_subscription(
            Odometry, "/odom", self.on_odom, 20
        )
        self.sub_model_states = self.create_subscription(
            ModelStates, "/gazebo/model_states", self.on_model_states, 20
        )
        self.sub_scan = self.create_subscription(
            LaserScan, "/scan", self.on_scan, 20
        )
        self.sub_points = self.create_subscription(
            PointCloud2, "/velodyne_points", self.on_points, 10
        )

        # 直接订阅 SAC-DWA 提供的 debug 动/静态障碍物话题，优先使用算法内部结果
        self.sub_dyn_info = self.create_subscription(
            Float32MultiArray, "/dwa_dynamic_obstacles_info", self.on_dynamic_info_from_node, 10
        )
        self.sub_static_info = self.create_subscription(
            Float32MultiArray, "/dwa_static_obstacles_info", self.on_static_info_from_node, 10
        )

        # 额外订阅 /scan_filtered：用于 GUI 展示“真正进入 costmap 的静态障碍”，
        # 这样 GUI 静态与膨胀行为保持一致。
        self.sub_scan_filtered = self.create_subscription(
            LaserScan, "/scan_filtered", self.on_scan_filtered, 10
        )

        self.get_logger().info(
            "DWA prediction GUI (pure listener) started. "
            "Subscribed to /odom, /gazebo/model_states, /scan, /velodyne_points, "
            "/dwa_dynamic_obstacles_info, /dwa_static_obstacles_info."
        )

    # ========= ROS 回调 =========

    def on_scan_filtered(self, msg: LaserScan):
        """从 /scan_filtered 提取静态障碍点，只用于 GUI 显示，保证与 costmap 膨胀一致。"""
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        valid_min = max(float(msg.range_min), 0.03)
        valid = np.isfinite(ranges) & (ranges > valid_min) & (ranges < msg.range_max)
        angles = msg.angle_min + np.arange(len(ranges), dtype=np.float32) * msg.angle_increment

        xs: List[float] = []
        ys: List[float] = []
        for i in np.where(valid)[0]:
            r = ranges[i]
            a = angles[i]
            xs.append(r * math.cos(a))
            ys.append(r * math.sin(a))

        if xs:
            pts = np.column_stack((xs, ys)).astype(np.float32)
        else:
            pts = np.empty((0, 2), dtype=np.float32)

        # 这里是激光坐标系下的点，但对于 GUI 来说，只看相对形状即可；
        # 更重要的是，它和 costmap 使用的是同一份 /scan_filtered 数据。
        with self._lock:
            self._static_from_node = pts
            self.last_static_stamp = time.monotonic()

    def on_odom(self, msg: Odometry):
        with self._lock:
            self._has_odom = True
            self.robot_pos[0] = float(msg.pose.pose.position.x)
            self.robot_pos[1] = float(msg.pose.pose.position.y)
            self.robot_yaw = self._quaternion_to_yaw(msg.pose.pose.orientation)
            # 机体系线速度
            vbx = float(msg.twist.twist.linear.x)
            vby = float(msg.twist.twist.linear.y)
            self.robot_lin_speed = math.hypot(vbx, vby)
            self.robot_ang_speed = float(msg.twist.twist.angular.z)
            # 转到世界系，得到“实际移动方向”
            c = math.cos(self.robot_yaw)
            s = math.sin(self.robot_yaw)
            vx_w = c * vbx - s * vby
            vy_w = s * vbx + c * vby
            self.robot_vel_world[0] = float(vx_w)
            self.robot_vel_world[1] = float(vy_w)
            self.last_odom_stamp = time.monotonic()

    def on_model_states(self, msg: ModelStates):
        """
        当没有外部 /odom 时，从 Gazebo 的 /gazebo/model_states 中推机器人位姿，
        这样 GUI 在纯仿真场景下也能看到机器人运动轨迹。
        """
        with self._lock:
            # 如果已经有真实 /odom，在 GUI 里优先使用 /odom，避免重复源混淆
            if self._has_odom:
                return
        if not msg.name:
            return

        # 找到机器人模型在 model_states 中的索引
        idx = -1
        for candidate in self._robot_model_candidates:
            if candidate in msg.name:
                idx = msg.name.index(candidate)
                break
        if idx < 0:
            return

        pose = msg.pose[idx]
        twist = msg.twist[idx]
        with self._lock:
            self.robot_pos[0] = float(pose.position.x)
            self.robot_pos[1] = float(pose.position.y)
            self.robot_yaw = self._quaternion_to_yaw(pose.orientation)
            vx_w = float(twist.linear.x)
            vy_w = float(twist.linear.y)
            self.robot_lin_speed = math.hypot(vx_w, vy_w)
            self.robot_ang_speed = float(twist.angular.z)
            self.robot_vel_world[0] = vx_w
            self.robot_vel_world[1] = vy_w
            self.last_odom_stamp = time.monotonic()

    # ========= 从 SAC-DWA 节点读取动/静态障碍物（算法真实使用的数据） =========

    def on_dynamic_info_from_node(self, msg: Float32MultiArray):
        data = list(msg.data)
        dyn_list: List[Tuple[float, float, float, float, float]] = []
        if len(data) >= 5:
            n = len(data) // 5
            for i in range(n):
                x, y, vx, vy, spd = data[5 * i : 5 * i + 5]
                dyn_list.append(
                    (float(x), float(y), float(vx), float(vy), float(spd))
                )
        with self._lock:
            self._dynamic_from_node = dyn_list
            self.last_dyn_stamp = time.monotonic()

    def on_static_info_from_node(self, msg: Float32MultiArray):
        data = list(msg.data)
        pts = np.empty((0, 2), dtype=np.float32)
        if len(data) >= 2:
            n = len(data) // 2
            arr = np.asarray(data[: 2 * n], dtype=np.float32).reshape((-1, 2))
            pts = arr
        with self._lock:
            self._static_from_node = pts
            self.last_static_stamp = time.monotonic()

    def get_robot_pose_tf(self) -> Optional[Tuple[float, float, float]]:
        """
        优先从 TF 获取机器人在 world_frame 下的位姿，
        这样 GUI 可以直接跟 TF 树对齐（例如 map->base_link）。
        """
        try:
            t = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.robot_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except Exception:
            return None

        q = t.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        x = t.transform.translation.x
        y = t.transform.translation.y
        return float(x), float(y), float(yaw)

    def on_scan(self, msg: LaserScan):
        now_mono = time.monotonic()
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        valid_min = max(float(msg.range_min), 0.03)
        valid = np.isfinite(ranges) & (ranges > valid_min) & (ranges < msg.range_max)
        angles = msg.angle_min + np.arange(ranges.shape[0], dtype=np.float32) * msg.angle_increment

        # 机器人当前位姿
        with self._lock:
            rx, ry, yaw = float(self.robot_pos[0]), float(self.robot_pos[1]), float(self.robot_yaw)

        if np.any(valid):
            idx = np.where(valid)[0][::4]
            if idx.size > 0:
                rr = ranges[idx]
                aa = angles[idx]
                mask = rr > self.self_filter_radius
                rr = rr[mask]
                aa = aa[mask]
            if idx.size > 0 and rr.size > 0:
                wx = rx + rr * np.cos(yaw + aa)
                wy = ry + rr * np.sin(yaw + aa)
                pts = np.stack((wx, wy), axis=1)
                with self._lock:
                    self._scan_points_xy = pts.astype(np.float32)
                    self._last_scan_t = now_mono
            else:
                with self._lock:
                    self._scan_points_xy = np.empty((0, 2), dtype=np.float32)
                    self._last_scan_t = now_mono
        else:
            with self._lock:
                self._scan_points_xy = np.empty((0, 2), dtype=np.float32)
                self._last_scan_t = now_mono

        self._update_obstacles()

    def on_points(self, msg: PointCloud2):
        now_mono = time.monotonic()
        with self._lock:
            if not np.isfinite(self.robot_pos).all():
                self._pc_points_xy = np.empty((0, 2), dtype=np.float32)
                self._last_points_t = now_mono
                return
            rx, ry, yaw = float(self.robot_pos[0]), float(self.robot_pos[1]), float(self.robot_yaw)

        try:
            pts_iter = point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True
            )
            pts = np.asarray(list(pts_iter), dtype=np.float32)
        except Exception:
            pts = np.empty((0, 3), dtype=np.float32)

        if pts.size == 0:
            with self._lock:
                self._pc_points_xy = np.empty((0, 2), dtype=np.float32)
                self._last_points_t = now_mono
            self._update_obstacles()
            return

        x = pts[:, 0]
        y = pts[:, 1]
        z = pts[:, 2]
        r2 = x * x + y * y
        mask = (z > 0.05) & (z < 1.6) & (r2 > self.self_filter_radius * self.self_filter_radius) & (r2 < 6.0 * 6.0)
        pts = pts[mask]
        if pts.shape[0] == 0:
            with self._lock:
                self._pc_points_xy = np.empty((0, 2), dtype=np.float32)
                self._last_points_t = now_mono
            self._update_obstacles()
            return

        pts = pts[::6]
        c = math.cos(yaw)
        s = math.sin(yaw)
        px = pts[:, 0]
        py = pts[:, 1]
        wx = rx + c * px - s * py
        wy = ry + s * px + c * py
        with self._lock:
            self._pc_points_xy = np.stack((wx, wy), axis=1).astype(np.float32)
            self._last_points_t = now_mono

        self._update_obstacles()

    # ========= 内部：动静分离与预测 =========

    def _update_obstacles(self):
        """将 scan + 点云合并，做动静分离，并更新 GUI 用的静态点和动态预测。"""
        now_mono = time.monotonic()

        with self._lock:
            scan_fresh = (now_mono - self._last_scan_t) <= self.scan_timeout_s
            points_fresh = (now_mono - self._last_points_t) <= self.scan_timeout_s

            scan_pts = self._scan_points_xy if scan_fresh else np.empty((0, 2), dtype=np.float32)
            pc_pts = self._pc_points_xy if points_fresh else np.empty((0, 2), dtype=np.float32)

            if pc_pts.shape[0] > 0 and scan_pts.shape[0] > 0:
                pts_all = np.concatenate((pc_pts, scan_pts), axis=0)
            elif pc_pts.shape[0] > 0:
                pts_all = pc_pts
            else:
                pts_all = scan_pts

        if pts_all.shape[0] == 0:
            # 更新预测器，让其内部时间前进
            self.obstacle_predictor.update(None, timestamp=self._now_ros_time())
            with self._lock:
                self.static_points = np.empty((0, 2), dtype=np.float32)
                self.dynamic_obstacles = []
                self.last_static_stamp = now_mono
                self.last_dyn_stamp = now_mono
            return

        # 更新预测器 & 动静分离
        self.obstacle_predictor.update(pts_all, timestamp=self._now_ros_time())
        dynamic_centroids = self.obstacle_predictor.get_dynamic_centroids(min_speed=0.05)
        static_pts, _ = separate_dynamic_static_points(
            pts_all, dynamic_centroids, exclude_radius=self.dynamic_exclude_radius
        )

        # GUI 用的静态点缓存
        with self._lock:
            self.static_points = (
                static_pts.astype(np.float32) if static_pts is not None else np.empty((0, 2), dtype=np.float32)
            )
            self.last_static_stamp = now_mono

        # 获取预测轨迹，与 sac_dwa_node 相同接口
        with self._lock:
            rx, ry = float(self.robot_pos[0]), float(self.robot_pos[1])
        preds = self.obstacle_predictor.get_predictions(
            predict_time=self.predict_time,
            dt_step=self.control_dt,
            range_origin_xy=(rx, ry),
            max_range=4.0,
            max_preds=30,
        )
        preds = self.trail_filter.filter_predictions(preds)

        dyn_list: List[Tuple[float, float, float, float, float]] = []
        for p in preds:
            pos = p.get("position_xy", (0.0, 0.0))
            vel = p.get("velocity_xy", (0.0, 0.0))
            vx = float(vel[0])
            vy = float(vel[1])
            speed = math.hypot(vx, vy)
            dyn_list.append((float(pos[0]), float(pos[1]), vx, vy, speed))

        with self._lock:
            self.dynamic_obstacles = dyn_list
            self.last_dyn_stamp = now_mono

    # ========= 对外查询接口 =========

    def snapshot(self):
        """返回当前状态快照，供绘图线程使用。"""
        with self._lock:
            # 优先使用 SAC-DWA 节点发布的动/静态障碍物（算法真实使用的结果）
            dyn = list(self._dynamic_from_node) if self._dynamic_from_node else list(self.dynamic_obstacles)
            static = (
                self._static_from_node.copy()
                if self._static_from_node is not None and self._static_from_node.size > 0
                else self.static_points.copy()
            )
            return (
                self.robot_pos.copy(),
                float(self.robot_yaw),
                float(self.robot_lin_speed),
                float(self.robot_ang_speed),
                self.robot_vel_world.copy(),
                dyn,
                static,
                float(self.last_odom_stamp),
                float(self.last_dyn_stamp),
                float(self.last_static_stamp),
            )

    @staticmethod
    def _quaternion_to_yaw(q) -> float:
        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        return math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )

    @staticmethod
    def _now_ros_time() -> float:
        # GUI 只需要单调时间对齐预测器，不需要精确的 ROS Time
        t = time.time()
        return float(t)


def run_matplotlib_gui(node: PredictionGuiNode):
    """在主线程中运行 matplotlib GUI 循环。"""
    import matplotlib.pyplot as plt

    plt.ion()
    fig, ax = plt.subplots(figsize=(6, 6))
    fig.canvas.manager.set_window_title("SAC-DWA 动态障碍物预测可视化（只读 GUI）")

    robot_scatter = ax.scatter([], [], c="blue", s=60, label="Robot")
    dyn_scatter = ax.scatter([], [], c="red", s=40, label="Dynamic Obstacles")
    static_scatter = ax.scatter([], [], c="gray", s=8, alpha=0.5, label="Static Obstacles")

    text_info = ax.text(
        0.02,
        0.98,
        "",
        transform=ax.transAxes,
        fontsize=9,
        verticalalignment="top",
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.7),
    )

    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, linestyle="--", alpha=0.3)
    ax.legend(loc="upper right")

    robot_vel_quiver = None
    dyn_quiver = None
    dyn_speed_texts = []

    while rclpy.ok():
        (
            robot_pos,
            robot_yaw,
            robot_lin_speed,
            robot_ang_speed,
            robot_vel_world,
            dyn_list,
            static_pts,
            odom_ts,
            dyn_ts,
            static_ts,
        ) = node.snapshot()

        now = time.monotonic()

        # 机器人：位置 + 朝向 + 实际移动方向
        pose_tf = node.get_robot_pose_tf()
        if pose_tf is not None:
            rx, ry, robot_yaw = pose_tf
        else:
            rx, ry = robot_pos[0], robot_pos[1]
        robot_scatter.set_offsets(np.array([[rx, ry]], dtype=np.float32))
        # 1) 机器人机体朝向（蓝色箭头）——长度加大，便于在大范围视图中看到
        heading_len = 2.0
        rvx = heading_len * math.cos(robot_yaw)
        rvy = heading_len * math.sin(robot_yaw)
        if robot_vel_quiver is not None:
            robot_vel_quiver.remove()
        robot_vel_quiver = ax.quiver(
            [rx],
            [ry],
            [rvx],
            [rvy],
            color="blue",
            scale=1,
            scale_units="xy",
            angles="xy",
        )

        # 2) 实际移动方向（世界系线速度）在当前版本的 GUI 中不再单独以绿色箭头显示，
        # 以避免与机体朝向混淆；如需恢复，只需重新绘制基于 robot_vel_world 的箭头。

        # 动态障碍物
        # 先清除上一次的速度文本
        for txt in dyn_speed_texts:
            txt.remove()
        dyn_speed_texts = []

        if dyn_list:
            dyn_xy = np.array([[d[0], d[1]] for d in dyn_list], dtype=np.float32)
            dyn_v = np.array([[d[2], d[3]] for d in dyn_list], dtype=np.float32)
            dyn_scatter.set_offsets(dyn_xy)
            if dyn_quiver is not None:
                dyn_quiver.remove()
            dyn_quiver = ax.quiver(
                dyn_xy[:, 0],
                dyn_xy[:, 1],
                dyn_v[:, 0],
                dyn_v[:, 1],
                color="red",
                scale=1,
                scale_units="xy",
                angles="xy",
            )
            # 为每个动态障碍物添加速度文本标注（字体稍大并略微偏移，避免被点遮挡）
            for (x, y, _, _, spd) in dyn_list:
                txt = ax.text(
                    x + 0.3,
                    y + 0.3,
                    f"{spd:.2f} m/s",
                    color="red",
                    fontsize=9,
                    ha="left",
                    va="bottom",
                )
                dyn_speed_texts.append(txt)
        else:
            dyn_scatter.set_offsets(np.empty((0, 2)))
            if dyn_quiver is not None:
                dyn_quiver.remove()
                dyn_quiver = None

        # 静态障碍
        if static_pts.size > 0:
            static_scatter.set_offsets(static_pts)
        else:
            static_scatter.set_offsets(np.empty((0, 2)))

        # 视野始终以机器人当前位置为中心，并进一步扩大范围
        view_radius = 30.0
        ax.set_xlim(float(rx) - view_radius, float(rx) + view_radius)
        ax.set_ylim(float(ry) - view_radius, float(ry) + view_radius)

        # 文本信息
        dyn_count = len(dyn_list)
        static_count = int(static_pts.shape[0])
        text = (
            f"Robot @ ({rx:.2f}, {ry:.2f}), yaw={math.degrees(robot_yaw):.1f}°\n"
            f"v={robot_lin_speed:.2f} m/s, w={robot_ang_speed:.2f} rad/s\n"
            f"Dynamic: {dyn_count}  (last update {now - dyn_ts:.2f}s ago)\n"
            f"Static points: {static_count}  (last update {now - static_ts:.2f}s ago)\n"
            f"Odom age: {now - odom_ts:.2f}s"
        )
        text_info.set_text(text)

        fig.canvas.draw_idle()
        plt.pause(0.05)

    plt.ioff()
    plt.close(fig)


def main(args=None):
    rclpy.init(args=args)
    node = PredictionGuiNode()

    # 在独立线程中运行 ROS2 事件循环，主线程负责 matplotlib GUI
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    def _spin():
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()

    try:
        run_matplotlib_gui(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == "__main__":
    main()

