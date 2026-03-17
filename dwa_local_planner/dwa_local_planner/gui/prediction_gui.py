#!/usr/bin/env python3
"""
前瞻避障 GUI（SAC-DWA 可视化）

实时展示 DWA 决策使用的关键数据：
- 机器人位姿（位置 + 朝向）
- 机器人速度（线速度 / 角速度）
- 动态障碍物当前位置 + 速度方向 + 速度大小
- 动态障碍物基于 (vx, vy) 的轨迹预测
- DWA 内部使用的静态障碍物点云

原则：GUI 只“照搬” sac_dwa_node 的决策输入，不做额外推理，
让“算法看到什么，界面就看到什么”。
"""

from __future__ import annotations

import math
import sys
import threading
from dataclasses import dataclass, field
from typing import List, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformListener

from PyQt5 import QtCore, QtGui, QtWidgets


@dataclass
class WorldState:
    """线程安全的导航世界状态快照。"""

    robot_x: float = 0.0
    robot_y: float = 0.0
    robot_yaw: float = 0.0
    robot_v: float = 0.0
    robot_w: float = 0.0

    # 动态障碍: list[(x, y, vx, vy, speed)]
    dynamic_obstacles: List[Tuple[float, float, float, float, float]] = field(
        default_factory=list
    )
    # 静态障碍: (N, 2) 数组，直接来自 sac_dwa_node._last_static_pts
    static_points: np.ndarray = field(
        default_factory=lambda: np.empty((0, 2), dtype=np.float32)
    )

    lock: threading.Lock = field(default_factory=threading.Lock, repr=False)


class DWAGUINode(Node):
    """ROS2 节点：订阅 sac_dwa_node 数据，并可从 TF 回退更新位姿。"""

    def __init__(self, world: WorldState):
        super().__init__("dwa_prediction_gui")
        self.world = world
        self._last_tf_ok = False
        self._tf_fail_count = 0

        # TF 缓存：当 /odom 缺失或不更新时，直接从 TF 推位姿（map/odom -> base_link）
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=20,
        )

        # NOTE:
        # 当前系统中没有稳定的 `/odom` 话题，直接订阅会导致机器人位姿
        # 时有时无。这里暂时只依赖 TF (odom/map -> base_link) 来更新 GUI
        # 中的机器人位姿，如需重新启用里程计，请在有真实 odom 话题时再恢复订阅。
        # self.sub_odom = self.create_subscription(
        #     Odometry, "/odom", self.on_odom, qos
        # )
        self.sub_cmd = self.create_subscription(
            Twist, "/cmd_vel", self.on_cmd_vel, qos
        )
        self.sub_dyn = self.create_subscription(
            Float32MultiArray,
            "/dwa_dynamic_obstacles_info",
            self.on_dynamic_info,
            qos,
        )
        self.sub_static = self.create_subscription(
            Float32MultiArray,
            "/dwa_static_obstacles_info",
            self.on_static_info,
            qos,
        )

        # 定时从 TF 更新一次机器人位姿（不阻塞 GUI，频率和 sac_dwa_node 类似）
        self.tf_timer = self.create_timer(0.1, self._update_pose_from_tf)

        self.get_logger().info(
            "DWA prediction GUI node started. "
            "Subscribing to /odom, /cmd_vel, /dwa_dynamic_obstacles_info, /dwa_static_obstacles_info, and TF."
        )

    # ---------------- ROS 回调 ----------------

    def on_odom(self, msg: Odometry) -> None:
        with self.world.lock:
            self.world.robot_x = float(msg.pose.pose.position.x)
            self.world.robot_y = float(msg.pose.pose.position.y)
            q = msg.pose.pose.orientation
            # 与 DWAPlanner.quaternion_to_yaw 保持一致
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            self.world.robot_yaw = yaw
            self.world.robot_v = float(msg.twist.twist.linear.x)
            self.world.robot_w = float(msg.twist.twist.angular.z)

    def on_cmd_vel(self, msg: Twist) -> None:
        # 如果希望看“算法输出速度”而不是里程计估计，可以用 /cmd_vel 覆盖掉 v / w。
        with self.world.lock:
            self.world.robot_v = float(msg.linear.x)
            self.world.robot_w = float(msg.angular.z)

    def _quat_to_yaw(self, q) -> float:
        """与 DWAPlanner.quaternion_to_yaw 一致的 yaw 计算。"""
        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

    def _update_pose_from_tf(self) -> None:
        """
        当 /odom 没数据或频率很低时，从 TF 里取 map/odom->base_link 的位姿做 GUI 显示。
        只影响可视化，不会改动 DWA 内部逻辑。
        """
        got_tf = False
        for target, source in [("odom", "base_link"), ("map", "base_link")]:
            try:
                t = self.tf_buffer.lookup_transform(
                    target, source, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.05)
                )
            except Exception as e:
                self._tf_fail_count += 1
                # 记录一下 TF 闪断情况（节流打印）
                if self._tf_fail_count % 20 == 1:
                    self.get_logger().warn(
                        f"TF lookup failed ({target}->{source}), count={self._tf_fail_count}: {e}"
                    )
                continue

            with self.world.lock:
                self.world.robot_x = float(t.transform.translation.x)
                self.world.robot_y = float(t.transform.translation.y)
                # 构造一个临时“pose”式四元数对象
                class _Q:
                    __slots__ = ("w", "x", "y", "z")

                    def __init__(self, qmsg):
                        self.w = float(qmsg.w)
                        self.x = float(qmsg.x)
                        self.y = float(qmsg.y)
                        self.z = float(qmsg.z)

                self.world.robot_yaw = self._quat_to_yaw(_Q(t.transform.rotation))
            got_tf = True
            break

        if got_tf:
            if not self._last_tf_ok:
                self.get_logger().info("TF pose stream OK, robot pose updating from TF.")
            self._last_tf_ok = True
        else:
            if self._last_tf_ok:
                self.get_logger().warn("TF pose stream lost, robot pose will stay at last value.")
            self._last_tf_ok = False

    def on_dynamic_info(self, msg: Float32MultiArray) -> None:
        data = list(msg.data)
        dyn_list: List[Tuple[float, float, float, float, float]] = []
        # sac_dwa_node.on_timer 中按 [x, y, vx, vy, speed] 打平
        for i in range(0, len(data), 5):
            if i + 4 >= len(data):
                break
            x = float(data[i + 0])
            y = float(data[i + 1])
            vx = float(data[i + 2])
            vy = float(data[i + 3])
            sp = float(data[i + 4])
            dyn_list.append((x, y, vx, vy, sp))
        with self.world.lock:
            self.world.dynamic_obstacles = dyn_list

    def on_static_info(self, msg: Float32MultiArray) -> None:
        data = np.asarray(msg.data, dtype=np.float32)
        if data.size == 0 or data.size % 2 != 0:
            pts = np.empty((0, 2), dtype=np.float32)
        else:
            pts = data.reshape((-1, 2))
        with self.world.lock:
            self.world.static_points = pts


class MapCanvas(QtWidgets.QWidget):
    """2D 画布：把世界坐标的决策数据实时画出来。"""

    def __init__(self, world: WorldState, parent: QtWidgets.QWidget | None = None):
        super().__init__(parent)
        self.world = world
        self.setMinimumSize(800, 800)

        # 显示范围（米），以正方形为例：[-view_size/2, +view_size/2]
        self.view_size = 20.0
        self.center_follow_robot = True

        # 最近一次合法的机器人位姿，用于过滤 NaN / Inf
        self._last_valid_pose: Tuple[float, float, float] | None = None

        # 周期刷新，保证“实时显示”
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self.update)
        self._timer.start(50)  # 20Hz 刷新

    # ----------- 坐标变换 -----------

    def world_to_view(self, wx: float, wy: float, cx: float, cy: float) -> Tuple[float, float]:
        """世界坐标 -> 窗口像素坐标，以 (cx, cy) 作为画面中心。"""
        size = float(min(self.width(), self.height()))
        if size <= 1.0:
            return 0.0, 0.0
        scale = size / self.view_size
        vx = size / 2.0 + (wx - cx) * scale
        vy = size / 2.0 - (wy - cy) * scale  # y 轴向上
        return vx, vy

    # ----------- 绘制入口 -----------

    def paintEvent(self, event: QtGui.QPaintEvent) -> None:
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        rect = self.rect()
        painter.fillRect(rect, QtGui.QColor(30, 30, 30))

        with self.world.lock:
            rx = self.world.robot_x
            ry = self.world.robot_y
            yaw = self.world.robot_yaw
            v = self.world.robot_v
            w = self.world.robot_w
            dyn = list(self.world.dynamic_obstacles)
            static_pts = np.copy(self.world.static_points)

        # 过滤非法位姿，避免出现 NaN / Inf 导致整帧机器人不画出来
        draw_robot = True
        if any(math.isnan(v) or math.isinf(v) for v in (rx, ry, yaw)):
            if self._last_valid_pose is not None:
                print("[MapCanvas] invalid robot pose (NaN/Inf), fallback to last valid pose")
                rx, ry, yaw = self._last_valid_pose
            else:
                print("[MapCanvas] invalid robot pose (NaN/Inf) and no last valid pose, skip drawing robot")
                draw_robot = False
        else:
            self._last_valid_pose = (rx, ry, yaw)

        # 视图中心：跟随机器人
        cx, cy = (rx, ry) if self.center_follow_robot else (0.0, 0.0)

        self._draw_grid_and_axes(painter, cx, cy)
        self._draw_static_obstacles(painter, static_pts, cx, cy)
        self._draw_dynamic_obstacles(painter, dyn, cx, cy)
        if draw_robot:
            self._draw_robot(painter, rx, ry, yaw, v, w, cx, cy)
        self._draw_hud(painter, v, w)

    # ----------- 各部分绘制 -----------

    def _draw_grid_and_axes(self, painter: QtGui.QPainter, cx: float, cy: float) -> None:
        size = float(min(self.width(), self.height()))
        if size <= 1.0:
            return
        half = self.view_size / 2.0
        step = 1.0

        # 网格（使用 QPointF 以避免 float/int 重载冲突）
        painter.setPen(QtGui.QPen(QtGui.QColor(55, 55, 55), 1))
        g = -half
        while g <= half + 1e-6:
            x1, y1 = self.world_to_view(-half + cx, g + cy, cx, cy)
            x2, y2 = self.world_to_view(half + cx, g + cy, cx, cy)
            painter.drawLine(QtCore.QPointF(x1, y1), QtCore.QPointF(x2, y2))

            x1, y1 = self.world_to_view(g + cx, -half + cy, cx, cy)
            x2, y2 = self.world_to_view(g + cx, half + cy, cx, cy)
            painter.drawLine(QtCore.QPointF(x1, y1), QtCore.QPointF(x2, y2))
            g += step

        # 原点标记（同样使用 QPointF）
        ox, oy = self.world_to_view(0.0, 0.0, cx, cy)
        painter.setPen(QtGui.QPen(QtGui.QColor(100, 100, 100), 1.5))
        painter.drawLine(
            QtCore.QPointF(ox - 10.0, oy),
            QtCore.QPointF(ox + 10.0, oy),
        )
        painter.drawLine(
            QtCore.QPointF(ox, oy - 10.0),
            QtCore.QPointF(ox, oy + 10.0),
        )

    def _draw_static_obstacles(
        self,
        painter: QtGui.QPainter,
        static_pts: np.ndarray,
        cx: float,
        cy: float,
    ) -> None:
        if static_pts.shape[0] == 0:
            return
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QColor(150, 150, 150, 200))
        for x, y in static_pts:
            vx, vy = self.world_to_view(float(x), float(y), cx, cy)
            painter.drawEllipse(QtCore.QPointF(vx, vy), 2.0, 2.0)

    def _draw_dynamic_obstacles(
        self,
        painter: QtGui.QPainter,
        dyn: List[Tuple[float, float, float, float, float]],
        cx: float,
        cy: float,
    ) -> None:
        for x, y, vx, vy, sp in dyn:
            px, py = self.world_to_view(x, y, cx, cy)

            # 当前位置
            painter.setPen(QtCore.Qt.NoPen)
            painter.setBrush(QtGui.QColor(0, 200, 255))
            painter.drawEllipse(QtCore.QPointF(px, py), 4.0, 4.0)

            # 速度箭头（方向 + 大小）
            v_scale = 0.8  # 仅用于可视化缩放
            ex = x + vx * v_scale
            ey = y + vy * v_scale
            ex_v, ey_v = self.world_to_view(ex, ey, cx, cy)
            painter.setPen(QtGui.QPen(QtGui.QColor(0, 200, 255), 2))
            painter.drawLine(QtCore.QPointF(px, py), QtCore.QPointF(ex_v, ey_v))
            self._draw_arrow_head(painter, px, py, ex_v, ey_v, QtGui.QColor(0, 200, 255))

            # 利用 (vx, vy) 做简单轨迹预测线：和 DWA 用的一致的物理量
            painter.setPen(QtGui.QPen(QtGui.QColor(0, 255, 160, 170), 1.5))
            last_px, last_py = px, py
            horizon = 4.0  # s，拉长预测距离
            dt = 0.25
            t = dt
            while t <= horizon + 1e-6:
                fx = x + vx * t
                fy = y + vy * t
                fx_v, fy_v = self.world_to_view(fx, fy, cx, cy)
                painter.drawLine(
                    QtCore.QPointF(last_px, last_py),
                    QtCore.QPointF(fx_v, fy_v),
                )
                last_px, last_py = fx_v, fy_v
                t += dt

            # 速度大小文字
            painter.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255)))
            painter.setFont(QtGui.QFont("Arial", 8))
            painter.drawText(int(px + 5), int(py - 5), f"{sp:.2f} m/s")

    def _draw_robot(
        self,
        painter: QtGui.QPainter,
        rx: float,
        ry: float,
        yaw: float,
        v: float,
        w: float,
        cx: float,
        cy: float,
    ) -> None:
        # 机器人用三角形表示（与朝向一致）
        base_r = 0.25
        pts_world = [
            (rx + base_r * math.cos(yaw), ry + base_r * math.sin(yaw)),
            (rx + base_r * math.cos(yaw + 2.5), ry + base_r * math.sin(yaw + 2.5)),
            (rx + base_r * math.cos(yaw - 2.5), ry + base_r * math.sin(yaw - 2.5)),
        ]
        pts_view = [
            QtCore.QPointF(*self.world_to_view(px, py, cx, cy)) for (px, py) in pts_world
        ]

        painter.setPen(QtGui.QPen(QtGui.QColor(255, 255, 0), 2))
        painter.setBrush(QtGui.QColor(255, 255, 0, 80))
        painter.drawPolygon(QtGui.QPolygonF(pts_view))

        # 线速度箭头（从机器人中心出发）
        sx, sy = self.world_to_view(rx, ry, cx, cy)
        v_scale = 1.0
        vx = v * math.cos(yaw) * v_scale
        vy = v * math.sin(yaw) * v_scale
        ex, ey = self.world_to_view(rx + vx, ry + vy, cx, cy)
        painter.setPen(QtGui.QPen(QtGui.QColor(0, 255, 0), 2))
        painter.drawLine(QtCore.QPointF(sx, sy), QtCore.QPointF(ex, ey))
        self._draw_arrow_head(painter, sx, sy, ex, ey, QtGui.QColor(0, 255, 0))

        # 线速度数值
        painter.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255)))
        painter.setFont(QtGui.QFont("Arial", 8))
        painter.drawText(int(ex + 5), int(ey), f"{v:.2f} m/s")

    def _draw_hud(self, painter: QtGui.QPainter, v: float, w: float) -> None:
        # 左上角显示机器人当前线速度 / 角速度
        painter.setPen(QtGui.QPen(QtGui.QColor(255, 255, 255)))
        painter.setFont(QtGui.QFont("Arial", 10))
        text = f"Robot v: {v:.3f} m/s   w: {w:.3f} rad/s"
        painter.drawText(10, 20, text)

    # ----------- 小工具：箭头箭头头部 -----------

    @staticmethod
    def _draw_arrow_head(
        painter: QtGui.QPainter,
        x1: float,
        y1: float,
        x2: float,
        y2: float,
        color: QtGui.QColor,
    ) -> None:
        dx = x2 - x1
        dy = y2 - y1
        length = math.hypot(dx, dy)
        if length < 1e-3:
            return
        ux = dx / length
        uy = dy / length
        size = 8.0
        left_x = x2 - ux * size - uy * size * 0.5
        left_y = y2 - uy * size + ux * size * 0.5
        right_x = x2 - ux * size + uy * size * 0.5
        right_y = y2 - uy * size - ux * size * 0.5

        painter.setBrush(color)
        painter.setPen(QtGui.QPen(color, 1))
        poly = QtGui.QPolygonF(
            [
                QtCore.QPointF(x2, y2),
                QtCore.QPointF(left_x, left_y),
                QtCore.QPointF(right_x, right_y),
            ]
        )
        painter.drawPolygon(poly)


class MainWindow(QtWidgets.QMainWindow):
    """主窗口：一个画布 + 可以扩展其它信息面板。"""

    def __init__(self, world: WorldState):
        super().__init__()
        self.setWindowTitle("SAC-DWA Prediction GUI")
        self.canvas = MapCanvas(world)
        self.setCentralWidget(self.canvas)
        self.resize(900, 900)


def main() -> None:
    """
    入口：
    - 启动 rclpy + Qt 事件循环
    - 保证 GUI 帧率足够高，实现“实时显示”
    """
    rclpy.init()

    world = WorldState()
    node = DWAGUINode(world)

    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow(world)
    window.show()

    # 用定时器驱动 rclpy.spin_once，避免阻塞 Qt 事件循环
    def spin_once():
        rclpy.spin_once(node, timeout_sec=0.0)

    ros_timer = QtCore.QTimer()
    ros_timer.timeout.connect(spin_once)
    ros_timer.start(10)  # 100Hz 调度一次 ROS 回调

    try:
        exit_code = app.exec_()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()

