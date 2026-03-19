#!/usr/bin/env python3
"""
DWA (Dynamic Window Approach) 局部规划器

算法：方向 + 障碍物 + 速度 + 前瞻预测 四评分加权，动态窗口采样 (v,w)。
- direction: 轨迹末端朝向目标
- obstacle:  当前占据/膨胀（反应式绕行）
- velocity:  速度偏好
- predictive: 候选轨迹 vs 障碍未来轨迹的 TTC + 时空距离联合风险（前瞻式绕行）

参考 DWB 思路：长预测、障碍权重不宜过高、接近目标减速、大角度先转向。
作者：dong
版本：2.0
"""

import numpy as np
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class DWAPlanner:
    def __init__(self):
        """初始化DWA规划器

        三评分：direction(轨迹末端朝向目标)、obstacle(轨迹最近障碍距离)、velocity(v)。
        动态窗口用 control_interval 内的 accel/decel 约束采样范围。
        """
        # 机器人物理参数：显著提高速度与转向能力，并允许原地旋转
        self.max_linear_speed = 2.5     # 最大线速度 (m/s)
        self.min_linear_speed = 0.0     # 允许 v=0，用于原地转弯
        self.max_angular_speed = 3.0    # 最大角速度 (rad/s)，减小转弯半径
        self.min_angular_speed = -3.0
        self.max_linear_accel = 3.5     # 线加速度
        self.max_linear_decel = 3.0     # 线减速度
        self.max_angular_accel = 3.5    # 角加速度
        self.max_angular_decel = 3.5    # 角减速度

        # 控制与预测（control_interval 由 sac_dwa_node 按 control_hz 设置）
        self.control_interval = 0.05    # 默认 20Hz，与动态窗口 dt 一致
        # 加长 DWA 内部轨迹预测时长，配合动态障碍预测做更远距离前瞻
        self.predict_time = 2.5         # 轨迹预测时长 (s)

        # 采样：保留足够角度分辨率，避免算力过高
        self.velocity_samples = 6       # 线速度采样（进一步降算力）
        self.angular_samples = 20       # 角速度采样（提高大转角可行轨迹密度）

        # 权重：direction + obstacle + velocity + predictive 四评分
        self.weight_direction = 0.28    # 方向
        self.weight_obstacle = 0.18     # 障碍（反应式，当前占据/膨胀）
        self.weight_velocity = 0.40     # 速度（提升速度偏好）
        self.weight_predictive = 0.22   # 前瞻预测（TTC + 时空距离）

        # 前瞻预测输入：get_predictions() 返回的 list[dict]
        self.predicted_obstacles = []

        # 当前状态
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        self.current_pose = None

        # 地图信息
        self.map = None
        self.map_resolution = 0.05
        self.map_origin = (0.0, 0.0)
        self.cost_scale = 100.0
        # 半径分层：
        # - hard_collision_radius: 绝对碰撞半径（硬约束）
        # - soft_safety_radius:   软风险半径（评分衰减起点）
        self.hard_collision_radius = 0.24
        self.soft_safety_radius = 0.30
        self.robot_radius = self.hard_collision_radius
        self.scan_points = np.empty((0, 2), dtype=np.float32)

        # 目标点
        self.target_x = 0.0
        self.target_y = 0.0

        # 调试参数
        self.debug_enabled = False       # 默认关闭，由 sac_dwa_node 设置
        self.last_debug = {}

        # 角速度平滑：抑制左右频繁切换导致的 zigzag
        self._last_best_w = 0.0
        self._w_smoothing_score_margin = 0.03   # 分数差在此内时优先延续上一帧方向

    def set_weights(self, direction=None, obstacle=None, velocity=None, predictive=None):
        """动态设置DWA评分权重（用于SAC在线调参）"""
        if direction is not None:
            self.weight_direction = float(direction)
        if obstacle is not None:
            self.weight_obstacle = float(obstacle)
        if velocity is not None:
            self.weight_velocity = float(velocity)
        if predictive is not None:
            self.weight_predictive = float(predictive)

        # 归一化，避免权重尺度漂移
        total = (self.weight_direction + self.weight_obstacle +
                 self.weight_velocity + self.weight_predictive)
        if total > 1e-9:
            self.weight_direction /= total
            self.weight_obstacle /= total
            self.weight_velocity /= total
            self.weight_predictive /= total

    def update_predicted_obstacles(self, predictions):
        """更新前瞻预测目标（来自 predictor.get_predictions()）"""
        self.predicted_obstacles = list(predictions) if predictions else []

    def set_debug(self, enabled: bool):
        """设置调试开关"""
        self.debug_enabled = bool(enabled)

    # ============ 工具函数 ============

    def quaternion_to_yaw(self, q):
        """将四元数转换为 yaw 角"""
        qw, qx, qy, qz = q.w, q.x, q.y, q.z
        yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                         1.0 - 2.0 * (qy * qy + qz * qz))
        return yaw

    def get_robot_current_angle(self):
        """获取机器人当前角度 (yaw)"""
        if self.current_pose is None:
            return 0.0
        return self.quaternion_to_yaw(self.current_pose.orientation)

    def normalize_angle(self, angle):
        """将角度归一化到 [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def world_to_map(self, x, y):
        """将世界坐标转换为地图坐标"""
        map_x = int((x - self.map_origin[0]) / self.map_resolution)
        map_y = int((y - self.map_origin[1]) / self.map_resolution)
        return map_x, map_y

    def map_to_world(self, map_x, map_y):
        """将地图坐标转换为世界坐标"""
        world_x = self.map_origin[0] + map_x * self.map_resolution
        world_y = self.map_origin[1] + map_y * self.map_resolution
        return world_x, world_y

    def _normalize_cost(self, raw_cost):
        """统一代价值语义。local costmap 的 unknown(-1) 不直接按障碍处理。"""
        c = int(raw_cost)
        if c < 0:
            # rolling local costmap 中 unknown 很常见，按 free 处理可避免“远离障碍也停住”
            return 0
        return max(0, min(c, 254))

    # ============ 状态更新 ============

    def set_target(self, x, y):
        """设置目标点"""
        self.target_x = x
        self.target_y = y
        if self.debug_enabled:
            print(f"DWA目标点设置为: ({x:.2f}, {y:.2f})")

    def update_robot_state(self, odom_msg):
        """更新机器人状态"""
        self.current_pose = odom_msg.pose.pose
        self.current_linear_speed = odom_msg.twist.twist.linear.x
        self.current_angular_speed = odom_msg.twist.twist.angular.z
        if self.debug_enabled:
            print(f"机器人状态更新: 位置({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f}), "
                  f"线速度={self.current_linear_speed:.2f}m/s, 角速度={self.current_angular_speed:.2f}rad/s")

    def update_map(self, map_array, resolution, origin):
        """更新地图信息"""
        if resolution <= 0:
            return
        self.map = map_array
        self.map_resolution = resolution
        self.map_origin = origin
        # Nav2 costmap 常见为 0..100，也可能是 0..254；按当前数据自适应缩放
        try:
            max_val = int(np.max(map_array))
        except Exception:
            max_val = 100
        self.cost_scale = 254.0 if max_val > 100 else 100.0

    def update_scan_points(self, points):
        """更新由激光点云投影得到的世界坐标障碍点。"""
        if points is None:
            self.scan_points = np.empty((0, 2), dtype=np.float32)
            return
        arr = np.asarray(points, dtype=np.float32)
        if arr.ndim != 2 or arr.shape[1] != 2:
            self.scan_points = np.empty((0, 2), dtype=np.float32)
            return
        self.scan_points = arr

    # ============ 动态窗口计算 ============

    def compute_dynamic_window(self):
        """计算动态窗口 (DWB: acc_lim_x/theta, decel_lim_x/theta)"""
        v0 = self.current_linear_speed
        w0 = self.current_angular_speed
        dt = self.control_interval

        v_min = max(self.min_linear_speed, v0 - abs(self.max_linear_decel) * dt)
        v_max = min(self.max_linear_speed, v0 + self.max_linear_accel * dt)
        w_min = max(self.min_angular_speed, w0 - abs(self.max_angular_decel) * dt)
        w_max = min(self.max_angular_speed, w0 + self.max_angular_accel * dt)

        return v_min, v_max, w_min, w_max

    # ============ 轨迹预测 ============

    def predict_trajectory(self, v, w, predict_time):
        """预测轨迹"""
        if self.current_pose is None:
            return []

        trajectory = []
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        theta = self.get_robot_current_angle()

        dt = self.control_interval
        t = 0.0
        while t <= predict_time:
            trajectory.append((x, y))
            x += v * math.cos(theta) * dt
            y += v * math.sin(theta) * dt
            theta += w * dt
            t += dt
        return trajectory

    # ============ 轨迹评价 ============

    def calculate_direction_score(self, trajectory, v, w):
        """方向角评价: 使用标准DWA公式
        
        公式: score = 1 - |delta_theta| / pi
        其中:
        - delta_theta = theta_g - theta_r
        - theta_g = arctan2(yg - yr, xg - xr)  # 目标角度
        - theta_r = theta0 + w * t  # 机器人朝向角度
        - (xr, yr): 机器人预测轨迹末端点位置
        - (xg, yg): 全局路径上的目标点
        - theta0: 机器人当前位置朝向
        - w: 轨迹角速度
        - t: 轨迹预测时间
        """
        if not trajectory or self.current_pose is None:
            return 0.0

        # 获取机器人当前位置和朝向（theta0 为标准 yaw，不要 +pi 否则会原地转圈）
        xr, yr = self.current_pose.position.x, self.current_pose.position.y
        theta0 = self.quaternion_to_yaw(self.current_pose.orientation)
        
        # 计算轨迹末端点位置 (xr, yr) - 使用轨迹的最后一个点
        if len(trajectory) > 0:
            xr, yr = trajectory[-1]  # 轨迹末端点
        
        # 计算目标角度 theta_g = arctan2(yg - yr, xg - xr)
        theta_g = math.atan2(self.target_y - yr, self.target_x - xr)
        
        # 计算机器人朝向角度 theta_r：由候选角速度 w 积分到预测末端
        theta_r = theta0 + w * self.predict_time
        
        # 计算角度差 delta_theta = theta_g - theta_r
        delta_theta = theta_g - theta_r
        delta_theta = self.normalize_angle(delta_theta)  # 归一化到[-pi, pi]
        
        # 计算方向评分: score = 1 - |delta_theta| / pi
        direction_score = 1.0 - abs(delta_theta) / math.pi
        
        return direction_score

    def calculate_obstacle_score(self, trajectory):
        """障碍物距离评价: 使用标准DWA公式计算
        
        公式:
        score(d) = 0                    if d <= d_safe
        score(d) = (d - d_safe)/(d_max - d_safe)  if d_safe < d < d_max  
        score(d) = 1                    if d >= d_max
        """
        if not trajectory:
            return 0.0
        map_available = self.map is not None
        scan_available = self.scan_points.shape[0] > 0
        # 实时传感器优先：costmap 在动态场景下容易残留，导致“前方空旷也停住”。
        # 当 scan/pointcloud 可用时，障碍评分以实时点云为主；仅在无实时点时回退到 costmap。
        use_map_for_obstacle = map_available and not scan_available
        if not map_available and not scan_available:
            # 短时感知空窗不应把全部轨迹直接判死，否则会出现“空旷也走走停停”。
            # 真正硬碰撞仍由后续帧的 scan/map 与 is_collision_point() 约束。
            return 0.65

        # 仅保留“碰撞 + 最近障碍距离”两类核心判断
        min_distance = float('inf')
        found_obstacle = False

        # 下采样轨迹点评价，避免高频大负载
        sampled_traj = trajectory[::2] if len(trajectory) > 2 else trajectory
        for x, y in sampled_traj:
            # 轨迹任一点发生碰撞（按机器人半径邻域检查）直接判无效
            if self.is_collision_point(x, y):
                return 0.0

            d = None
            if scan_available:
                d = self.get_distance_to_obstacle_from_scan(x, y)
            elif use_map_for_obstacle:
                d_map, _ = self.get_distance_to_obstacle_from_map(x, y)
                d = d_map

            if d is not None:  # 找到了障碍物
                found_obstacle = True
                if d < min_distance:
                    min_distance = d

        # 如果整个轨迹路径上都没有找到障碍物，给最高分
        if not found_obstacle:
            return 1.0

        d_safe = self.soft_safety_radius   # 软风险半径（与硬碰撞半径分离）
        d_max = 1.5     # 超出视为安全 (m)

        # 近障碍段采用“软衰减”而非直接归零，避免几何可通过却被整体判死
        # 真实硬碰撞仍由 is_collision_point() + hard_collision_radius 负责。
        if min_distance <= d_safe:
            return 0.0  # 距离太近，评分为0
        elif min_distance >= d_max:
            return 1.0  # 距离足够远，评分为1
        else:
            score = (min_distance - d_safe) / (d_max - d_safe)
            return float(np.clip(score, 0.0, 1.0))

    def is_collision_point(self, x, y):
        """按机器人半径检查该点是否碰撞或进入不可接受高代价区域（当前占据/膨胀）。"""
        if self.map is None:
            d_scan = self.get_distance_to_obstacle_from_scan(x, y)
            if d_scan is None:
                return True
            return d_scan <= self.robot_radius
        map_x, map_y = self.world_to_map(x, y)
        if not (0 <= map_x < self.map.shape[1] and 0 <= map_y < self.map.shape[0]):
            # 轨迹点超出局部 costmap 边界时，优先回退到实时 scan 判断；
            # 避免 rolling window 边缘把整组轨迹误判为碰撞。
            d_scan = self.get_distance_to_obstacle_from_scan(x, y)
            if d_scan is not None:
                return d_scan <= self.robot_radius
            return False

        radius_cells = max(1, int(self.robot_radius / max(self.map_resolution, 1e-6)))
        # 仅将接近致命代价判为碰撞（100制地图下用 98，减少“边缘误碰撞”）
        lethal_cost = 100 if self.cost_scale <= 100.0 else 250

        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy > radius_cells * radius_cells:
                    continue
                cx = map_x + dx
                cy = map_y + dy
                if not (0 <= cx < self.map.shape[1] and 0 <= cy < self.map.shape[0]):
                    # 邻域越界不直接判碰撞，避免窗口边缘造成误停
                    continue
                c = self._normalize_cost(self.map[cy, cx])
                if c >= lethal_cost:
                    return True
        return False

    def get_distance_to_obstacle_from_scan(self, x, y):
        """使用激光/点云障碍点计算到障碍物的最近距离。obstacle 评分仅用当前占据（反应式）。"""
        pts = self.scan_points
        if pts.shape[0] == 0:
            return None
        dx = pts[:, 0] - float(x)
        dy = pts[:, 1] - float(y)
        d2 = dx * dx + dy * dy
        return float(math.sqrt(float(np.min(d2))))


    def get_distance_to_obstacle_from_map(self, x, y):
        """使用地图信息计算预测点到障碍物的距离"""
        if self.map is None:
            if self.debug_enabled:
                print("⚠️ DWA地图数据为空！")
            return None, None

        # 将世界坐标转换为地图坐标
        map_x, map_y = self.world_to_map(x, y)
        
        # 检查是否在地图范围内
        if not (0 <= map_x < self.map.shape[1] and 0 <= map_y < self.map.shape[0]):
            return None, None

        search_radius = max(1, min(int(0.5 / self.map_resolution), 12))  # 搜索半径0.5米，上限12格
        min_distance = float('inf')
        center_cost = self._normalize_cost(self.map[map_y, map_x])
        
        for dy in range(-search_radius, search_radius + 1):
            for dx in range(-search_radius, search_radius + 1):
                if dx*dx + dy*dy <= search_radius*search_radius:
                    check_x = map_x + dx
                    check_y = map_y + dy
                    
                    if (0 <= check_x < self.map.shape[1] and 
                        0 <= check_y < self.map.shape[0]):
                        
                        cost = self._normalize_cost(self.map[check_y, check_x])
                        # 仅把“真正障碍核”(lethal)用于距离计算；
                        # 膨胀层边缘不作为硬距离源，允许沿膨胀边缘运动
                        if cost >= 100:
                            distance = math.sqrt(dx*dx + dy*dy) * self.map_resolution
                            if distance < min_distance:
                                min_distance = distance
        
        if min_distance == float('inf'):
            return None, center_cost
        
        return min_distance, center_cost

    def calculate_speed_score(self, v):
        """速度评价: 使用 velocity(v,w) = |v| 公式计算并归一化"""
        if self.max_linear_speed <= 0:
            return 0.0
        velocity_score = abs(v)
        return velocity_score / self.max_linear_speed

    def calculate_predictive_score(self, trajectory, v, w):
        """
        前瞻预测评分：TTC + 时空距离联合风险。
        候选轨迹 vs 障碍未来轨迹的时空对准比较，低 TTC 或近距离 = 高风险 = 低分。
        并加入“前后通行”偏好：在侧向交汇时，优先选择从动态障碍物后方通过（让后）。
        """
        if not trajectory or not self.predicted_obstacles or self.current_pose is None:
            return 1.0  # 无预测目标时给满分

        theta0 = self.get_robot_current_angle()
        dt = self.control_interval
        d_safe = self.soft_safety_radius
        ttc_safe = 2.5   # TTC > 2.5s 视为安全（提前规避横穿）
        vo_horizon = min(3.0, max(1.2, self.predict_time))

        min_d = float('inf')
        min_ttc = float('inf')
        has_dynamic_risk = False
        ahead_risk_raw = 0.0   # 机器人位于动态障碍“前方”且接近时的风险
        behind_bonus_raw = 0.0 # 机器人位于动态障碍“后方”且接近时的奖励
        vo_hard_collision = False
        vo_risk_raw = 0.0

        for pred in self.predicted_obstacles:
            pos_xy = pred.get("position_xy", (0, 0))
            vel_xy = pred.get("velocity_xy", (0, 0))
            future_xy = pred.get("future_xy", [])
            vox, voy = vel_xy[0], vel_xy[1]
            obs_speed = math.hypot(vox, voy)
            # 预测评分只关注“明显动态”目标，避免与 obstacle_score 对静态障碍重复惩罚
            if obs_speed < 0.05:
                continue
            uox, uoy = vox / obs_speed, voy / obs_speed

            for i, (xr, yr) in enumerate(trajectory):
                t_i = i * dt
                theta_i = theta0 + w * t_i
                vr_x = v * math.cos(theta_i)
                vr_y = v * math.sin(theta_i)

                if i == 0:
                    xo, yo = pos_xy[0], pos_xy[1]
                elif i - 1 < len(future_xy):
                    xo, yo = future_xy[i - 1][0], future_xy[i - 1][1]
                else:
                    continue

                dx = xr - xo
                dy = yr - yo
                d = math.sqrt(dx * dx + dy * dy)
                if d < min_d:
                    min_d = d

                # RVO2/ORCA 思想（时序 VO 判别）：
                # 在每个轨迹时刻使用相对位姿/相对速度，检查剩余时间窗内是否进入碰撞圆
                obs_r = float(pred.get("inflation_radius", 0.25))
                R = self.robot_radius + max(0.15, obs_r)
                if d <= R:
                    vo_hard_collision = True
                    has_dynamic_risk = True
                else:
                    horizon_remain = vo_horizon - t_i
                    if horizon_remain > 0.0:
                        urx = vr_x - vox
                        ury = vr_y - voy
                        a = urx * urx + ury * ury
                        b = 2.0 * (dx * urx + dy * ury)
                        c = dx * dx + dy * dy - R * R
                        if a > 1e-8:
                            disc = b * b - 4.0 * a * c
                            if disc >= 0.0:
                                sqrt_disc = math.sqrt(disc)
                                t_enter = (-b - sqrt_disc) / (2.0 * a)
                                t_exit = (-b + sqrt_disc) / (2.0 * a)
                                if t_exit >= 0.0 and t_enter <= horizon_remain:
                                    ttc_vo = max(0.0, t_enter)
                                    vo_risk_raw = max(vo_risk_raw, 1.0 / (0.2 + ttc_vo))
                                    has_dynamic_risk = True
                                    if ttc_vo < 0.8:
                                        vo_hard_collision = True

                # “让前/让后”判据：
                # s_along > 0: 机器人在障碍速度方向前方（更容易迎面交汇/切前）
                # s_along < 0: 机器人在障碍后方（更安全，鼓励后绕）
                if d < 1.6:
                    s_along = dx * uox + dy * uoy
                    proximity = 1.0 / (d + 0.25)
                    if s_along > 0.0:
                        ahead_risk_raw = max(ahead_risk_raw, s_along * proximity)
                    else:
                        behind_bonus_raw = max(behind_bonus_raw, (-s_along) * proximity)

                if d < 1e-6:
                    min_ttc = 0.0
                    continue

                v_rel_x = vr_x - vox
                v_rel_y = vr_y - voy
                v_closing = -(dx * v_rel_x + dy * v_rel_y) / d
                if v_closing > 0.01:
                    has_dynamic_risk = True
                    ttc = d / v_closing
                    if ttc < min_ttc:
                        min_ttc = ttc

        if not has_dynamic_risk:
            return 1.0

        if vo_hard_collision:
            return 0.0

        if min_ttc == float('inf'):
            min_ttc = 10.0

        risk_ttc = 1.0 / (1.0 + min_ttc) if min_ttc < ttc_safe else 0.0
        risk_d = math.exp(-min_d / max(d_safe, 0.1)) if min_d < 1.8 else 0.0
        # 归一化“切前风险/后绕奖励”
        ahead_risk = math.tanh(ahead_risk_raw)
        behind_bonus = math.tanh(behind_bonus_raw)
        vo_risk = math.tanh(vo_risk_raw)

        # 横穿场景：VO/TTC 主导，同时显式惩罚“切前”，鼓励“后绕”
        risk = 0.42 * risk_ttc + 0.18 * risk_d + 0.15 * ahead_risk + 0.25 * vo_risk
        score = 1.0 - min(risk, 1.0) + 0.15 * behind_bonus
        return float(np.clip(score, 0.0, 1.0))

    # ============ 搜索最佳速度组合 ============

    def find_best_velocity_angle_combination(self):
        """寻找最佳速度角度组合 (v, w) - 使用跨轨迹归一化"""
        if self.current_pose is None:
            self.last_debug = {
                "status": "no_pose",
                "best_v": 0.0,
                "best_w": 0.0,
            }
            return 0.0, 0.0

        v_min, v_max, w_min, w_max = self.compute_dynamic_window()
        
        v_samples = np.linspace(v_min, v_max, self.velocity_samples)
        w_samples = np.linspace(w_min, w_max, self.angular_samples)

        trajectories_data = []
        direction_scores = []
        obstacle_scores = []
        velocity_scores = []
        predictive_scores = []

        for v in v_samples:
            for w in w_samples:
                traj = self.predict_trajectory(v, w, self.predict_time)

                direction_score = self.calculate_direction_score(traj, v, w)
                obstacle_score = self.calculate_obstacle_score(traj)
                velocity_score = self.calculate_speed_score(v)
                predictive_score = self.calculate_predictive_score(traj, v, w)

                trajectories_data.append((v, w, traj))
                direction_scores.append(direction_score)
                obstacle_scores.append(obstacle_score)
                velocity_scores.append(velocity_score)
                predictive_scores.append(predictive_score)

        best_score = float('-inf')
        best_v, best_w = 0.0, 0.0
        feasible_count = 0

        totals = np.zeros(len(trajectories_data), dtype=np.float64)
        for i, (v, w, traj) in enumerate(trajectories_data):
            totals[i] = (
                self.weight_direction * direction_scores[i] +
                self.weight_obstacle * obstacle_scores[i] +
                self.weight_velocity * velocity_scores[i] +
                self.weight_predictive * predictive_scores[i]
            )
            # 真实可行性：
            # - obstacle_score=0 仍视为不可行（接近硬碰撞）
            # - predictive_score=0 仅在存在明显前进速度时判不可行，允许原地转向先脱困
            pred_blocked = predictive_scores[i] <= 1e-6 and abs(v) > 0.06
            if obstacle_scores[i] <= 1e-6 or pred_blocked:
                totals[i] = -1e9
                continue
            if totals[i] > best_score:
                best_score, best_v, best_w = totals[i], v, w
            feasible_count += 1

        # 角速度平滑：若最优与上一帧方向相反且分数接近，优先延续上一帧方向
        if feasible_count > 0 and (best_w * self._last_best_w) < 0:
            for i, (v, w, _) in enumerate(trajectories_data):
                if (w * self._last_best_w) >= 0 and totals[i] >= best_score - self._w_smoothing_score_margin:
                    best_v, best_w = v, w
                    break
        self._last_best_w = best_w

        # 没有可行前进轨迹时，原地朝目标慢速转向
        if feasible_count == 0:
            if self.current_pose is None:
                self.last_debug = {
                    "status": "no_feasible_no_pose",
                    "best_v": 0.0,
                    "best_w": 0.0,
                }
                return 0.0, 0.0
            goal_heading = math.atan2(
                self.target_y - float(self.current_pose.position.y),
                self.target_x - float(self.current_pose.position.x),
            )
            cur_heading = self.quaternion_to_yaw(self.current_pose.orientation)
            err = self.normalize_angle(goal_heading - cur_heading)
            w = max(-0.7, min(0.7, 1.2 * err))
            self._last_best_w = w
            self.last_debug = {
                "status": "no_feasible_rotate",
                "best_v": 0.0,
                "best_w": float(w),
                "best_score": float("-inf"),
                "feasible_count": 0,
            }
            return 0.0, w

        # 不再强制最小前进，避免在局部受困时“顶着走不出去”
        forced_min_progress = False

        obs_arr = np.asarray(obstacle_scores, dtype=np.float32)
        pred_arr = np.asarray(predictive_scores, dtype=np.float32)
        totals = (
            self.weight_direction * np.asarray(direction_scores, dtype=np.float32) +
            self.weight_obstacle * np.asarray(obstacle_scores, dtype=np.float32) +
            self.weight_velocity * np.asarray(velocity_scores, dtype=np.float32) +
            self.weight_predictive * pred_arr
        )
        best_idx = int(np.argmax(totals))
        self.last_debug = {
            "status": "ok",
            "best_v": float(best_v),
            "best_w": float(best_w),
            "best_score": float(best_score),
            "feasible_count": int(feasible_count),
            "obs_min": float(np.min(obs_arr)) if obs_arr.size > 0 else 0.0,
            "obs_max": float(np.max(obs_arr)) if obs_arr.size > 0 else 0.0,
            "obs_best": float(obs_arr[best_idx]) if obs_arr.size > 0 else 0.0,
            "pred_cnt": len(self.predicted_obstacles),
            "pred_min": float(np.min(pred_arr)) if pred_arr.size > 0 else 1.0,
            "pred_max": float(np.max(pred_arr)) if pred_arr.size > 0 else 1.0,
            "pred_best": float(pred_arr[best_idx]) if pred_arr.size > 0 else 1.0,
            "forced_min_progress": bool(forced_min_progress),
        }

        return best_v, best_w


    def generate_velocity_command(self):
        """生成速度命令"""
        v, w = self.find_best_velocity_angle_combination()
        cmd_vel = Twist()
        cmd_vel.linear.x = v
        cmd_vel.angular.z = w
        return cmd_vel
