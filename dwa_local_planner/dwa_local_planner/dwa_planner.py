#!/usr/bin/env python3
"""
DWA (Dynamic Window Approach) 局部规划器

算法：方向 + 障碍物 + 速度 三评分加权，动态窗口采样 (v,w)。
参考 DWB 思路：长预测、障碍权重不宜过高、接近目标减速、大角度先转向。
参数按本算法实际结构（仅有单一障碍项、差分驱动机理）调优。

作者：dong
版本：1.0
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
        # 机器人物理参数：先保证“能动得起来”，再做细调
        self.max_linear_speed = 0.8     # 最大线速度
        self.min_linear_speed = 0.0     # 最小线速度
        self.max_angular_speed = 1.2    # 最大角速度
        self.min_angular_speed = -1.2   # 最小角速度
        self.max_linear_accel = 1.0     # 线加速度
        self.max_linear_decel = 1.0     # 线减速度
        self.max_angular_accel = 2.0    # 角加速度
        self.max_angular_decel = 2.0    # 角减速度

        # 控制与预测（control_interval 由 sac_dwa_node 按 control_hz 设置）
        self.control_interval = 0.05    # 默认 20Hz，与动态窗口 dt 一致
        self.predict_time = 1.0         # 轨迹预测时长 (s)，降低计算负载

        # 采样：保留足够角度分辨率，避免算力过高
        self.velocity_samples = 6       # 线速度采样（进一步降算力）
        self.angular_samples = 8        # 角速度采样（进一步降算力）

        # 权重：提升速度权重，避免慢速挪动
        self.weight_direction = 0.40    # 方向
        self.weight_obstacle = 0.25     # 障碍
        self.weight_velocity = 0.35     # 速度

        # 当前状态
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        self.current_pose = None

        # 地图信息
        self.map = None
        self.map_resolution = 0.05
        self.map_origin = (0.0, 0.0)

        # 目标点
        self.target_x = 0.0
        self.target_y = 0.0

        # 调试参数
        self.debug_enabled = False       # 默认关闭，由 sac_dwa_node 设置

    def set_weights(self, direction=None, obstacle=None, velocity=None):
        """动态设置DWA评分权重（用于SAC在线调参）"""
        if direction is not None:
            self.weight_direction = float(direction)
        if obstacle is not None:
            self.weight_obstacle = float(obstacle)
        if velocity is not None:
            self.weight_velocity = float(velocity)

        # 归一化，避免权重尺度漂移
        total = self.weight_direction + self.weight_obstacle + self.weight_velocity
        if total > 1e-9:
            self.weight_direction /= total
            self.weight_obstacle /= total
            self.weight_velocity /= total

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
        if self.map is None:
            # If costmap is temporarily unavailable, avoid suppressing all trajectories.
            return 1.0

        # 计算轨迹上每个点到障碍物的最小距离
        min_distance = float('inf')
        found_obstacle = False
        
        # 下采样轨迹点评价，避免高频大负载
        sampled_traj = trajectory[::2] if len(trajectory) > 2 else trajectory
        for x, y in sampled_traj:
            d = self.get_distance_to_obstacle_from_map(x, y)
            if d is not None:  # 找到了障碍物
                found_obstacle = True
                if d < min_distance:
                    min_distance = d

        # 如果整个轨迹路径上都没有找到障碍物，给最高分
        if not found_obstacle:
            return 1.0  # 没有障碍物时给最高分

        d_safe = 0.22   # 安全距离 (m)
        d_max = 1.5     # 超出视为安全 (m)
        
        if min_distance <= d_safe:
            return 0.0  # 距离太近，评分为0
        elif min_distance >= d_max:
            return 1.0  # 距离足够远，评分为1
        else:
            # 线性插值计算评分
            score = (min_distance - d_safe) / (d_max - d_safe)
            return score


    def get_distance_to_obstacle_from_map(self, x, y):
        """使用地图信息计算预测点到障碍物的距离"""
        if self.map is None:
            if self.debug_enabled:
                print("⚠️ DWA地图数据为空！")
            return None

        # 将世界坐标转换为地图坐标
        map_x, map_y = self.world_to_map(x, y)
        
        # 检查是否在地图范围内
        if not (0 <= map_x < self.map.shape[1] and 0 <= map_y < self.map.shape[0]):
            return None

        search_radius = max(1, min(int(0.5 / self.map_resolution), 12))  # 搜索半径0.5米，上限12格
        min_distance = float('inf')
        
        for dy in range(-search_radius, search_radius + 1):
            for dx in range(-search_radius, search_radius + 1):
                if dx*dx + dy*dy <= search_radius*search_radius:
                    check_x = map_x + dx
                    check_y = map_y + dy
                    
                    if (0 <= check_x < self.map.shape[1] and 
                        0 <= check_y < self.map.shape[0]):
                        
                        cost = int(self.map[check_y, check_x])
                        if cost >= 99:  # 100=OccupancyGrid障碍物, 254=Nav2 costmap致命障碍
                            distance = math.sqrt(dx*dx + dy*dy) * self.map_resolution
                            if distance < min_distance:
                                min_distance = distance
        
        if min_distance == float('inf'):
            return None
        
        return min_distance

    def calculate_speed_score(self, v):
        """速度评价: 使用 velocity(v,w) = |v| 公式计算并归一化"""
        if self.max_linear_speed <= 0:
            return 0.0
        velocity_score = abs(v)
        return velocity_score / self.max_linear_speed


    # ============ 搜索最佳速度组合 ============

    def find_best_velocity_angle_combination(self):
        """寻找最佳速度角度组合 (v, w) - 使用跨轨迹归一化"""
        if self.current_pose is None:
            return 0.0, 0.0

        v_min, v_max, w_min, w_max = self.compute_dynamic_window()
        
        v_samples = np.linspace(v_min, v_max, self.velocity_samples)
        w_samples = np.linspace(w_min, w_max, self.angular_samples)

        trajectories_data = []
        direction_scores = []
        obstacle_scores = []
        velocity_scores = []

        for v in v_samples:
            for w in w_samples:
                traj = self.predict_trajectory(v, w, self.predict_time)
                
                direction_score = self.calculate_direction_score(traj, v, w)
                obstacle_score = self.calculate_obstacle_score(traj)
                velocity_score = self.calculate_speed_score(v)
                
                trajectories_data.append((v, w, traj))
                direction_scores.append(direction_score)
                obstacle_scores.append(obstacle_score)
                velocity_scores.append(velocity_score)

        best_score = float('-inf')
        best_v, best_w = 0.0, 0.0

        for i, (v, w, traj) in enumerate(trajectories_data):
            total_score = (self.weight_direction * direction_scores[i] + 
                          self.weight_obstacle * obstacle_scores[i] + 
                          self.weight_velocity * velocity_scores[i])
                
            if total_score > best_score:
                best_score, best_v, best_w = total_score, v, w

        return best_v, best_w


    def generate_velocity_command(self):
        """生成速度命令"""
        v, w = self.find_best_velocity_angle_combination()
        cmd_vel = Twist()
        cmd_vel.linear.x = v
        cmd_vel.angular.z = w
        return cmd_vel
