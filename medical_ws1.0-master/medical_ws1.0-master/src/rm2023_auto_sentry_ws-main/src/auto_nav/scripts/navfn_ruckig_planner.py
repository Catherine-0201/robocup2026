#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
阶段1：NavfnRuckig医疗机器人路径规划器 (Python版本)
功能：
1. 接收NavfnROS的全局路径
2. 使用ruckig生成满足加速度约束的轨迹
3. 实现速度分段的加速度限制（可参数化配置）
4. 复用原有的路径跟随和阈值化重规划功能
5. 发布控制指令到 /cmd_vel
"""

import rospy
import numpy as np
import math
import time
from collections import deque

# ROS消息类型
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from roborts_msgs.srv import PidPlannerStatus, PidPlannerStatusResponse

# TF相关
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# ruckig Python库
try:
    import ruckig
    RUCKIG_AVAILABLE = True
    rospy.loginfo("成功导入ruckig Python库")
except ImportError:
    RUCKIG_AVAILABLE = False
    rospy.logwarn("未找到ruckig Python库，将使用简化轨迹生成")


class VelocityAccelerationLimit:
    """
    速度加速度限制配置类
    
    功能说明：
    - 定义不同速度区间对应的加速度限制
    - 实现速度分段控制策略
    - 为ruckig轨迹生成器提供约束参数
    """
    def __init__(self, v_min, v_max, acc_x_limit, acc_y_limit, acc_z_limit):
        self.v_min = v_min          # 速度区间下限 [m/s]
        self.v_max = v_max          # 速度区间上限 [m/s]
        self.acc_x_limit = acc_x_limit  # X方向加速度限制 [m/s²]
        self.acc_y_limit = acc_y_limit  # Y方向加速度限制 [m/s²]  
        self.acc_z_limit = acc_z_limit  # 角加速度限制 [rad/s²]


class NavfnRuckigPlanner:
    """
    NavfnRuckig医疗机器人路径规划器
    
    系统架构：
    ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
    │   move_base     │───→│ NavfnRuckig     │───→│   底盘控制      │
    │  (NavfnROS)     │    │   规划器        │    │  (/cmd_vel)     │
    │  全局路径生成    │    │ 局部轨迹生成     │    │                │
    └─────────────────┘    └─────────────────┘    └─────────────────┘
    
    核心模块功能：
    1. 【路径接收模块】- 订阅NavfnROS的全局路径
    2. 【轨迹生成模块】- 使用ruckig生成满足加速度约束的局部轨迹  
    3. 【速度控制模块】- 实现速度分段加速度限制
    4. 【路径跟随模块】- 复用原有路径跟随算法
    5. 【阈值重规划模块】- 减少路径抖动的智能重规划
    6. 【安全监控模块】- 紧急停止和恢复行为
    
    话题接口：
    - 订阅: /move_base/NavfnROS/plan (全局路径)
    - 发布: /navfn_ruckig_planner/local_path (局部轨迹)
    - 发布: /cmd_vel (速度控制指令)
    """

    def __init__(self):
        rospy.init_node('navfn_ruckig_planner', anonymous=False)
        
        # 初始化参数
        self.load_parameters()
        
        # 初始化变量
        self.initialize_variables()
        
        # 初始化ROS组件
        self.initialize_ros_components()
        
        # 加载速度加速度限制配置
        self.load_velocity_acceleration_limits()
        
        # 初始化ruckig轨迹生成器
        self.initialize_ruckig_generator()
        
        rospy.loginfo("NavfnRuckig医疗机器人规划器(Python版本)已启动")
        rospy.loginfo(f"最大速度: {self.max_velocity:.2f} m/s, 规划频率: {self.plan_frequency} Hz")
        rospy.loginfo(f"速度区间数量: {len(self.velocity_limits)}, 阈值化重规划: {'启用' if self.enable_path_threshold else '禁用'}")

    def load_parameters(self):
        """
        【参数加载模块】
        
        功能：从ROS参数服务器加载所有配置参数
        包括：基础运动参数、ruckig轨迹参数、阈值重规划参数、安全参数
        """
        # 基础规划参数
        self.max_velocity = rospy.get_param('~max_velocity', 1.0)
        self.plan_frequency = rospy.get_param('~plan_frequency', 20)
        self.goal_dist_tolerance = rospy.get_param('~goal_dist_tolerance', 0.25)
        self.prune_ahead_distance = rospy.get_param('~prune_ahead_distance', 1.0)
        self.global_frame = rospy.get_param('~global_frame', 'map')
        self.robot_frame = rospy.get_param('~robot_frame', 'base_footprint')
        
        # ruckig轨迹生成参数
        self.trajectory_time_step = rospy.get_param('~trajectory_time_step', 0.1)
        self.trajectory_duration = rospy.get_param('~trajectory_duration', 3.0)
        
        # 阈值化重规划参数
        self.enable_path_threshold = rospy.get_param('~enable_path_threshold', True)
        self.path_distance_threshold = rospy.get_param('~path_distance_threshold', 0.3)
        self.path_angle_threshold = rospy.get_param('~path_angle_threshold', 0.52)
        self.path_update_min_interval = rospy.get_param('~path_update_min_interval', 1.0)
        self.path_comparison_points = rospy.get_param('~path_comparison_points', 5)
        
        # 安全参数
        self.emergency_stop_enabled = rospy.get_param('~emergency_stop_enabled', True)
        self.max_planning_time = rospy.get_param('~max_planning_time', 1.0)
        self.recovery_enabled = rospy.get_param('~recovery_enabled', True)

    def initialize_variables(self):
        """初始化变量"""
        # 路径和状态
        self.global_path = None
        self.current_local_path = None
        self.plan_active = False
        self.prune_index = 0
        
        # 机器人状态
        self.robot_yaw = 0.0
        self.game_state = 4
        self.planner_state = 2  # 0: 静止, 1: 原地旋转, 2: 路径跟踪
        
        # 速度和加速度
        self.current_speed = 0.0
        self.prev_cmd_vel = Twist()
        self.velocity_limits = []
        
        # 阈值化重规划
        self.last_path_update_time = rospy.Time.now()
        
        # ruckig相关
        self.ruckig_otg = None

    def initialize_ros_components(self):
        """初始化ROS组件"""
        # 发布器
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.local_path_pub = rospy.Publisher('~local_path', Path, queue_size=5)
        
        # 订阅器
        self.global_path_sub = rospy.Subscriber('/move_base/NavfnROS/plan', Path, 
                                               self.global_path_callback, queue_size=5)
        
        # 服务器
        self.planner_server = rospy.Service('~planner_status', PidPlannerStatus, 
                                          self.planner_state_request)
        
        # TF监听器
        self.tf_listener = tf.TransformListener()
        
        # 定时器
        self.plan_timer = rospy.Timer(rospy.Duration(1.0/self.plan_frequency), 
                                     self.plan_timer_callback)

    def load_velocity_acceleration_limits(self):
        """
        【核心功能：速度分段加速度限制模块】
        
        功能：
        1. 从参数服务器读取速度区间配置
        2. 构建速度-加速度映射关系
        3. 支持参数化配置，无需重新编译
        
        配置格式：
        velocity_acceleration_limits:
          - [v_min, v_max, acc_x, acc_y, acc_z]  # 速度区间和对应加速度限制
        
        举例：
        - [0.0, 0.5, 0.2, 0.2, 0.5]  # 低速区间：保守加速度
        - [0.5, 1.0, 0.3, 0.3, 0.8]  # 中速区间：平衡性能
        """
        self.velocity_limits = []
        
        # 从参数服务器读取速度加速度限制配置
        limits_config = rospy.get_param('~velocity_acceleration_limits', None)
        
        if limits_config and isinstance(limits_config, list):
            for limit in limits_config:
                if isinstance(limit, list) and len(limit) == 5:
                    v_min, v_max, acc_x, acc_y, acc_z = limit
                    self.velocity_limits.append(
                        VelocityAccelerationLimit(v_min, v_max, acc_x, acc_y, acc_z)
                    )
                    rospy.loginfo(f"速度区间 [{v_min:.2f}-{v_max:.2f} m/s] -> 加速度限制: [{acc_x:.2f}, {acc_y:.2f}, {acc_z:.2f}]")
        else:
            # 默认配置
            self.velocity_limits.append(
                VelocityAccelerationLimit(0.0, 0.5, 0.2, 0.2, 0.5)
            )
            self.velocity_limits.append(
                VelocityAccelerationLimit(0.5, 1.0, 0.3, 0.3, 0.8)
            )
            rospy.logwarn("未找到速度加速度限制配置，使用默认值")

    def initialize_ruckig_generator(self):
        """初始化ruckig轨迹生成器"""
        if RUCKIG_AVAILABLE:
            try:
                # 创建3DOF ruckig轨迹生成器 (x, y, theta)
                self.ruckig_otg = ruckig.Ruckig(3, self.trajectory_time_step)
                rospy.loginfo(f"ruckig轨迹生成器初始化完成 - 时间步长: {self.trajectory_time_step:.3f} s")
            except Exception as e:
                rospy.logwarn(f"ruckig初始化失败: {e}, 将使用简化轨迹生成")
                self.ruckig_otg = None
        else:
            rospy.logwarn("ruckig不可用，将使用简化轨迹生成")

    def get_current_acceleration_limits(self, current_speed):
        """
        【速度自适应控制核心算法】
        
        功能：根据机器人当前速度，动态查找对应的加速度限制
        
        输入：current_speed [m/s] - 机器人当前速度
        输出：VelocityAccelerationLimit对象 - 包含对应的加速度限制
        
        逻辑：遍历所有速度区间，找到当前速度所在区间，返回对应限制
        """
        for limit in self.velocity_limits:
            if limit.v_min <= current_speed < limit.v_max:
                return limit
        
        # 如果没有找到匹配的区间，返回最后一个区间的限制
        if self.velocity_limits:
            return self.velocity_limits[-1]
        
        # 默认限制
        return VelocityAccelerationLimit(0.0, 1.0, 0.2, 0.2, 0.5)

    def update_current_speed(self, cmd_vel):
        """更新当前速度"""
        self.current_speed = math.sqrt(cmd_vel.linear.x**2 + cmd_vel.linear.y**2)

    def get_robot_pose(self, target_frame):
        """获取机器人在指定坐标系中的位姿"""
        try:
            now = rospy.Time()
            self.tf_listener.waitForTransform(target_frame, self.robot_frame, now, rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform(target_frame, self.robot_frame, now)
            
            pose = PoseStamped()
            pose.header.frame_id = target_frame
            pose.header.stamp = now
            pose.pose.position.x = trans[0]
            pose.pose.position.y = trans[1]
            pose.pose.position.z = trans[2]
            pose.pose.orientation.x = rot[0]
            pose.pose.orientation.y = rot[1]
            pose.pose.orientation.z = rot[2]
            pose.pose.orientation.w = rot[3]
            
            # 更新机器人航向角
            _, _, self.robot_yaw = euler_from_quaternion(rot)
            
            return pose, True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"获取机器人位姿失败: {e}")
            return None, False

    def euclidean_distance(self, pose1, pose2):
        """计算两点之间的欧几里得距离"""
        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        return math.sqrt(dx*dx + dy*dy)

    def find_nearest_pose(self, robot_pose, path):
        """
        【路径剪枝模块 - 复用原有成熟算法】
        
        功能：
        1. 找到全局路径上距离机器人最近的点
        2. 实现智能路径剪枝，避免走回头路  
        3. 应用前瞻距离(prune_ahead_distance)策略
        
        核心逻辑：
        - 从当前prune_index开始搜索
        - 当距离开始增大时停止搜索
        - 检查方向一致性，避免反向移动
        - 确保前瞻距离满足要求
        
        参数：prune_ahead_distance = 1.0m (launch文件中配置)
        """
        if not path or not path.poses:
            return 0
            
        dist_threshold = 10.0  # 10米阈值
        sq_dist_threshold = dist_threshold * dist_threshold
        
        if self.prune_index != 0:
            sq_dist = self.euclidean_distance(robot_pose, path.poses[self.prune_index-1])
        else:
            sq_dist = 1e10
            
        while self.prune_index < len(path.poses):
            new_sq_dist = self.euclidean_distance(robot_pose, path.poses[self.prune_index])
            if new_sq_dist > sq_dist and sq_dist < sq_dist_threshold:
                # 判断是否在同一方向且距离超过前瞻距离
                if self.prune_index > 0:
                    curr_pose = path.poses[self.prune_index].pose.position
                    prev_pose = path.poses[self.prune_index-1].pose.position
                    robot_pos = robot_pose.pose.position
                    
                    dot_product = ((curr_pose.x - robot_pos.x) * (prev_pose.x - robot_pos.x) + 
                                  (curr_pose.y - robot_pos.y) * (prev_pose.y - robot_pos.y))
                    
                    if dot_product > 0 and sq_dist > self.prune_ahead_distance:
                        self.prune_index -= 1
                    else:
                        sq_dist = new_sq_dist
                break
            sq_dist = new_sq_dist
            self.prune_index += 1
            
        self.prune_index = min(self.prune_index, len(path.poses) - 1)
        return self.prune_index

    def generate_ruckig_trajectory(self, waypoints, robot_pose):
        """
        【核心轨迹生成模块 - ruckig轨迹规划】
        
        功能：
        1. 使用ruckig库生成满足加速度约束的平滑轨迹
        2. 根据当前速度动态调整加速度限制  
        3. 处理3DOF运动：x, y, theta
        4. 提供回退机制：ruckig不可用时使用简化轨迹
        
        输入状态：
        - 当前位置：robot_pose (x, y, theta)
        - 当前速度：prev_cmd_vel (vx, vy, omega)  
        - 当前加速度：假设为0
        
        约束条件：
        - 最大速度：max_velocity [m/s]
        - 最大加速度：根据速度区间动态调整
        - 最大jerk：固定为1.0 [m/s³]
        
        输出：nav_msgs/Path格式的局部轨迹
        """
        if not waypoints or len(waypoints) < 2:
            rospy.logwarn("路径点不足，无法生成ruckig轨迹")
            return None
            
        # 获取当前速度对应的加速度限制
        limits = self.get_current_acceleration_limits(self.current_speed)
        
        if self.ruckig_otg is not None:
            try:
                # 设置ruckig输入参数
                input_param = ruckig.InputParameter(3)
                
                # 当前状态（位置、速度、加速度）
                robot_x = robot_pose.pose.position.x
                robot_y = robot_pose.pose.position.y
                robot_theta = self.robot_yaw
                
                input_param.current_position = [robot_x, robot_y, robot_theta]
                input_param.current_velocity = [self.prev_cmd_vel.linear.x, 
                                               self.prev_cmd_vel.linear.y, 
                                               self.prev_cmd_vel.angular.z]
                input_param.current_acceleration = [0.0, 0.0, 0.0]
                
                # 目标状态（使用第一个或第二个路径点作为目标）
                target_idx = min(1, len(waypoints) - 1)
                target_pose = waypoints[target_idx].pose
                target_x = target_pose.position.x
                target_y = target_pose.position.y
                target_orientation = [target_pose.orientation.x, target_pose.orientation.y,
                                    target_pose.orientation.z, target_pose.orientation.w]
                _, _, target_theta = euler_from_quaternion(target_orientation)
                
                input_param.target_position = [target_x, target_y, target_theta]
                input_param.target_velocity = [0.0, 0.0, 0.0]
                input_param.target_acceleration = [0.0, 0.0, 0.0]
                
                # 设置约束
                input_param.max_velocity = [self.max_velocity, self.max_velocity, 1.0]
                input_param.max_acceleration = [limits.acc_x_limit, limits.acc_y_limit, limits.acc_z_limit]
                input_param.max_jerk = [1.0, 1.0, 1.0]
                
                # 生成轨迹
                output_param = ruckig.OutputParameter(3)
                result = self.ruckig_otg.calculate(input_param, output_param)
                
                if result == ruckig.Result.Working or result == ruckig.Result.Finished:
                    # 构建局部路径
                    local_path = Path()
                    local_path.header.frame_id = self.global_frame
                    local_path.header.stamp = rospy.Time.now()
                    
                    # 采样轨迹点
                    trajectory = output_param.trajectory
                    duration = min(trajectory.duration, self.trajectory_duration)
                    
                    t = 0.0
                    while t <= duration:
                        position = [0.0, 0.0, 0.0]
                        velocity = [0.0, 0.0, 0.0]
                        acceleration = [0.0, 0.0, 0.0]
                        
                        trajectory.at_time(t, position, velocity, acceleration)
                        
                        pose = PoseStamped()
                        pose.header.frame_id = self.global_frame
                        pose.header.stamp = rospy.Time.now()
                        pose.pose.position.x = position[0]
                        pose.pose.position.y = position[1]
                        pose.pose.position.z = 0.0
                        
                        # 转换角度为四元数
                        quat = quaternion_from_euler(0, 0, position[2])
                        pose.pose.orientation.x = quat[0]
                        pose.pose.orientation.y = quat[1]
                        pose.pose.orientation.z = quat[2]
                        pose.pose.orientation.w = quat[3]
                        
                        local_path.poses.append(pose)
                        
                        t += self.trajectory_time_step
                    
                    rospy.logdebug(f"ruckig轨迹生成成功 - 轨迹点数: {len(local_path.poses)}, "
                                 f"轨迹时长: {duration:.2f} s, 使用加速度限制: "
                                 f"[{limits.acc_x_limit:.2f}, {limits.acc_y_limit:.2f}, {limits.acc_z_limit:.2f}]")
                    
                    return local_path
                else:
                    rospy.logwarn(f"ruckig轨迹生成失败 - 错误代码: {result}")
                    
            except Exception as e:
                rospy.logwarn(f"ruckig轨迹生成异常: {e}")
        
        # 回退到简单的直线轨迹
        return self.generate_simple_trajectory(waypoints, robot_pose)

    def generate_simple_trajectory(self, waypoints, robot_pose):
        """生成简单的直线轨迹（ruckig不可用时的回退方案）"""
        local_path = Path()
        local_path.header.frame_id = self.global_frame
        local_path.header.stamp = rospy.Time.now()
        
        # 添加当前位置和目标位置
        local_path.poses.append(robot_pose)
        if waypoints:
            local_path.poses.append(waypoints[0])
        
        return local_path

    def follow_trajectory(self, robot_pose, local_path):
        """跟踪轨迹并计算速度指令（复用原有逻辑）"""
        if not local_path or len(local_path.poses) < 2:
            return Twist()
        
        # 计算目标方向和距离
        target_pose = local_path.poses[1].pose
        diff_x = target_pose.position.x - robot_pose.pose.position.x
        diff_y = target_pose.position.y - robot_pose.pose.position.y
        diff_yaw = math.atan2(diff_y, diff_x)
        diff_distance = self.euclidean_distance(robot_pose, local_path.poses[1])
        
        # 归一化角度
        if diff_yaw > math.pi:
            diff_yaw -= 2 * math.pi
        elif diff_yaw < -math.pi:
            diff_yaw += 2 * math.pi
        
        rospy.logdebug(f"NavfnRuckig - diff_yaw: {diff_yaw:.3f}, diff_distance: {diff_distance:.3f}")
        
        # 获取当前速度对应的加速度限制来调整控制增益
        self.update_current_speed(self.prev_cmd_vel)
        limits = self.get_current_acceleration_limits(self.current_speed)
        
        # 使用加速度限制来调整控制增益（简单的P控制）
        p_gain = min(limits.acc_x_limit / 0.3, 1.0)  # 基于加速度限制调整增益
        
        # 计算全局坐标系下的速度指令
        vx_global = self.max_velocity * math.cos(diff_yaw) * p_gain
        vy_global = self.max_velocity * math.sin(diff_yaw) * p_gain
        
        # 转换到机器人坐标系
        cmd_vel = Twist()
        cmd_vel.linear.x = vx_global * math.cos(self.robot_yaw) + vy_global * math.sin(self.robot_yaw)
        cmd_vel.linear.y = -vx_global * math.sin(self.robot_yaw) + vy_global * math.cos(self.robot_yaw)
        
        # 医疗机器人在路径跟踪时不需要额外的角速度
        cmd_vel.angular.z = 0.0
        
        # 速度限制
        speed_magnitude = math.sqrt(cmd_vel.linear.x**2 + cmd_vel.linear.y**2)
        if speed_magnitude > self.max_velocity:
            scale = self.max_velocity / speed_magnitude
            cmd_vel.linear.x *= scale
            cmd_vel.linear.y *= scale
        
        return cmd_vel

    def should_accept_new_path(self, new_path):
        """
        【阈值化重规划模块 - 复用原有成熟算法】
        
        功能：智能判断是否接受新的全局路径，减少路径抖动
        
        判断条件：
        1. 时间阈值：距离上次接受路径的时间间隔 > 1.0秒
        2. 距离阈值：新路径与当前路径的平均偏差 > 0.3米  
        3. 角度阈值：路径方向改变 > 30度(0.52弧度)
        
        算法逻辑：
        - 比较前N个路径点(path_comparison_points=5)
        - 计算平均距离偏差和角度偏差
        - 满足任一阈值条件则接受新路径
        
        效果：避免全局规划器的高频路径更新导致机器人抖动
        """
        if not self.enable_path_threshold:
            return True
        
        # 检查时间间隔阈值
        current_time = rospy.Time.now()
        if (current_time - self.last_path_update_time).to_sec() < self.path_update_min_interval:
            return False
        
        # 检查是否有当前路径进行比较
        if not self.global_path or not self.global_path.poses or not new_path.poses:
            return True
        
        # 计算路径距离偏差
        compare_points = min(self.path_comparison_points,
                           min(len(self.global_path.poses), len(new_path.poses)))
        total_distance_diff = 0.0
        
        for i in range(compare_points):
            dx = new_path.poses[i].pose.position.x - self.global_path.poses[i].pose.position.x
            dy = new_path.poses[i].pose.position.y - self.global_path.poses[i].pose.position.y
            total_distance_diff += math.sqrt(dx*dx + dy*dy)
        
        avg_distance_diff = total_distance_diff / compare_points
        
        # 计算路径角度偏差
        angle_diff = 0.0
        if compare_points >= 2:
            # 计算当前路径的初始方向
            old_dx = self.global_path.poses[1].pose.position.x - self.global_path.poses[0].pose.position.x
            old_dy = self.global_path.poses[1].pose.position.y - self.global_path.poses[0].pose.position.y
            old_angle = math.atan2(old_dy, old_dx)
            
            # 计算新路径的初始方向
            new_dx = new_path.poses[1].pose.position.x - new_path.poses[0].pose.position.x
            new_dy = new_path.poses[1].pose.position.y - new_path.poses[0].pose.position.y
            new_angle = math.atan2(new_dy, new_dx)
            
            # 计算角度差异
            angle_diff = abs(new_angle - old_angle)
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
        
        # 判断是否接受新路径
        accept_path = (avg_distance_diff > self.path_distance_threshold or 
                      angle_diff > self.path_angle_threshold)
        
        rospy.logdebug(f"阈值化重规划 - 距离偏差: {avg_distance_diff:.3f} m (阈值: {self.path_distance_threshold:.3f}), "
                      f"角度偏差: {angle_diff:.3f} rad (阈值: {self.path_angle_threshold:.3f}), "
                      f"接受: {'是' if accept_path else '否'}")
        
        return accept_path

    def publish_zero_velocity(self):
        """发布零速度指令"""
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)

    def plan_timer_callback(self, event):
        """
        【主控制循环 - 20Hz定时器回调】
        
        功能：系统的主要控制逻辑，以20Hz频率运行
        
        执行流程：
        1. 状态检查：确认当前规划器状态(0=静止, 1=旋转, 2=路径跟踪)
        2. 位姿获取：获取机器人当前位姿
        3. 目标检查：判断是否已到达目标位置
        4. 路径剪枝：找到最近的路径点(find_nearest_pose)
        5. 轨迹生成：使用ruckig生成局部轨迹
        6. 路径跟随：计算并发布速度控制指令
        7. 性能监控：输出规划耗时统计
        
        输出频率：20Hz (与原adaptive_pid保持一致)
        """
        if self.planner_state == 2:  # 路径跟踪状态
            if self.plan_active and self.global_path:
                start_time = time.time()
                
                # 获取当前机器人位姿
                robot_pose, success = self.get_robot_pose(self.global_frame)
                if not success:
                    rospy.logwarn("无法获取机器人位姿")
                    return
                
                # 检查是否已到达目标
                if (self.euclidean_distance(robot_pose, self.global_path.poses[-1]) <= self.goal_dist_tolerance or
                    self.prune_index == len(self.global_path.poses) - 1):
                    self.plan_active = False
                    self.publish_zero_velocity()
                    rospy.loginfo("NavfnRuckig医疗机器人已到达目标位置！")
                    return
                
                # 找到最近的路径点
                self.find_nearest_pose(robot_pose, self.global_path)
                
                # 提取前方的路径点作为waypoints
                waypoints = []
                waypoint_count = min(5, len(self.global_path.poses) - self.prune_index)
                for i in range(waypoint_count):
                    if self.prune_index + i < len(self.global_path.poses):
                        waypoints.append(self.global_path.poses[self.prune_index + i])
                
                # 使用ruckig生成局部轨迹
                self.current_local_path = self.generate_ruckig_trajectory(waypoints, robot_pose)
                if self.current_local_path:
                    self.local_path_pub.publish(self.current_local_path)
                
                # 跟踪轨迹并计算速度
                cmd_vel = self.follow_trajectory(robot_pose, self.current_local_path)
                self.cmd_vel_pub.publish(cmd_vel)
                
                # 保存当前速度指令用于下次计算
                self.prev_cmd_vel = cmd_vel
                
                plan_time = (time.time() - start_time) * 1000
                rospy.logdebug(f"NavfnRuckig规划耗时 {plan_time:.1f} ms，已通过 {self.prune_index}/{len(self.global_path.poses)} 路径点")
            else:
                # 无路径时保持静止
                self.publish_zero_velocity()
                
        elif self.planner_state == 1:  # 原地旋转状态
            cmd_vel = Twist()
            cmd_vel.angular.z = 0.5  # 固定角速度
            self.cmd_vel_pub.publish(cmd_vel)
            rospy.logdebug("医疗机器人 - 原地旋转")
        else:  # planner_state == 0, 静止状态
            self.publish_zero_velocity()

    def global_path_callback(self, msg):
        """
        【全局路径接收模块】
        
        功能：接收来自move_base/NavfnROS的全局路径
        
        话题：/move_base/NavfnROS/plan
        消息类型：nav_msgs/Path
        
        处理流程：
        1. 接收新的全局路径
        2. 调用阈值化重规划模块判断是否接受
        3. 如果接受，更新全局路径并重置规划状态
        4. 记录路径更新时间用于阈值判断
        """
        if msg.poses:
            # 使用阈值化重规划判断是否接受新路径
            if self.should_accept_new_path(msg):
                self.global_path = msg
                self.prune_index = 0
                self.plan_active = True
                self.last_path_update_time = rospy.Time.now()
                
                rospy.loginfo(f"NavfnRuckig医疗机器人 - 接受新的全局路径，包含 {len(msg.poses)} 个路径点")
            else:
                # 拒绝新路径，保持当前路径不变
                rospy.logdebug("NavfnRuckig医疗机器人 - 拒绝新路径，偏差未超过阈值，保持当前路径")

    def planner_state_request(self, req):
        """规划器状态请求服务处理"""
        rospy.loginfo(f"NavfnRuckig医疗机器人 - 请求数据: planner_state = {req.planner_state}, max_velocity = {req.max_x_speed}")
        
        # 医疗机器人的状态检查：0=静止, 1=原地旋转, 2=路径跟踪
        if req.planner_state < 0 or req.planner_state > 2:
            rospy.logerr(f"NavfnRuckig医疗机器人 - 无效的规划器状态: {req.planner_state} (应该是 0-2)")
            return PidPlannerStatusResponse(result=0)
        
        self.planner_state = req.planner_state
        
        # 更新速度参数
        if req.max_x_speed > 0:
            self.max_velocity = req.max_x_speed
            rospy.loginfo(f"NavfnRuckig医疗机器人 - 最大速度已更新: {self.max_velocity:.2f} m/s")
        
        # 打印当前状态
        state_names = {0: "静止", 1: "原地旋转", 2: "路径跟踪"}
        state_name = state_names.get(self.planner_state, "未知")
        rospy.loginfo(f"NavfnRuckig医疗机器人 - 状态切换至: {state_name}")
        
        return PidPlannerStatusResponse(result=1)

    def run(self):
        """运行规划器"""
        rospy.loginfo("NavfnRuckig医疗机器人规划器开始运行")
        rospy.spin()


if __name__ == '__main__':
    try:
        planner = NavfnRuckigPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("NavfnRuckig规划器节点已停止")
    except Exception as e:
        rospy.logerr(f"NavfnRuckig规划器运行异常: {e}")
        import traceback
        traceback.print_exc()
