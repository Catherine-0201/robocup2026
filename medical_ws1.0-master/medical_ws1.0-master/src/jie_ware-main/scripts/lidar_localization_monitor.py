#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
激光雷达定位质量监控节点 - 专门监控 lidar_loc.cpp 的 TF 变换
通过监测 lidar_loc.cpp 发布的 map->odom 变换的稳定性来检测定位异常
"""

import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import math
from collections import deque
from threading import Lock
import datetime

# ROS消息类型
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, Pose
from std_msgs.msg import Bool, String
import tf.transformations as tf_trans


class LidarLocalizationMonitor:
    def __init__(self):
        rospy.init_node('lidar_localization_monitor', anonymous=False)
        
        # 参数配置
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param('~base_frame', 'base_footprint')
        self.laser_frame = rospy.get_param('~laser_frame', 'laser')
        
        # 突变检测参数
        self.position_jump_threshold = rospy.get_param('~position_jump_threshold', 1.0)  # 位移突变阈值(米)
        self.rotation_jump_threshold = rospy.get_param('~rotation_jump_threshold', 0.5)  # 角度突变阈值(弧度)
        self.velocity_threshold = rospy.get_param('~velocity_threshold', 3.0)  # 最大合理速度(米/秒)
        self.angular_velocity_threshold = rospy.get_param('~angular_velocity_threshold', 2.0)  # 最大合理角速度(弧度/秒)
        self.stable_frames_required = rospy.get_param('~stable_frames_required', 10)  # 恢复需要的稳定帧数
        self.max_fallback_duration = rospy.get_param('~max_fallback_duration', 15.0)  # 最大备用定位时间(秒)
        
        # 启动和稳定化参数
        self.startup_delay = rospy.get_param('~startup_delay', 5.0)  # 启动延迟时间(秒)
        self.monitoring_frequency = rospy.get_param('~monitoring_frequency', 10.0)  # 监控频率(Hz)
        self.min_time_between_switches = rospy.get_param('~min_time_between_switches', 2.0)  # 切换间隔(秒)
        
        # 滤波参数
        self.use_moving_average = rospy.get_param('~use_moving_average', True)  # 是否使用移动平均
        self.moving_average_window = rospy.get_param('~moving_average_window', 3)  # 移动平均窗口
        
        # 状态变量
        self.is_localization_reliable = True  # 初始假设可靠，但会在首次检查后修正
        self.initial_check_done = False  # 是否已完成初始检查
        self.fallback_start_time = None
        self.stable_frame_count = 0
        self.last_switch_time = None  # 上次切换时间
        self.startup_time = rospy.Time.now()  # 启动时间
        self.monitoring_active = False  # 监控是否激活
        
        # 备用定位增强变量
        self.backup_mode_active = False  # 备用模式是否激活
        self.lidar_loc_suspended = False  # lidar_loc.cpp 是否被暂停
        self.backup_tf_rate = rospy.get_param('~backup_tf_rate', 30.0)  # 备用TF发布频率
        self.smooth_transition = rospy.get_param('~smooth_transition', True)  # 是否使用平滑过渡
        
        # 位姿历史记录
        self.last_pose = None
        self.last_timestamp = None
        self.pose_history = deque(maxlen=20)  # 保存最近20帧
        self.pose_changes_buffer = deque(maxlen=self.moving_average_window)  # 用于移动平均
        
        # 里程计数据
        self.last_odom_pose = None
        self.odom_drift_compensation = None  # 用于补偿里程计漂移
        
        # 备用定位状态跟踪
        self.backup_pose_history = deque(maxlen=10)  # 备用定位位姿历史
        self.last_lidar_loc_pose = None  # 最后一次有效的激光定位位姿
        self.transition_start_pose = None  # 切换开始时的位姿
        self.recovery_attempt_count = 0  # 恢复尝试次数
        
        # 动态障碍物检测相关
        self.use_filtered_scan = rospy.get_param('~use_filtered_scan', True)  # 是否使用过滤后的激光数据
        self.localization_confidence = 1.0  # 定位置信度
        self.low_confidence_threshold = rospy.get_param('~low_confidence_threshold', 0.5)  # 低置信度阈值
        
        # 地图匹配相关参数
        self.map_match_threshold = rospy.get_param('~map_match_threshold', 0.75)  # 地图匹配度阈值(0-1)
        self.match_check_radius = rospy.get_param('~match_check_radius', 5.0)  # 匹配检查半径(米)
        self.min_match_points = rospy.get_param('~min_match_points', 50)  # 最少匹配点数
        self.enable_map_matching = rospy.get_param('~enable_map_matching', True)  # 是否启用地图匹配检查
        
        # 地图数据
        self.map_data = None  # 占用栅格地图数据
        self.current_scan_data = None  # 当前激光扫描数据
        self.last_match_score = 0.0  # 上次匹配分数
        
        # 状态输出控制
        self.last_reliable_state = None  # 上次的可靠状态，用于检测状态变化
        self.startup_normal_logged = False  # 是否已经输出过启动正常信息
        self.match_score_display_counter = 0  # 地图匹配度显示计数器
        
        # 里程计定位优化参数
        self.enable_motion_filtering = rospy.get_param('~enable_motion_filtering', True)
        self.motion_filter_alpha = rospy.get_param('~motion_filter_alpha', 0.8)
        self.max_linear_velocity = rospy.get_param('~max_linear_velocity', 2.0)
        self.max_angular_velocity = rospy.get_param('~max_angular_velocity', 1.5)
        self.drift_correction_interval = rospy.get_param('~drift_correction_interval', 2.0)
        self.position_variance_threshold = rospy.get_param('~position_variance_threshold', 0.1)
        self.enable_velocity_smoothing = rospy.get_param('~enable_velocity_smoothing', True)
        self.velocity_window_size = rospy.get_param('~velocity_window_size', 5)
        
        # 里程计话题配置
        self.odom_topic = rospy.get_param('~odom_topic', '/odom')  # 默认使用 /odom
        self.use_odom_combined = rospy.get_param('~use_odom_combined', True)  # 是否优先使用 odom_combined
        self.odom_combined_topic = rospy.get_param('~odom_combined_topic', '/odom_combined')
        
        # 运动滤波和平滑相关变量
        self.filtered_pose = None  # 滤波后的位姿
        self.velocity_history = deque(maxlen=self.velocity_window_size)  # 速度历史
        self.last_drift_correction_time = None  # 上次漂移修正时间
        self.pose_variance_buffer = deque(maxlen=10)  # 位姿方差缓冲区
        
        # 线程锁
        self.data_lock = Lock()
        
        # TF相关
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # 统一定位数据发布器
        self.unified_pose_pub = rospy.Publisher(
            '/unified_pose', PoseWithCovarianceStamped, queue_size=1)
        self.unified_tf_status_pub = rospy.Publisher(
            '/unified_tf_status', String, queue_size=1)
        
        # 兼容性发布器（保持向后兼容）
        self.localization_status_pub = rospy.Publisher(
            '/localization_status', Bool, queue_size=1)
        self.localization_info_pub = rospy.Publisher(
            '/localization_info', String, queue_size=1)
        self.backup_pose_pub = rospy.Publisher(
            '/backup_pose', PoseWithCovarianceStamped, queue_size=1)
        self.lidar_loc_tf_status_pub = rospy.Publisher(
            '/lidar_loc_tf_status', String, queue_size=1)
        
        # 订阅器
        # 里程计订阅 - 支持多个话题源
        if self.use_odom_combined:
            # 优先订阅 odom_combined，同时监听 /odom 作为备用
            self.odom_combined_sub = rospy.Subscriber(self.odom_combined_topic, Odometry, self.odom_combined_callback)
            self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
            rospy.loginfo("里程计订阅: 主要使用 {}，备用使用 {}".format(self.odom_combined_topic, self.odom_topic))
        else:
            # 只使用 /odom 话题
            self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
            rospy.loginfo("里程计订阅: 使用 {}".format(self.odom_topic))
        
        self.filtered_scan_sub = rospy.Subscriber('/scan_filtered', LaserScan, self.filtered_scan_callback)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        # 同时监听原始激光数据用于地图匹配
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        # 定时器 - 主要监控逻辑
        monitor_period = 1.0 / self.monitoring_frequency
        self.monitor_timer = rospy.Timer(rospy.Duration(monitor_period), self.monitor_pose_changes)
        
        # 定时器 - 备用TF发布
        backup_tf_period = 1.0 / self.backup_tf_rate
        self.backup_tf_timer = rospy.Timer(rospy.Duration(backup_tf_period), self.backup_tf_callback)
        
        rospy.loginfo("激光雷达定位监控节点已启动 - 专门监控 lidar_loc.cpp 的 TF 变换")
        
        # 启动提示
        self.print_terminal_alert(
            "INFO", 
            "激光雷达定位监控节点已启动",
            {
                "监控频率": f"{self.monitoring_frequency} Hz",
                "启动延迟": f"{self.startup_delay} 秒",
                "位移阈值": f"{self.position_jump_threshold} m",
                "角度阈值": f"{self.rotation_jump_threshold} rad",
                "速度阈值": f"{self.velocity_threshold} m/s",
                "角速度阈值": f"{self.angular_velocity_threshold} rad/s",
                "备用模式最大时长": f"{self.max_fallback_duration} 秒",
                "里程计话题": f"{self.odom_combined_topic if self.use_odom_combined else self.odom_topic}",
                "使用融合里程计": "是" if self.use_odom_combined else "否",
                "统一定位数据": "启用",
                "统一话题": "/unified_pose, /unified_tf_status",
                "地图匹配功能": "启用" if self.enable_map_matching else "禁用",
                "地图匹配阈值": f"{self.map_match_threshold:.3f}",
                "匹配检查半径": f"{self.match_check_radius} m",
                "最少匹配点数": self.min_match_points
            }
        )

    def print_terminal_alert(self, alert_type, message, details=None):
        """在终端中显示醒目的警告信息"""
        # 获取当前时间
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        # 定义颜色代码
        colors = {
            'RED': '\033[91m',
            'YELLOW': '\033[93m',
            'GREEN': '\033[92m',
            'BLUE': '\033[94m',
            'CYAN': '\033[96m',
            'MAGENTA': '\033[95m',
            'WHITE': '\033[97m',
            'BOLD': '\033[1m',
            'UNDERLINE': '\033[4m',
            'END': '\033[0m'
        }
        
        # 根据警告类型选择颜色和图标
        if alert_type == "TF_JUMP":
            color = colors['RED']
            icon = "🚨"
            border_char = "="
        elif alert_type == "RECOVERY":
            color = colors['GREEN']
            icon = "✅"
            border_char = "-"
        elif alert_type == "TIMEOUT":
            color = colors['YELLOW']
            icon = "⏰"
            border_char = "*"
        elif alert_type == "LOW_CONFIDENCE":
            color = colors['MAGENTA']
            icon = "⚠️"
            border_char = "~"
        else:
            color = colors['CYAN']
            icon = "ℹ️"
            border_char = "-"
        
        # 计算边框长度
        max_width = 80
        message_lines = [f"[{timestamp}] {icon} {message}"]
        
        if details:
            if isinstance(details, dict):
                for key, value in details.items():
                    if isinstance(value, float):
                        message_lines.append(f"  • {key}: {value:.4f}")
                    else:
                        message_lines.append(f"  • {key}: {value}")
            else:
                message_lines.append(f"  详情: {details}")
        
        # 找到最长行的长度
        max_line_length = max(len(line) for line in message_lines)
        border_length = min(max_width, max(max_line_length + 4, 50))
        
        # 打印警告框
        print(f"\n{color}{colors['BOLD']}")
        print(border_char * border_length)
        for line in message_lines:
            padding = border_length - len(line) - 2
            print(f"{border_char} {line}{' ' * padding}{border_char}")
        print(border_char * border_length)
        print(f"{colors['END']}")

    def get_current_map_to_odom_transform(self):
        """获取 lidar_loc.cpp 发布的 map->odom 变换"""
        try:
            # 直接监控 map->odom 变换，这是 lidar_loc.cpp 发布的核心变换
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.odom_frame, rospy.Time.now(), rospy.Duration(0.1))
            
            # 提取位置和姿态
            position = transform.transform.translation
            rotation = transform.transform.rotation
            
            # 转换为欧拉角
            euler = tf_trans.euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
            
            pose_data = {
                'x': position.x,
                'y': position.y,
                'yaw': euler[2],
                'timestamp': transform.header.stamp
            }
            
            return pose_data
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(1.0, "无法获取 map->odom 变换: {}".format(e))
            return None

    def get_current_pose_in_map(self):
        """获取当前机器人在地图坐标系中的位姿（通过组合变换）"""
        try:
            # 获取base_frame在map坐标系中的位姿
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rospy.Time.now(), rospy.Duration(0.1))
            
            # 提取位置和姿态
            position = transform.transform.translation
            rotation = transform.transform.rotation
            
            # 转换为欧拉角
            euler = tf_trans.euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
            
            pose_data = {
                'x': position.x,
                'y': position.y,
                'yaw': euler[2],
                'timestamp': transform.header.stamp
            }
            
            return pose_data
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(1.0, "无法获取当前位姿: {}".format(e))
            return None

    def map_callback(self, msg):
        """地图回调函数"""
        with self.data_lock:
            self.map_data = msg
            rospy.loginfo("收到地图数据，尺寸: {}x{}, 分辨率: {:.4f}m".format(
                msg.info.width, msg.info.height, msg.info.resolution))

    def scan_callback(self, msg):
        """原始激光数据回调函数（用于地图匹配）"""
        with self.data_lock:
            self.current_scan_data = msg

    def odom_combined_callback(self, msg):
        """odom_combined 回调函数 - 优先使用融合后的里程计数据"""
        with self.data_lock:
            self.last_odom_pose = {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'yaw': tf_trans.euler_from_quaternion([
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w
                ])[2],
                'timestamp': msg.header.stamp,
                'source': 'odom_combined',  # 标记数据来源
                'covariance': msg.pose.covariance  # 保存协方差信息用于质量评估
            }
            
            rospy.logdebug("收到 odom_combined 数据: x={:.3f}, y={:.3f}, yaw={:.3f}".format(
                msg.pose.pose.position.x, msg.pose.pose.position.y, 
                tf_trans.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
                                              msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]))

    def odom_callback(self, msg):
        """里程计回调函数 - 作为备用数据源"""
        # 只有在没有 odom_combined 数据时才使用 /odom 数据
        if not self.use_odom_combined or self.last_odom_pose is None or 'source' not in self.last_odom_pose:
            with self.data_lock:
                self.last_odom_pose = {
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y,
                    'yaw': tf_trans.euler_from_quaternion([
                        msg.pose.pose.orientation.x,
                        msg.pose.pose.orientation.y,
                        msg.pose.pose.orientation.z,
                        msg.pose.pose.orientation.w
                    ])[2],
                    'timestamp': msg.header.stamp,
                    'source': 'odom',  # 标记数据来源
                    'covariance': msg.pose.covariance  # 保存协方差信息
                }
                
                rospy.logdebug("使用备用 /odom 数据: x={:.3f}, y={:.3f}, yaw={:.3f}".format(
                    msg.pose.pose.position.x, msg.pose.pose.position.y, 
                    tf_trans.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
                                                  msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])[2]))

    def filtered_scan_callback(self, msg):
        """过滤后激光数据回调函数"""
        if not self.use_filtered_scan:
            return
            
        # 根据过滤后的激光数据计算定位置信度
        valid_points = sum(1 for r in msg.ranges if msg.range_min <= r <= msg.range_max)
        total_points = len(msg.ranges)
        
        if total_points > 0:
            valid_ratio = valid_points / total_points
            
            # 更新定位置信度
            with self.data_lock:
                if valid_ratio >= 0.8:
                    self.localization_confidence = 1.0
                elif valid_ratio >= 0.6:
                    self.localization_confidence = 0.8
                elif valid_ratio >= 0.4:
                    self.localization_confidence = 0.5
                else:
                    self.localization_confidence = 0.2
                    
                # 如果置信度过低，触发备用定位
                if (self.localization_confidence < self.low_confidence_threshold and 
                    self.is_localization_reliable):
                    rospy.logwarn("检测到大量动态障碍物，定位置信度过低 ({:.2f})，切换到里程计定位".format(
                        self.localization_confidence))
                    
                    # 显示终端警告
                    self.print_terminal_alert(
                        "LOW_CONFIDENCE", 
                        "检测到大量动态障碍物，定位置信度过低！",
                        {
                            "当前置信度": self.localization_confidence,
                            "阈值": self.low_confidence_threshold,
                            "有效激光点比例": valid_ratio,
                            "切换到": "里程计定位模式"
                        }
                    )
                    
                    self.is_localization_reliable = False
                    self.fallback_start_time = rospy.Time.now()
                    self.last_switch_time = rospy.Time.now()
                    self.stable_frame_count = 0
                    
                    if self.last_odom_pose is not None:
                        self.calculate_odom_drift_compensation()

    def detect_pose_jump(self, current_pose, last_pose):
        """检测位姿突变"""
        if last_pose is None:
            return False, {}
            
        # 计算时间间隔
        dt = (current_pose['timestamp'] - last_pose['timestamp']).to_sec()
        if dt <= 0 or dt > 1.0:  # 避免异常时间间隔
            return False, {}
        
        # 计算位移变化
        dx = current_pose['x'] - last_pose['x']
        dy = current_pose['y'] - last_pose['y']
        position_change = math.sqrt(dx*dx + dy*dy)
        
        # 计算角度变化
        dyaw = current_pose['yaw'] - last_pose['yaw']
        # 处理角度环绕
        while dyaw > math.pi:
            dyaw -= 2 * math.pi
        while dyaw < -math.pi:
            dyaw += 2 * math.pi
        angle_change = abs(dyaw)
        
        # 计算速度
        velocity = position_change / dt
        angular_velocity = angle_change / dt
        
        # 检测突变
        jump_info = {
            'position_change': position_change,
            'angle_change': angle_change,
            'velocity': velocity,
            'angular_velocity': angular_velocity,
            'dt': dt
        }
        
        # 判断是否为突变
        is_jump = (position_change > self.position_jump_threshold or
                   angle_change > self.rotation_jump_threshold or
                   velocity > self.velocity_threshold or
                   angular_velocity > self.angular_velocity_threshold)
        
        return is_jump, jump_info

    def calculate_map_match_score(self):
        """计算当前激光点云与地图障碍物的匹配度"""
        if not self.enable_map_matching:
            return 1.0  # 如果禁用地图匹配，始终返回满分
            
        if (self.map_data is None or self.current_scan_data is None):
            rospy.logwarn_throttle(5.0, "地图匹配计算: 缺少地图或激光数据")
            return 0.0
        
        try:
            # 获取当前机器人在地图坐标系中的位姿
            current_pose = self.get_current_pose_in_map()
            if current_pose is None:
                return 0.0
            
            # 将激光点转换到地图坐标系
            map_points = []
            angle = self.current_scan_data.angle_min
            
            for i, range_val in enumerate(self.current_scan_data.ranges):
                if (range_val >= self.current_scan_data.range_min and 
                    range_val <= self.current_scan_data.range_max and
                    range_val <= self.match_check_radius):  # 只检查一定范围内的点
                    
                    # 在激光雷达坐标系下的点
                    laser_x = range_val * math.cos(angle)
                    laser_y = range_val * math.sin(angle)
                    
                    # 转换到地图坐标系（假设激光雷达在机器人中心）
                    cos_yaw = math.cos(current_pose['yaw'])
                    sin_yaw = math.sin(current_pose['yaw'])
                    
                    map_x = current_pose['x'] + laser_x * cos_yaw - laser_y * sin_yaw
                    map_y = current_pose['y'] + laser_x * sin_yaw + laser_y * cos_yaw
                    
                    map_points.append((map_x, map_y))
                
                angle += self.current_scan_data.angle_increment
            
            if len(map_points) < self.min_match_points:
                rospy.logwarn_throttle(2.0, "地图匹配计算: 有效点数不足 ({} < {})".format(
                    len(map_points), self.min_match_points))
                return 0.0
            
            # 计算匹配分数
            matched_points = 0
            total_points = len(map_points)
            
            for map_x, map_y in map_points:
                # 将地图坐标转换为栅格坐标
                grid_x = int((map_x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
                grid_y = int((map_y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)
                
                # 检查是否在地图范围内
                if (0 <= grid_x < self.map_data.info.width and 
                    0 <= grid_y < self.map_data.info.height):
                    
                    # 获取栅格值
                    index = grid_y * self.map_data.info.width + grid_x
                    if index < len(self.map_data.data):
                        grid_value = self.map_data.data[index]
                        
                        # 检查周围区域是否有障碍物（容忍一定误差）
                        match_found = False
                        search_radius = 2  # 搜索半径（栅格）
                        
                        for dy in range(-search_radius, search_radius + 1):
                            for dx in range(-search_radius, search_radius + 1):
                                check_x = grid_x + dx
                                check_y = grid_y + dy
                                
                                if (0 <= check_x < self.map_data.info.width and 
                                    0 <= check_y < self.map_data.info.height):
                                    
                                    check_index = check_y * self.map_data.info.width + check_x
                                    if check_index < len(self.map_data.data):
                                        check_value = self.map_data.data[check_index]
                                        
                                        # 如果找到障碍物 (值为100) 或未知区域 (值为-1)
                                        if check_value == 100 or check_value == -1:
                                            match_found = True
                                            break
                            
                            if match_found:
                                break
                        
                        if match_found:
                            matched_points += 1
            
            # 计算匹配分数
            match_score = matched_points / total_points if total_points > 0 else 0.0
            
            rospy.logdebug("地图匹配计算: {}/{} 点匹配，分数: {:.3f}".format(
                matched_points, total_points, match_score))
            
            return match_score
            
        except Exception as e:
            rospy.logwarn("地图匹配计算失败: {}".format(e))
            return 0.0

    def print_realtime_match_score(self, force_recalculate=False):
        """实时显示地图匹配度信息（简洁版本）"""
        # 每10次循环显示一次，避免输出过于频繁
        self.match_score_display_counter += 1
        
        # 每5次循环重新计算匹配度（约0.5秒一次），或强制重新计算
        if self.match_score_display_counter % 5 == 0 or force_recalculate:
            if self.is_localization_reliable:  # 只在正常模式下重新计算
                new_match_score = self.calculate_map_match_score()
                self.last_match_score = new_match_score
                
                # 检查在正常模式下地图匹配度是否突然变差
                if (self.enable_map_matching and 
                    new_match_score < self.map_match_threshold):
                    
                    rospy.logwarn("正常模式检测到地图匹配度过低({:.3f} < {:.3f})，切换到备用定位模式".format(
                        new_match_score, self.map_match_threshold))
                    
                    # 返回值，告诉调用者需要切换模式
                    return new_match_score, True  # 返回匹配度和是否需要切换
                
        if self.match_score_display_counter >= 10:
            self.match_score_display_counter = 0
            
            # 获取当前时间
            timestamp = datetime.datetime.now().strftime("%H:%M:%S")
            
            # 根据状态选择颜色
            if self.is_localization_reliable:
                status_color = '\033[92m'  # 绿色
                status_text = "正常"
            else:
                status_color = '\033[93m'  # 黄色
                status_text = "备用"
            
            # 根据匹配度选择颜色
            if self.last_match_score >= self.map_match_threshold:
                score_color = '\033[92m'  # 绿色
            elif self.last_match_score >= 0.6:
                score_color = '\033[93m'  # 黄色
            else:
                score_color = '\033[91m'  # 红色
            
            # 根据模式显示不同的稳定帧要求
            if self.is_localization_reliable:
                frames_required = self.stable_frames_required
            else:
                frames_required = max(3, self.stable_frames_required // 3)
            
            print(f"\r\033[K[{timestamp}] 定位状态: {status_color}{status_text}\033[0m | "
                  f"地图匹配度: {score_color}{self.last_match_score:.3f}\033[0m/"
                  f"{self.map_match_threshold:.3f} | "
                  f"稳定帧: {self.stable_frame_count}/{frames_required}", 
                  end='', flush=True)
        
        # 函数结束时返回默认值
        return self.last_match_score, False

    def apply_motion_filtering(self, current_pose):
        """应用运动滤波减少位姿抖动"""
        if not self.enable_motion_filtering:
            return current_pose
            
        if self.filtered_pose is None:
            self.filtered_pose = current_pose.copy()
            return current_pose
        
        # 低通滤波 - 平滑位姿变化
        alpha = self.motion_filter_alpha
        filtered_pose = {
            'x': alpha * self.filtered_pose['x'] + (1 - alpha) * current_pose['x'],
            'y': alpha * self.filtered_pose['y'] + (1 - alpha) * current_pose['y'],
            'yaw': self.filter_angle(self.filtered_pose['yaw'], current_pose['yaw'], alpha),
            'timestamp': current_pose['timestamp']
        }
        
        self.filtered_pose = filtered_pose
        return filtered_pose
    
    def filter_angle(self, old_angle, new_angle, alpha):
        """角度滤波，处理角度环绕问题"""
        # 计算角度差，处理环绕
        diff = new_angle - old_angle
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        
        # 应用滤波
        return old_angle + (1 - alpha) * diff
    
    def validate_velocity(self, current_pose, last_pose):
        """验证速度是否合理，过滤异常数据"""
        if last_pose is None:
            return True
            
        dt = (current_pose['timestamp'] - last_pose['timestamp']).to_sec()
        if dt <= 0 or dt > 1.0:
            return True  # 时间异常，跳过检查
        
        # 计算速度
        dx = current_pose['x'] - last_pose['x']
        dy = current_pose['y'] - last_pose['y']
        linear_velocity = math.sqrt(dx*dx + dy*dy) / dt
        
        dyaw = current_pose['yaw'] - last_pose['yaw']
        while dyaw > math.pi:
            dyaw -= 2 * math.pi
        while dyaw < -math.pi:
            dyaw += 2 * math.pi
        angular_velocity = abs(dyaw) / dt
        
        # 检查速度是否合理
        if linear_velocity > self.max_linear_velocity:
            rospy.logwarn_throttle(2.0, "检测到异常线性速度: {:.3f} m/s (限制: {:.3f})".format(
                linear_velocity, self.max_linear_velocity))
            return False
            
        if angular_velocity > self.max_angular_velocity:
            rospy.logwarn_throttle(2.0, "检测到异常角速度: {:.3f} rad/s (限制: {:.3f})".format(
                angular_velocity, self.max_angular_velocity))
            return False
        
        # 记录速度历史用于平滑
        if self.enable_velocity_smoothing:
            self.velocity_history.append({
                'linear': linear_velocity,
                'angular': angular_velocity,
                'timestamp': current_pose['timestamp']
            })
        
        return True
    
    def get_smoothed_velocity(self):
        """获取平滑后的速度"""
        if len(self.velocity_history) < 2:
            return 0.0, 0.0
            
        linear_velocities = [v['linear'] for v in self.velocity_history]
        angular_velocities = [v['angular'] for v in self.velocity_history]
        
        # 使用中位数过滤异常值
        linear_median = sorted(linear_velocities)[len(linear_velocities)//2]
        angular_median = sorted(angular_velocities)[len(angular_velocities)//2]
        
        return linear_median, angular_median
    
    def update_pose_variance(self, pose):
        """更新位姿方差，用于评估定位质量"""
        self.pose_variance_buffer.append({
            'x': pose['x'],
            'y': pose['y'],
            'yaw': pose['yaw']
        })
        
        if len(self.pose_variance_buffer) < 5:
            return 0.0
        
        # 计算位置方差
        x_values = [p['x'] for p in self.pose_variance_buffer]
        y_values = [p['y'] for p in self.pose_variance_buffer]
        
        x_mean = sum(x_values) / len(x_values)
        y_mean = sum(y_values) / len(y_values)
        
        x_variance = sum((x - x_mean)**2 for x in x_values) / len(x_values)
        y_variance = sum((y - y_mean)**2 for y in y_values) / len(y_values)
        
        position_variance = math.sqrt(x_variance + y_variance)
        
        if position_variance > self.position_variance_threshold:
            rospy.logwarn_throttle(5.0, "位姿方差较大: {:.4f} (阈值: {:.4f})，定位可能不稳定".format(
                position_variance, self.position_variance_threshold))
        
        return position_variance

    def monitor_pose_changes(self, event):
        """监控 map->odom 变换的稳定性 - 主要监控逻辑"""
        # 检查启动延迟
        if not self.monitoring_active:
            startup_elapsed = (rospy.Time.now() - self.startup_time).to_sec()
            if startup_elapsed < self.startup_delay:
                return  # 还在启动延迟期内，不进行监控
            else:
                self.monitoring_active = True
                rospy.loginfo("启动延迟结束，开始监控 lidar_loc.cpp 的 TF 变换 (延迟了 {:.1f} 秒)".format(startup_elapsed))
        
        # 获取当前的 map->odom 变换（lidar_loc.cpp 发布的核心变换）
        current_map_to_odom = self.get_current_map_to_odom_transform()
        if current_map_to_odom is None:
            return
        
        # 同时获取机器人在地图中的位姿用于备用定位
        current_pose = self.get_current_pose_in_map()
        if current_pose is None:
            current_pose = current_map_to_odom  # 使用 map->odom 作为备用
        
        with self.data_lock:
            # 首次启动时进行初始地图匹配度检查
            if not self.initial_check_done:
                initial_match_score = self.calculate_map_match_score()
                self.last_match_score = initial_match_score
                
                if initial_match_score < self.map_match_threshold and self.enable_map_matching:
                    # 启动时地图匹配度不足，立即切换到备用模式
                    rospy.logwarn("启动检查: 地图匹配度不足({:.3f} < {:.3f})，启动备用定位模式".format(
                        initial_match_score, self.map_match_threshold))
                    
                    # 显示启动检查警告
                    print(f"\n")
                    self.print_terminal_alert(
                        "LOW_CONFIDENCE", 
                        "启动检查: 地图匹配度不足！",
                        {
                            "当前匹配度": f"{initial_match_score:.4f}",
                            "需要匹配度": f"{self.map_match_threshold:.4f}",
                            "启动状态": "切换到备用定位模式"
                        }
                    )
                    
                    self.is_localization_reliable = False
                    self.fallback_start_time = rospy.Time.now()
                    self.last_switch_time = rospy.Time.now()
                    self.stable_frame_count = 0
                    self.activate_backup_mode()
                    
                    if self.last_odom_pose is not None:
                        self.calculate_odom_drift_compensation()
                else:
                    rospy.loginfo("启动检查: 地图匹配度良好({:.3f})，激光雷达定位正常".format(initial_match_score))
                
                self.initial_check_done = True
            
            # 检测 map->odom 变换的突变（这是 lidar_loc.cpp 定位质量的直接指标）
            is_map_odom_jump, map_odom_jump_info = self.detect_pose_jump(current_map_to_odom, self.last_pose)
            
            # 同时检测机器人位姿的变化作为辅助判断
            is_pose_jump, pose_jump_info = self.detect_pose_jump(current_pose, self.last_pose)
            
            # 综合判断：优先考虑 map->odom 变换的稳定性
            is_jump = is_map_odom_jump
            jump_info = map_odom_jump_info if map_odom_jump_info else pose_jump_info
            
            # 应用移动平均滤波
            if self.use_moving_average and jump_info:
                self.pose_changes_buffer.append({
                    'position_change': jump_info['position_change'],
                    'angle_change': jump_info['angle_change'],
                    'velocity': jump_info['velocity'],
                    'angular_velocity': jump_info['angular_velocity']
                })
                
                # 如果缓冲区有足够数据，使用平均值
                if len(self.pose_changes_buffer) >= self.moving_average_window:
                    avg_position_change = sum(item['position_change'] for item in self.pose_changes_buffer) / len(self.pose_changes_buffer)
                    avg_angle_change = sum(item['angle_change'] for item in self.pose_changes_buffer) / len(self.pose_changes_buffer)
                    avg_velocity = sum(item['velocity'] for item in self.pose_changes_buffer) / len(self.pose_changes_buffer)
                    avg_angular_velocity = sum(item['angular_velocity'] for item in self.pose_changes_buffer) / len(self.pose_changes_buffer)
                    
                    # 重新评估是否为突变（使用平滑后的值）
                    is_jump = (avg_position_change > self.position_jump_threshold or
                               avg_angle_change > self.rotation_jump_threshold or
                               avg_velocity > self.velocity_threshold or
                               avg_angular_velocity > self.angular_velocity_threshold)
                    
                    # 更新jump_info为平滑后的值
                    jump_info.update({
                        'position_change': avg_position_change,
                        'angle_change': avg_angle_change,
                        'velocity': avg_velocity,
                        'angular_velocity': avg_angular_velocity,
                        'filtered': True
                    })
            
            # 更新位姿历史（记录 map->odom 变换）
            self.pose_history.append(current_map_to_odom)
            
            if is_jump:
                # 检查切换间隔
                current_time = rospy.Time.now()
                if (self.last_switch_time is not None and 
                    (current_time - self.last_switch_time).to_sec() < self.min_time_between_switches):
                    rospy.logdebug("检测到突变，但距离上次切换时间过短，忽略本次突变")
                    is_jump = False  # 忽略本次突变
                
            if is_jump:
                # 检测到突变
                if self.is_localization_reliable:
                    rospy.logwarn("检测到 lidar_loc.cpp 发布的 map->odom 变换突变！切换到里程计定位模式")
                    rospy.logwarn("map->odom 变换突变信息: 位移={:.3f}m, 角度={:.3f}rad, 速度={:.3f}m/s, 角速度={:.3f}rad/s".format(
                        jump_info['position_change'], jump_info['angle_change'],
                        jump_info['velocity'], jump_info['angular_velocity']))
                    
                    # 先换行，避免覆盖实时显示的内容
                    print(f"\n")
                    
                    # 显示详细的终端警告
                    self.print_terminal_alert(
                        "TF_JUMP", 
                        "检测到 map->odom 坐标变换突变！",
                        {
                            "位移变化": f"{jump_info['position_change']:.4f} m (阈值: {self.position_jump_threshold} m)",
                            "角度变化": f"{jump_info['angle_change']:.4f} rad (阈值: {self.rotation_jump_threshold} rad)",
                            "线性速度": f"{jump_info['velocity']:.4f} m/s (阈值: {self.velocity_threshold} m/s)",
                            "角速度": f"{jump_info['angular_velocity']:.4f} rad/s (阈值: {self.angular_velocity_threshold} rad/s)",
                            "时间间隔": f"{jump_info['dt']:.4f} s",
                            "使用滤波": jump_info.get('filtered', False),
                            "切换到": "里程计定位模式"
                        }
                    )
                    
                    # 启动备用定位模式
                    self.activate_backup_mode()
                    
                    # 切换到备用模式
                    self.is_localization_reliable = False
                    self.fallback_start_time = rospy.Time.now()
                    self.last_switch_time = rospy.Time.now()  # 记录切换时间
                    self.stable_frame_count = 0
                    
                    # 记录里程计漂移补偿
                    if self.last_odom_pose is not None:
                        self.calculate_odom_drift_compensation()
                
                # 重置稳定帧计数
                self.stable_frame_count = 0
            else:
                # 没有突变
                if not self.is_localization_reliable:
                    # 当前处于备用模式，检查恢复条件
                    
                    # 计算地图匹配度
                    map_match_score = self.calculate_map_match_score()
                    self.last_match_score = map_match_score
                    
                    # 简化恢复逻辑：主要依靠地图匹配度，减少稳定帧要求
                    min_stable_frames = max(3, self.stable_frames_required // 3)  # 减少到原来的1/3，最少3帧
                    self.stable_frame_count += 1
                    
                    frames_stable = self.stable_frame_count >= min_stable_frames
                    map_matches = map_match_score >= self.map_match_threshold
                    
                    rospy.logdebug("恢复检查: 稳定帧={}/{}, 地图匹配={:.3f}/{:.3f}".format(
                        self.stable_frame_count, min_stable_frames,
                        map_match_score, self.map_match_threshold))
                    
                    # 放宽恢复条件：匹配度达标且最少稳定帧数即可
                    if map_matches and frames_stable:
                        rospy.loginfo("连续{}帧稳定且地图匹配度达标({:.3f})，恢复激光雷达定位".format(
                            min_stable_frames, map_match_score))
                        
                        # 计算备用模式持续时间
                        fallback_duration = (rospy.Time.now() - self.fallback_start_time).to_sec() if self.fallback_start_time else 0
                        
                        # 先换行，避免覆盖实时显示的内容
                        print(f"\n")
                        
                        # 显示恢复信息
                        self.print_terminal_alert(
                            "RECOVERY", 
                            "激光雷达定位已恢复正常！",
                            {
                                "稳定帧数": self.stable_frame_count,
                                "需要帧数": min_stable_frames,
                                "地图匹配度": f"{map_match_score:.4f}",
                                "匹配阈值": f"{self.map_match_threshold:.4f}",
                                "备用模式持续": f"{fallback_duration:.2f} 秒",
                                "恢复尝试次数": self.recovery_attempt_count,
                                "恢复到": "激光雷达定位模式"
                            }
                        )
                        
                        # 停用备用定位模式
                        self.deactivate_backup_mode()
                        
                        self.is_localization_reliable = True
                        self.fallback_start_time = None
                        self.last_switch_time = rospy.Time.now()  # 记录恢复时间
                        self.stable_frame_count = 0
                        self.recovery_attempt_count = 0
                        self.odom_drift_compensation = None
                    
                    elif not map_matches:
                        # 地图匹配度不够，记录但不重置计数（给更多机会）
                        if self.stable_frame_count % 30 == 0:  # 每30帧提醒一次，约3秒
                            rospy.logwarn("地图匹配度不足({:.3f} < {:.3f})，等待匹配度提升".format(
                                map_match_score, self.map_match_threshold))
                        
                        # 如果长时间匹配度不足，可以考虑降低要求
                        if self.stable_frame_count >= self.stable_frames_required:
                            # 长时间无法恢复，显示警告但继续等待
                            if self.stable_frame_count == self.stable_frames_required:
                                print(f"\n")
                                self.print_terminal_alert(
                                    "LOW_CONFIDENCE", 
                                    "长时间地图匹配度不足，继续等待",
                                    {
                                        "当前匹配度": f"{map_match_score:.4f}",
                                        "需要匹配度": f"{self.map_match_threshold:.4f}",
                                        "等待时间": f"{self.stable_frame_count/10:.1f} 秒",
                                        "建议": "检查环境或调低匹配阈值"
                                    }
                                )
                    
                    # 检查备用模式超时
                    elif self.fallback_start_time is not None:
                        fallback_duration = (rospy.Time.now() - self.fallback_start_time).to_sec()
                        if fallback_duration > self.max_fallback_duration:
                            rospy.logwarn("备用定位模式超时，强制恢复激光雷达定位")
                            
                            # 先换行，避免覆盖实时显示的内容
                            print(f"\n")
                            
                            # 显示超时警告
                            self.print_terminal_alert(
                                "TIMEOUT", 
                                "备用定位模式超时！强制恢复激光雷达定位",
                                {
                                    "备用模式时长": f"{fallback_duration:.2f} 秒",
                                    "最大允许时长": f"{self.max_fallback_duration} 秒",
                                    "当前稳定帧数": self.stable_frame_count,
                                    "需要稳定帧数": self.stable_frames_required,
                                    "恢复尝试次数": self.recovery_attempt_count + 1,
                                    "强制恢复到": "激光雷达定位模式"
                                }
                            )
                            
                            # 强制停用备用定位模式
                            self.deactivate_backup_mode()
                            
                            self.is_localization_reliable = True
                            self.fallback_start_time = None
                            self.last_switch_time = rospy.Time.now()  # 记录强制恢复时间
                            self.stable_frame_count = 0
                            self.recovery_attempt_count += 1
                            self.odom_drift_compensation = None
            
            # 检测状态变化并输出相应信息
            current_reliable_state = self.is_localization_reliable
            
            # 如果状态发生变化，或者是启动后第一次检测到正常状态
            if (self.last_reliable_state != current_reliable_state or
                (current_reliable_state and not self.startup_normal_logged and self.initial_check_done)):
                
                if current_reliable_state:
                    if not self.startup_normal_logged and self.initial_check_done:
                        # 启动后第一次检测到正常状态（已完成初始检查）
                        rospy.loginfo("激光雷达定位工作正常，开始监控")
                        self.startup_normal_logged = True
                        # 输出单行确认信息，不使用大框
                        print(f"\n\033[92m✓ 激光雷达定位正常工作中...\033[0m")
                    # 不再重复输出正常状态信息，恢复信息已在别处处理
                
                # 更新状态记录
                self.last_reliable_state = current_reliable_state
            
            # 实时显示地图匹配度和状态信息
            if self.monitoring_active and self.initial_check_done:
                # 地图匹配度计算和显示（函数内部控制计算频率）
                match_score, need_switch = self.print_realtime_match_score()
                
                # 如果在正常模式下检测到匹配度过低，切换到备用模式
                if need_switch and self.is_localization_reliable:
                    # 先换行，避免覆盖实时显示的内容
                    print(f"\n")
                    
                    # 显示匹配度过低警告
                    self.print_terminal_alert(
                        "LOW_CONFIDENCE", 
                        "正常运行时检测到地图匹配度过低！",
                        {
                            "当前匹配度": f"{match_score:.4f}",
                            "需要匹配度": f"{self.map_match_threshold:.4f}",
                            "触发原因": "定位质量下降",
                            "切换到": "备用定位模式"
                        }
                    )
                    
                    # 切换到备用模式
                    self.is_localization_reliable = False
                    self.fallback_start_time = rospy.Time.now()
                    self.last_switch_time = rospy.Time.now()
                    self.stable_frame_count = 0
                    self.activate_backup_mode()
                    
                    if self.last_odom_pose is not None:
                        self.calculate_odom_drift_compensation()
            
            # 更新上次位姿（记录 map->odom 变换）
            self.last_pose = current_map_to_odom
            
            # 发布统一定位数据
            if self.is_localization_reliable:
                # 激光定位正常，发布激光定位数据
                self.publish_unified_pose(
                    current_map_to_odom, 
                    source="lidar_loc", 
                    confidence=min(1.0, self.localization_confidence)
                )
                self.publish_unified_tf_status(
                    is_reliable=True,
                    source="lidar_loc",
                    confidence=min(1.0, self.localization_confidence),
                    additional_info={'map_match_score': self.last_match_score}
                )
            else:
                # 使用备用定位，发布里程计数据
                if self.backup_mode_active and self.last_odom_pose is not None:
                    odom_quality, _ = self.evaluate_odom_quality()
                    self.publish_unified_pose(
                        self.last_odom_pose,
                        source="odometry_fallback",
                        confidence=odom_quality
                    )
                    self.publish_unified_tf_status(
                        is_reliable=False,
                        source="odometry_fallback",
                        confidence=odom_quality,
                        additional_info={'fallback_duration': (rospy.Time.now() - self.fallback_start_time).to_sec() if self.fallback_start_time else 0}
                    )
            
            # 发布兼容性状态信息（保持向后兼容）
            self.publish_status_info(is_jump, jump_info)

    def calculate_odom_drift_compensation(self):
        """计算里程计漂移补偿"""
        if self.last_pose is None or self.last_odom_pose is None:
            return
            
        # 计算激光定位位姿与里程计位姿的差值
        self.odom_drift_compensation = {
            'dx': self.last_pose['x'] - self.last_odom_pose['x'],
            'dy': self.last_pose['y'] - self.last_odom_pose['y'],
            'dyaw': self.last_pose['yaw'] - self.last_odom_pose['yaw']
        }
        
        rospy.loginfo("计算里程计漂移补偿: dx={:.3f}, dy={:.3f}, dyaw={:.3f}".format(
            self.odom_drift_compensation['dx'],
            self.odom_drift_compensation['dy'],
            self.odom_drift_compensation['dyaw']))

    def get_compensated_odom_pose(self):
        """获取补偿后的里程计位姿"""
        if self.last_odom_pose is None or self.odom_drift_compensation is None:
            return None
            
        compensated_pose = {
            'x': self.last_odom_pose['x'] + self.odom_drift_compensation['dx'],
            'y': self.last_odom_pose['y'] + self.odom_drift_compensation['dy'],
            'yaw': self.last_odom_pose['yaw'] + self.odom_drift_compensation['dyaw'],
            'timestamp': self.last_odom_pose['timestamp']
        }
        
        return compensated_pose

    def evaluate_odom_quality(self):
        """评估当前里程计数据的质量"""
        if self.last_odom_pose is None or 'covariance' not in self.last_odom_pose:
            return 0.0, "unknown"
        
        try:
            # 从协方差矩阵中提取位置和角度的不确定性
            cov = self.last_odom_pose['covariance']
            
            # 位置不确定性 (x, y 的方差)
            pos_variance = (cov[0] + cov[7]) / 2.0  # (cov[0] + cov[7]) / 2
            
            # 角度不确定性 (yaw 的方差)
            yaw_variance = cov[35]  # 最后一个元素通常是 yaw 的方差
            
            # 计算质量分数 (0-1，1为最好)
            # 位置质量：基于位置方差，方差越小质量越高
            pos_quality = max(0.0, 1.0 - min(1.0, pos_variance / 0.1))  # 0.1m² 作为参考方差
            
            # 角度质量：基于角度方差，方差越小质量越高
            yaw_quality = max(0.0, 1.0 - min(1.0, yaw_variance / 0.01))  # 0.01rad² 作为参考方差
            
            # 综合质量分数
            overall_quality = (pos_quality + yaw_quality) / 2.0
            
            # 数据来源质量评估
            source_quality = 1.0 if self.last_odom_pose.get('source') == 'odom_combined' else 0.8
            
            # 最终质量分数
            final_quality = overall_quality * source_quality
            
            quality_level = "excellent" if final_quality > 0.9 else \
                           "good" if final_quality > 0.7 else \
                           "fair" if final_quality > 0.5 else \
                           "poor"
            
            rospy.logdebug("里程计质量评估: 位置质量={:.3f}, 角度质量={:.3f}, 综合质量={:.3f}, 来源={}, 等级={}".format(
                pos_quality, yaw_quality, final_quality, 
                self.last_odom_pose.get('source', 'unknown'), quality_level))
            
            return final_quality, quality_level
            
        except Exception as e:
            rospy.logwarn("里程计质量评估失败: {}".format(e))
            return 0.5, "error"

    def publish_unified_pose(self, pose_data, source="unknown", confidence=1.0):
        """发布统一定位数据，包含来源和质量信息"""
        try:
            # 创建统一位姿消息
            unified_pose_msg = PoseWithCovarianceStamped()
            unified_pose_msg.header.stamp = rospy.Time.now()
            unified_pose_msg.header.frame_id = self.map_frame
            
            # 设置位姿
            unified_pose_msg.pose.pose.position.x = pose_data['x']
            unified_pose_msg.pose.pose.position.y = pose_data['y']
            unified_pose_msg.pose.pose.position.z = 0.0
            
            # 设置姿态（四元数）
            quat = tf_trans.quaternion_from_euler(0, 0, pose_data['yaw'])
            unified_pose_msg.pose.pose.orientation.x = quat[0]
            unified_pose_msg.pose.pose.orientation.y = quat[1]
            unified_pose_msg.pose.pose.orientation.z = quat[2]
            unified_pose_msg.pose.pose.orientation.w = quat[3]
            
            # 设置协方差矩阵（基于置信度调整）
            # 创建一个6x6的协方差矩阵
            covariance = [0.0] * 36
            
            # 位置协方差 (x, y, z)
            pos_variance = 0.01 * (1.0 - confidence)  # 置信度越低，方差越大
            covariance[0] = pos_variance   # x方差
            covariance[7] = pos_variance   # y方差
            covariance[14] = pos_variance  # z方差
            
            # 角度协方差 (roll, pitch, yaw)
            angle_variance = 0.001 * (1.0 - confidence)  # 角度方差
            covariance[21] = angle_variance  # roll方差
            covariance[28] = angle_variance  # pitch方差
            covariance[35] = angle_variance  # yaw方差
            
            unified_pose_msg.pose.covariance = covariance
            
            # 发布统一位姿
            self.unified_pose_pub.publish(unified_pose_msg)
            
            rospy.logdebug("发布统一定位数据: 来源={}, 置信度={:.3f}, 位置=({:.3f}, {:.3f}, {:.3f})".format(
                source, confidence, pose_data['x'], pose_data['y'], pose_data['yaw']))
                
        except Exception as e:
            rospy.logwarn("发布统一定位数据失败: {}".format(e))

    def publish_unified_tf_status(self, is_reliable, source, confidence, additional_info=None):
        """发布统一的TF状态信息"""
        try:
            # 创建统一状态消息
            unified_status = {
                'timestamp': rospy.Time.now().to_sec(),
                'localization_reliable': is_reliable,
                'data_source': source,
                'confidence': confidence,
                'backup_mode_active': self.backup_mode_active,
                'monitoring_active': self.monitoring_active,
                'stable_frames': self.stable_frame_count,
                'recovery_attempts': self.recovery_attempt_count
            }
            
            # 添加里程计相关信息
            if self.last_odom_pose is not None:
                odom_quality, odom_quality_level = self.evaluate_odom_quality()
                unified_status.update({
                    'odom_quality': odom_quality,
                    'odom_quality_level': odom_quality_level,
                    'odom_source': self.last_odom_pose.get('source', 'unknown')
                })
            
            # 添加地图匹配信息
            if self.enable_map_matching:
                unified_status.update({
                    'map_match_score': self.last_match_score,
                    'map_match_threshold': self.map_match_threshold
                })
            
            # 添加额外信息
            if additional_info:
                unified_status.update(additional_info)
            
            # 发布统一状态
            status_msg = String()
            status_msg.data = str(unified_status)
            self.unified_tf_status_pub.publish(status_msg)
            
            rospy.logdebug("发布统一TF状态: 来源={}, 可靠={}, 置信度={:.3f}".format(
                source, is_reliable, confidence))
                
        except Exception as e:
            rospy.logwarn("发布统一TF状态失败: {}".format(e))

    def publish_backup_tf(self):
        """发布基于里程计的备用TF（优化版本）"""
        if not self.backup_mode_active:
            return
            
        compensated_pose = self.get_compensated_odom_pose()
        if compensated_pose is None:
            rospy.logwarn_throttle(2.0, "无法获取补偿后的里程计位姿，备用TF发布失败")
            return
            
        try:
            # 应用速度验证 - 过滤异常数据
            if not self.validate_velocity(compensated_pose, self.backup_pose_history[-1] if self.backup_pose_history else None):
                rospy.logdebug("跳过异常里程计数据")
                return
            
            # 应用运动滤波减少抖动
            filtered_pose = self.apply_motion_filtering(compensated_pose)
            
            # 更新位姿方差评估
            variance = self.update_pose_variance(filtered_pose)
            
            # 如果启用平滑过渡，对位姿进行插值
            if self.smooth_transition and self.transition_start_pose is not None:
                final_pose = self.apply_smooth_transition(filtered_pose)
            else:
                final_pose = filtered_pose
            
            # 定期漂移修正
            current_time = rospy.Time.now()
            if (self.last_drift_correction_time is None or 
                (current_time - self.last_drift_correction_time).to_sec() >= self.drift_correction_interval):
                
                # 基于地图匹配度进行微调
                if self.last_match_score > 0.3:  # 有一定匹配度时才进行微调
                    final_pose = self.apply_drift_correction(final_pose)
                
                self.last_drift_correction_time = current_time
            
            # 创建map到odom的变换
            map_to_odom = TransformStamped()
            map_to_odom.header.stamp = rospy.Time.now()
            map_to_odom.header.frame_id = self.map_frame
            map_to_odom.child_frame_id = self.odom_frame
            
            # 设置位置
            map_to_odom.transform.translation.x = final_pose['x']
            map_to_odom.transform.translation.y = final_pose['y']
            map_to_odom.transform.translation.z = 0.0
            
            # 设置姿态
            quat = tf_trans.quaternion_from_euler(0, 0, final_pose['yaw'])
            map_to_odom.transform.rotation.x = quat[0]
            map_to_odom.transform.rotation.y = quat[1]
            map_to_odom.transform.rotation.z = quat[2]
            map_to_odom.transform.rotation.w = quat[3]
            
            # 发布变换
            self.tf_broadcaster.sendTransform(map_to_odom)
            
            # 记录备用位姿历史
            self.backup_pose_history.append(final_pose)
            
            # 发布统一定位数据（备用模式下的实时位姿）
            odom_quality, _ = self.evaluate_odom_quality()
            self.publish_unified_pose(
                final_pose,
                source="odometry_fallback_realtime",
                confidence=odom_quality
            )
            
            # 获取平滑后的速度用于调试
            linear_vel, angular_vel = self.get_smoothed_velocity()
            
            rospy.logdebug("优化备用TF: x={:.3f}, y={:.3f}, yaw={:.3f}, 方差={:.4f}, 速度=({:.3f},{:.3f})".format(
                final_pose['x'], final_pose['y'], final_pose['yaw'], variance, linear_vel, angular_vel))
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(1.0, "发布备用TF时出错: {}".format(e))
    
    def apply_drift_correction(self, pose):
        """应用漂移修正，基于地图匹配度进行微调"""
        if self.last_match_score < 0.2:  # 匹配度太低，不进行修正
            return pose
        
        # 基于匹配度的置信度权重
        confidence_weight = min(1.0, self.last_match_score / self.map_match_threshold)
        
        # 如果有最近的激光定位位姿作为参考
        if self.last_lidar_loc_pose is not None:
            # 计算与参考位姿的偏差
            dx = self.last_lidar_loc_pose['x'] - pose['x']
            dy = self.last_lidar_loc_pose['y'] - pose['y']
            
            # 小幅度修正，避免突然跳跃
            correction_factor = 0.1 * confidence_weight
            
            corrected_pose = {
                'x': pose['x'] + dx * correction_factor,
                'y': pose['y'] + dy * correction_factor,
                'yaw': pose['yaw'],  # 角度修正较为敏感，暂时不修正
                'timestamp': pose['timestamp']
            }
            
            rospy.logdebug("应用漂移修正: dx={:.4f}, dy={:.4f}, 权重={:.3f}".format(
                dx * correction_factor, dy * correction_factor, confidence_weight))
            
            return corrected_pose
        
        return pose

    def backup_tf_callback(self, event):
        """备用TF发布定时器回调"""
        if self.backup_mode_active:
            self.publish_backup_tf()

    def activate_backup_mode(self):
        """激活备用定位模式"""
        if not self.backup_mode_active:
            rospy.loginfo("激活备用定位模式 - 基于里程计的定位")
            self.backup_mode_active = True
            
            # 保存当前激光定位位姿作为参考
            if self.last_pose is not None:
                self.last_lidar_loc_pose = self.last_pose.copy()
                self.transition_start_pose = self.last_pose.copy()
            
            # 清空备用位姿历史
            self.backup_pose_history.clear()
            
            rospy.loginfo("备用定位模式已激活，将使用补偿后的里程计数据")
            
            # 显示备用模式激活信息
            self.print_terminal_alert(
                "INFO", 
                "备用定位模式已激活",
                {
                    "定位源": "里程计 + 漂移补偿",
                    "备用TF频率": f"{self.backup_tf_rate} Hz",
                    "使用平滑过渡": self.smooth_transition,
                    "状态": "监控节点接管TF发布"
                }
            )

    def deactivate_backup_mode(self):
        """停用备用定位模式"""
        if self.backup_mode_active:
            rospy.loginfo("停用备用定位模式 - 恢复到激光雷达定位")
            self.backup_mode_active = False
            self.lidar_loc_suspended = False
            
            # 清理备用模式相关数据
            self.backup_pose_history.clear()
            self.transition_start_pose = None
            
            rospy.loginfo("已恢复到激光雷达定位模式")
            
            # 显示备用模式停用信息
            self.print_terminal_alert(
                "INFO", 
                "备用定位模式已停用",
                {
                    "恢复到": "激光雷达定位模式",
                    "TF发布": "lidar_loc.cpp 节点",
                    "状态": "监控节点停止TF发布"
                }
            )

    def apply_smooth_transition(self, target_pose):
        """应用平滑过渡到目标位姿"""
        if self.transition_start_pose is None or len(self.backup_pose_history) == 0:
            return target_pose
            
        # 计算从启动备用模式到现在的时间
        if self.fallback_start_time is not None:
            elapsed_time = (rospy.Time.now() - self.fallback_start_time).to_sec()
            # 在前2秒内进行平滑过渡
            transition_duration = 2.0
            
            if elapsed_time < transition_duration:
                # 计算插值因子
                alpha = elapsed_time / transition_duration
                alpha = min(1.0, max(0.0, alpha))
                
                # 对位置进行线性插值
                smoothed_pose = {
                    'x': self.transition_start_pose['x'] * (1 - alpha) + target_pose['x'] * alpha,
                    'y': self.transition_start_pose['y'] * (1 - alpha) + target_pose['y'] * alpha,
                    'yaw': self.interpolate_angle(self.transition_start_pose['yaw'], target_pose['yaw'], alpha),
                    'timestamp': target_pose['timestamp']
                }
                
                return smoothed_pose
        
        return target_pose

    def interpolate_angle(self, angle1, angle2, alpha):
        """对角度进行插值，处理环绕问题"""
        diff = angle2 - angle1
        # 处理角度环绕
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        
        return angle1 + alpha * diff

    def publish_status_info(self, is_jump, jump_info):
        """发布状态信息"""
        # 发布定位可靠性状态
        status_msg = Bool()
        status_msg.data = self.is_localization_reliable
        self.localization_status_pub.publish(status_msg)
        
        # 发布详细信息
        info_msg = String()
        status_info = {
            'reliable': self.is_localization_reliable,
            'is_jump': is_jump,
            'stable_frames': self.stable_frame_count,
            'confidence': self.localization_confidence,
            'map_match_score': self.last_match_score,
            'map_match_threshold': self.map_match_threshold,
            'map_matching_enabled': self.enable_map_matching,
        }
        
        if jump_info:
            status_info.update({
                'position_change': jump_info.get('position_change', 0.0),
                'angle_change': jump_info.get('angle_change', 0.0),
                'velocity': jump_info.get('velocity', 0.0),
                'angular_velocity': jump_info.get('angular_velocity', 0.0)
            })
        
        if self.fallback_start_time is not None:
            fallback_duration = (rospy.Time.now() - self.fallback_start_time).to_sec()
            status_info['fallback_duration'] = fallback_duration
        
        info_msg.data = str(status_info)
        self.localization_info_pub.publish(info_msg)
        
        # 发布 lidar_loc.cpp TF 状态信息
        tf_status_msg = String()
        
        # 评估里程计质量
        odom_quality, odom_quality_level = self.evaluate_odom_quality()
        
        tf_status_data = {
            'map_to_odom_reliable': self.is_localization_reliable,
            'monitoring_active': self.monitoring_active,
            'backup_mode_active': self.backup_mode_active,
            'jump_detected': is_jump,
            'stable_frames': self.stable_frame_count,
            'recovery_attempts': self.recovery_attempt_count,
            'using_odometry_fallback': not self.is_localization_reliable,
            'odom_quality': odom_quality,
            'odom_quality_level': odom_quality_level,
            'odom_source': self.last_odom_pose.get('source', 'unknown') if self.last_odom_pose else 'none'
        }
        if jump_info:
            tf_status_data.update({
                'map_odom_position_change': jump_info.get('position_change', 0.0),
                'map_odom_angle_change': jump_info.get('angle_change', 0.0),
                'map_odom_velocity': jump_info.get('velocity', 0.0),
                'map_odom_angular_velocity': jump_info.get('angular_velocity', 0.0)
            })
        if self.fallback_start_time is not None:
            tf_status_data['fallback_duration'] = (rospy.Time.now() - self.fallback_start_time).to_sec()
        tf_status_msg.data = str(tf_status_data)
        self.lidar_loc_tf_status_pub.publish(tf_status_msg)
        
        # 备用TF由专门的定时器发布，这里不需要重复调用

    def run(self):
        """运行节点"""
        rospy.loginfo("激光雷达定位监控节点开始运行 - 监控 lidar_loc.cpp 的 map->odom 变换...")
        rospy.spin()


if __name__ == '__main__':
    try:
        monitor = LidarLocalizationMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("激光雷达定位监控节点关闭")
