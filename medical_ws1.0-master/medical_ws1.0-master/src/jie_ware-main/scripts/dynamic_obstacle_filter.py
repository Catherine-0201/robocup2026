#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
动态障碍物检测和过滤模块
用于改进激光雷达定位在存在未知障碍物时的鲁棒性
"""

import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import math
from collections import deque

# ROS消息类型
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
import tf.transformations as tf_trans


class DynamicObstacleFilter:
    def __init__(self):
        # 参数配置
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.base_frame = rospy.get_param('~base_frame', 'base_footprint')
        self.laser_frame = rospy.get_param('~laser_frame', 'laser')
        
        # 动态障碍物检测参数
        self.static_tolerance = rospy.get_param('~static_tolerance', 0.3)  # 静态特征容差(米)
        self.min_consecutive_detections = rospy.get_param('~min_consecutive_detections', 3)  # 最少连续检测次数
        self.temporal_window = rospy.get_param('~temporal_window', 5.0)  # 时间窗口(秒)
        self.min_static_points_ratio = rospy.get_param('~min_static_points_ratio', 0.6)  # 最少静态点比例
        
        # 地图和激光数据
        self.map_data = None
        self.map_info = None
        self.filtered_scan_history = deque(maxlen=10)
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 发布器
        self.filtered_scan_pub = rospy.Publisher('/scan_filtered', LaserScan, queue_size=1)
        self.dynamic_points_pub = rospy.Publisher('/dynamic_obstacles', LaserScan, queue_size=1)
        
        # 订阅器
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        rospy.loginfo("动态障碍物过滤器已启动")

    def map_callback(self, msg):
        """地图回调函数"""
        self.map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.map_info = msg.info
        rospy.loginfo("接收到地图数据: {}x{}, 分辨率: {}".format(
            msg.info.width, msg.info.height, msg.info.resolution))

    def scan_callback(self, msg):
        """激光雷达数据回调和过滤"""
        # 如果没有地图数据，直接转发原始数据（保证系统正常运行）
        if self.map_data is None:
            rospy.logwarn_throttle(5.0, "等待地图数据，暂时转发原始激光数据")
            self.filtered_scan_pub.publish(msg)  # 转发原始数据
            return
        
        try:
            # 获取当前机器人位姿
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.laser_frame, msg.header.stamp, rospy.Duration(0.1))
            
            # 过滤动态障碍物
            filtered_scan, dynamic_scan = self.filter_dynamic_obstacles(msg, transform)
            
            # 检查过滤后的数据质量
            if self.is_filtered_scan_reliable(filtered_scan):
                # 发布过滤后的激光数据
                self.filtered_scan_pub.publish(filtered_scan)
            else:
                # 如果过滤后的数据不可靠，发布警告并使用原始数据
                rospy.logwarn("过滤后激光数据不可靠，可能存在大量动态障碍物")
                # 可以选择降低定位置信度或切换到备用定位
                
            # 发布检测到的动态障碍物（用于调试）
            if dynamic_scan.ranges:
                self.dynamic_points_pub.publish(dynamic_scan)
                
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, "TF查询失败，转发原始激光数据: {}".format(e))
            self.filtered_scan_pub.publish(msg)  # TF失败时也转发原始数据

    def filter_dynamic_obstacles(self, scan_msg, transform):
        """过滤动态障碍物"""
        filtered_ranges = list(scan_msg.ranges)
        dynamic_ranges = [float('inf')] * len(scan_msg.ranges)
        
        # 获取变换参数
        trans = transform.transform.translation
        rot = transform.transform.rotation
        euler = tf_trans.euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
        yaw = euler[2]
        
        angle = scan_msg.angle_min
        dynamic_count = 0
        
        for i, range_val in enumerate(scan_msg.ranges):
            if scan_msg.range_min <= range_val <= scan_msg.range_max:
                # 计算点在地图坐标系中的位置
                x_laser = range_val * math.cos(angle)
                y_laser = range_val * math.sin(angle)
                
                x_map = trans.x + x_laser * math.cos(yaw) - y_laser * math.sin(yaw)
                y_map = trans.y + x_laser * math.sin(yaw) + y_laser * math.cos(yaw)
                
                # 检查该点是否为动态障碍物
                if self.is_dynamic_obstacle(x_map, y_map):
                    # 标记为动态障碍物
                    filtered_ranges[i] = float('inf')  # 从定位数据中移除
                    dynamic_ranges[i] = range_val  # 记录到动态障碍物中
                    dynamic_count += 1
            
            angle += scan_msg.angle_increment
        
        # 创建过滤后的激光数据
        filtered_scan = LaserScan()
        filtered_scan.header = scan_msg.header
        filtered_scan.angle_min = scan_msg.angle_min
        filtered_scan.angle_max = scan_msg.angle_max
        filtered_scan.angle_increment = scan_msg.angle_increment
        filtered_scan.time_increment = scan_msg.time_increment
        filtered_scan.scan_time = scan_msg.scan_time
        filtered_scan.range_min = scan_msg.range_min
        filtered_scan.range_max = scan_msg.range_max
        filtered_scan.ranges = filtered_ranges
        
        # 创建动态障碍物数据
        dynamic_scan = LaserScan()
        dynamic_scan.header = scan_msg.header
        dynamic_scan.angle_min = scan_msg.angle_min
        dynamic_scan.angle_max = scan_msg.angle_max
        dynamic_scan.angle_increment = scan_msg.angle_increment
        dynamic_scan.time_increment = scan_msg.time_increment
        dynamic_scan.scan_time = scan_msg.scan_time
        dynamic_scan.range_min = scan_msg.range_min
        dynamic_scan.range_max = scan_msg.range_max
        dynamic_scan.ranges = dynamic_ranges
        
        # 记录过滤统计信息
        total_valid_points = sum(1 for r in scan_msg.ranges if scan_msg.range_min <= r <= scan_msg.range_max)
        if total_valid_points > 0:
            dynamic_ratio = dynamic_count / total_valid_points
            if dynamic_ratio > 0.1:  # 如果超过10%的点被识别为动态
                rospy.loginfo_throttle(2.0, "检测到 {:.1f}% 动态障碍物点 ({}/{})".format(
                    dynamic_ratio * 100, dynamic_count, total_valid_points))
        
        return filtered_scan, dynamic_scan

    def is_dynamic_obstacle(self, x_map, y_map):
        """判断给定地图坐标点是否为动态障碍物"""
        # 转换到地图栅格坐标
        map_x = int((x_map - self.map_info.origin.position.x) / self.map_info.resolution)
        map_y = int((y_map - self.map_info.origin.position.y) / self.map_info.resolution)
        
        # 检查是否在地图范围内
        if not (0 <= map_x < self.map_data.shape[1] and 0 <= map_y < self.map_data.shape[0]):
            return True  # 地图外的点视为动态
        
        # 检查该点及其周围区域
        return self.check_static_consistency(map_x, map_y)

    def check_static_consistency(self, map_x, map_y, check_radius=2):
        """检查点的静态一致性"""
        # 方法1：检查该点周围是否应该有障碍物
        has_nearby_static_obstacle = False
        
        for dx in range(-check_radius, check_radius + 1):
            for dy in range(-check_radius, check_radius + 1):
                check_x = map_x + dx
                check_y = map_y + dy
                
                if (0 <= check_x < self.map_data.shape[1] and 
                    0 <= check_y < self.map_data.shape[0]):
                    
                    cell_value = self.map_data[check_y, check_x]
                    if cell_value == 100:  # 静态障碍物
                        has_nearby_static_obstacle = True
                        break
            if has_nearby_static_obstacle:
                break
        
        # 方法2：检查该点在地图中的状态
        current_cell = self.map_data[map_y, map_x]
        
        # 如果激光点击中的位置在地图中标记为自由空间，则可能是动态障碍物
        if current_cell == 0:  # 自由空间
            return True  # 很可能是动态障碍物
        elif current_cell == 100:  # 静态障碍物
            return False  # 静态障碍物
        elif current_cell == -1:  # 未知区域
            # 未知区域的处理更复杂，可以基于周围的静态特征判断
            return not has_nearby_static_obstacle
        
        return False

    def is_filtered_scan_reliable(self, filtered_scan):
        """检查过滤后的激光数据是否可靠"""
        valid_points = sum(1 for r in filtered_scan.ranges 
                          if filtered_scan.range_min <= r <= filtered_scan.range_max)
        total_points = len(filtered_scan.ranges)
        
        if total_points == 0:
            return False
        
        valid_ratio = valid_points / total_points
        return valid_ratio >= self.min_static_points_ratio

    def get_localization_confidence(self, filtered_scan):
        """计算基于过滤后数据的定位置信度"""
        if not filtered_scan.ranges:
            return 0.0
        
        valid_points = sum(1 for r in filtered_scan.ranges 
                          if filtered_scan.range_min <= r <= filtered_scan.range_max)
        total_points = len(filtered_scan.ranges)
        
        if total_points == 0:
            return 0.0
        
        # 基于有效点比例计算置信度
        valid_ratio = valid_points / total_points
        
        # 置信度计算：有效点比例越高，置信度越高
        if valid_ratio >= 0.8:
            return 1.0
        elif valid_ratio >= 0.6:
            return 0.8
        elif valid_ratio >= 0.4:
            return 0.5
        else:
            return 0.2


if __name__ == '__main__':
    rospy.init_node('dynamic_obstacle_filter')
    try:
        filter_node = DynamicObstacleFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("动态障碍物过滤器关闭")
