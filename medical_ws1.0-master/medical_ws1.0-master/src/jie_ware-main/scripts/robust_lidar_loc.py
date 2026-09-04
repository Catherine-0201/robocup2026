#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
鲁棒激光雷达定位算法
改进原有的简单模板匹配，增加对动态障碍物的抗干扰能力
"""

import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import math
from collections import deque
import cv2

# ROS消息类型
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
import tf.transformations as tf_trans

class RobustLidarLoc:
    def __init__(self):
        rospy.init_node('robust_lidar_loc', anonymous=False)
        
        # 参数配置
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param('~base_frame', 'base_footprint')
        self.laser_frame = rospy.get_param('~laser_frame', 'laser')
        
        # 鲁棒性参数
        self.outlier_threshold = rospy.get_param('~outlier_threshold', 0.5)  # 异常点阈值(米)
        self.min_inlier_ratio = rospy.get_param('~min_inlier_ratio', 0.7)  # 最小内点比例
        self.consistency_window = rospy.get_param('~consistency_window', 5)  # 一致性检查窗口
        self.max_position_jump = rospy.get_param('~max_position_jump', 1.0)  # 最大位置跳变(米)
        self.max_angle_jump = rospy.get_param('~max_angle_jump', 0.5)  # 最大角度跳变(弧度)
        
        # 状态变量
        self.map_data = None
        self.map_info = None
        self.last_pose = None
        self.pose_history = deque(maxlen=self.consistency_window)
        self.confidence_history = deque(maxlen=10)
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # 发布器
        self.pose_pub = rospy.Publisher('/robust_pose', PoseWithCovarianceStamped, queue_size=1)
        
        # 订阅器
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.initial_pose_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.initial_pose_callback)
        
        rospy.loginfo("鲁棒激光雷达定位算法启动")

    def map_callback(self, msg):
        """地图回调函数"""
        self.map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.map_info = msg.info
        rospy.loginfo("接收到地图数据: {}x{}, 分辨率: {}".format(
            msg.info.width, msg.info.height, msg.info.resolution))

    def initial_pose_callback(self, msg):
        """初始位姿回调"""
        self.last_pose = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'yaw': tf_trans.euler_from_quaternion([
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ])[2],
            'timestamp': msg.header.stamp,
            'confidence': 1.0
        }
        rospy.loginfo("设置初始位姿: x={:.2f}, y={:.2f}, yaw={:.2f}".format(
            self.last_pose['x'], self.last_pose['y'], self.last_pose['yaw']))

    def scan_callback(self, msg):
        """激光雷达数据回调"""
        if self.map_data is None or self.last_pose is None:
            return

        try:
            # 进行鲁棒匹配
            new_pose, confidence = self.robust_scan_matching(msg, self.last_pose)
            
            # 一致性检查
            if self.is_pose_consistent(new_pose):
                # 更新位姿
                self.last_pose = new_pose
                self.pose_history.append(new_pose)
                self.confidence_history.append(confidence)
                
                # 发布TF变换
                self.publish_transform(new_pose)
                
                # 发布位姿
                self.publish_pose(new_pose, confidence)
                
                rospy.loginfo_throttle(2.0, "定位: x={:.2f}, y={:.2f}, yaw={:.2f}, 置信度={:.2f}".format(
                    new_pose['x'], new_pose['y'], new_pose['yaw'], confidence))
            else:
                rospy.logwarn("位姿不一致，拒绝更新")
                
        except Exception as e:
            rospy.logwarn("鲁棒定位失败: {}".format(e))

    def robust_scan_matching(self, scan_msg, current_pose):
        """鲁棒的激光扫描匹配"""
        # 将激光数据转换为地图坐标系下的点
        scan_points = self.scan_to_map_points(scan_msg, current_pose)
        
        # 分离内点和外点
        inliers, outliers = self.separate_inliers_outliers(scan_points)
        
        # 计算内点比例
        total_points = len(scan_points)
        inlier_ratio = len(inliers) / total_points if total_points > 0 else 0
        
        rospy.loginfo_throttle(5.0, "激光点: 总数={}, 内点={}, 外点={}, 内点比例={:.2f}".format(
            total_points, len(inliers), len(outliers), inlier_ratio))
        
        # 如果内点比例太低，降低置信度但仍尝试匹配
        if inlier_ratio < self.min_inlier_ratio:
            confidence = inlier_ratio
            rospy.logwarn_throttle(2.0, "内点比例过低 ({:.2f})，可能存在大量动态障碍物".format(inlier_ratio))
        else:
            confidence = min(1.0, inlier_ratio + 0.2)
        
        # 使用内点进行匹配优化
        if len(inliers) > 10:  # 确保有足够的内点
            optimized_pose = self.optimize_pose_with_inliers(inliers, current_pose)
        else:
            rospy.logwarn("内点数量不足，使用当前位姿")
            optimized_pose = current_pose.copy()
            confidence = 0.1
        
        optimized_pose['confidence'] = confidence
        optimized_pose['timestamp'] = scan_msg.header.stamp
        
        return optimized_pose, confidence

    def scan_to_map_points(self, scan_msg, pose):
        """将激光扫描转换为地图坐标系下的点"""
        points = []
        angle = scan_msg.angle_min
        
        try:
            # 获取激光雷达到base_link的变换
            laser_to_base = self.tf_buffer.lookup_transform(
                self.base_frame, self.laser_frame, scan_msg.header.stamp, rospy.Duration(0.1))
            
            # 提取变换参数
            laser_offset_x = laser_to_base.transform.translation.x
            laser_offset_y = laser_to_base.transform.translation.y
            laser_rotation = laser_to_base.transform.rotation
            laser_euler = tf_trans.euler_from_quaternion([
                laser_rotation.x, laser_rotation.y, laser_rotation.z, laser_rotation.w])
            laser_yaw = laser_euler[2]
            
        except Exception as e:
            rospy.logwarn_throttle(5.0, "无法获取激光雷达TF变换，使用零偏移: {}".format(e))
            laser_offset_x = 0.0
            laser_offset_y = 0.0
            laser_yaw = 0.0
        
        for range_val in scan_msg.ranges:
            if scan_msg.range_min <= range_val <= scan_msg.range_max:
                # 1. 激光雷达坐标系下的点
                x_laser = range_val * math.cos(angle)
                y_laser = range_val * math.sin(angle)
                
                # 2. 转换到base_link坐标系
                cos_laser_yaw = math.cos(laser_yaw)
                sin_laser_yaw = math.sin(laser_yaw)
                
                x_base = laser_offset_x + x_laser * cos_laser_yaw - y_laser * sin_laser_yaw
                y_base = laser_offset_y + x_laser * sin_laser_yaw + y_laser * cos_laser_yaw
                
                # 3. 转换到地图坐标系
                cos_robot_yaw = math.cos(pose['yaw'])
                sin_robot_yaw = math.sin(pose['yaw'])
                
                x_map = pose['x'] + x_base * cos_robot_yaw - y_base * sin_robot_yaw
                y_map = pose['y'] + x_base * sin_robot_yaw + y_base * cos_robot_yaw
                
                points.append([x_map, y_map])
            
            angle += scan_msg.angle_increment
        
        return np.array(points)

    def separate_inliers_outliers(self, points):
        """分离内点和外点"""
        inliers = []
        outliers = []
        
        for point in points:
            x_map, y_map = point
            
            # 转换为栅格坐标
            grid_x = int((x_map - self.map_info.origin.position.x) / self.map_info.resolution)
            grid_y = int((y_map - self.map_info.origin.position.y) / self.map_info.resolution)
            
            # 检查是否在地图范围内
            if (0 <= grid_x < self.map_data.shape[1] and 
                0 <= grid_y < self.map_data.shape[0]):
                
                # 检查该点是否与地图一致
                if self.is_point_consistent_with_map(grid_x, grid_y):
                    inliers.append(point)
                else:
                    outliers.append(point)
            else:
                outliers.append(point)  # 地图外的点视为外点
        
        return np.array(inliers) if inliers else np.empty((0, 2)), np.array(outliers) if outliers else np.empty((0, 2))

    def is_point_consistent_with_map(self, grid_x, grid_y, radius=2):
        """检查点是否与地图一致"""
        # 检查该点周围是否有静态障碍物
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                check_x = grid_x + dx
                check_y = grid_y + dy
                
                if (0 <= check_x < self.map_data.shape[1] and 
                    0 <= check_y < self.map_data.shape[0]):
                    
                    cell_value = self.map_data[check_y, check_x]
                    if cell_value == 100:  # 找到静态障碍物
                        return True
        
        # 如果周围没有静态障碍物，但当前位置应该是自由空间
        current_cell = self.map_data[grid_y, grid_x]
        if current_cell == 0:  # 自由空间中出现激光点，可能是动态障碍物
            return False
        
        return True  # 未知区域或其他情况，保守处理

    def optimize_pose_with_inliers(self, inliers, current_pose):
        """使用内点优化位姿"""
        if len(inliers) < 10:
            return current_pose
        
        # 简化的优化：在当前位姿附近搜索最佳匹配
        best_pose = current_pose.copy()
        best_score = self.calculate_matching_score(inliers, current_pose)
        
        # 搜索范围
        search_steps = 5
        position_step = 0.1  # 10cm步长
        angle_step = 0.05    # 约3度步长
        
        for dx in np.linspace(-0.5, 0.5, search_steps):
            for dy in np.linspace(-0.5, 0.5, search_steps):
                for dyaw in np.linspace(-0.15, 0.15, search_steps):
                    test_pose = current_pose.copy()
                    test_pose['x'] += dx
                    test_pose['y'] += dy
                    test_pose['yaw'] += dyaw
                    
                    score = self.calculate_matching_score(inliers, test_pose)
                    if score > best_score:
                        best_score = score
                        best_pose = test_pose
        
        return best_pose

    def calculate_matching_score(self, points, pose):
        """计算匹配分数"""
        score = 0
        total_points = len(points)
        
        if total_points == 0:
            return 0
        
        for point in points:
            x_map, y_map = point
            
            # 转换为栅格坐标
            grid_x = int((x_map - self.map_info.origin.position.x) / self.map_info.resolution)
            grid_y = int((y_map - self.map_info.origin.position.y) / self.map_info.resolution)
            
            if (0 <= grid_x < self.map_data.shape[1] and 
                0 <= grid_y < self.map_data.shape[0]):
                
                cell_value = self.map_data[grid_y, grid_x]
                if cell_value == 100:  # 障碍物
                    score += 1
                elif cell_value == -1:  # 未知区域
                    score += 0.5
        
        return score / total_points if total_points > 0 else 0

    def is_pose_consistent(self, new_pose):
        """检查新位姿是否一致"""
        if not self.pose_history:
            return True
        
        last_pose = self.pose_history[-1]
        
        # 检查位置跳变
        position_change = math.sqrt(
            (new_pose['x'] - last_pose['x']) ** 2 + 
            (new_pose['y'] - last_pose['y']) ** 2)
        
        # 检查角度跳变
        angle_change = abs(new_pose['yaw'] - last_pose['yaw'])
        if angle_change > math.pi:
            angle_change = 2 * math.pi - angle_change
        
        return (position_change < self.max_position_jump and 
                angle_change < self.max_angle_jump)

    def publish_transform(self, pose):
        """发布TF变换"""
        try:
            # 获取odom到base_link的变换
            odom_to_base = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, rospy.Time.now(), rospy.Duration(0.1))
            
            # 计算map到odom的变换
            # TF链: map -> odom -> base_link
            # 已知: map -> base_link (定位结果) 和 odom -> base_link (里程计)
            # 求: map -> odom
            
            # 提取odom到base的位姿
            odom_x = odom_to_base.transform.translation.x
            odom_y = odom_to_base.transform.translation.y
            odom_quat = [
                odom_to_base.transform.rotation.x,
                odom_to_base.transform.rotation.y,
                odom_to_base.transform.rotation.z,
                odom_to_base.transform.rotation.w
            ]
            odom_euler = tf_trans.euler_from_quaternion(odom_quat)
            odom_yaw = odom_euler[2]
            
            # 计算map到odom的变换
            # map_to_odom = map_to_base * inverse(odom_to_base)
            cos_odom = math.cos(odom_yaw)
            sin_odom = math.sin(odom_yaw)
            cos_map = math.cos(pose['yaw'])
            sin_map = math.sin(pose['yaw'])
            
            # 位置变换
            map_to_odom_x = pose['x'] - (cos_map * odom_x - sin_map * odom_y)
            map_to_odom_y = pose['y'] - (sin_map * odom_x + cos_map * odom_y)
            map_to_odom_yaw = pose['yaw'] - odom_yaw
            
            # 标准化角度
            while map_to_odom_yaw > math.pi:
                map_to_odom_yaw -= 2 * math.pi
            while map_to_odom_yaw < -math.pi:
                map_to_odom_yaw += 2 * math.pi
            
            # 创建并发布TF变换
            map_to_odom = TransformStamped()
            map_to_odom.header.stamp = rospy.Time.now()
            map_to_odom.header.frame_id = self.map_frame
            map_to_odom.child_frame_id = self.odom_frame
            
            map_to_odom.transform.translation.x = map_to_odom_x
            map_to_odom.transform.translation.y = map_to_odom_y
            map_to_odom.transform.translation.z = 0.0
            
            map_odom_quat = tf_trans.quaternion_from_euler(0, 0, map_to_odom_yaw)
            map_to_odom.transform.rotation.x = map_odom_quat[0]
            map_to_odom.transform.rotation.y = map_odom_quat[1]
            map_to_odom.transform.rotation.z = map_odom_quat[2]
            map_to_odom.transform.rotation.w = map_odom_quat[3]
            
            self.tf_broadcaster.sendTransform(map_to_odom)
            
        except Exception as e:
            rospy.logwarn_throttle(1.0, "发布TF失败: {}".format(e))

    def publish_pose(self, pose, confidence):
        """发布位姿"""
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = self.map_frame
        
        pose_msg.pose.pose.position.x = pose['x']
        pose_msg.pose.pose.position.y = pose['y']
        pose_msg.pose.pose.position.z = 0.0
        
        quat = tf_trans.quaternion_from_euler(0, 0, pose['yaw'])
        pose_msg.pose.pose.orientation.x = quat[0]
        pose_msg.pose.pose.orientation.y = quat[1]
        pose_msg.pose.pose.orientation.z = quat[2]
        pose_msg.pose.pose.orientation.w = quat[3]
        
        # 根据置信度设置协方差
        base_covariance = 0.1 * (1.0 - confidence) + 0.01
        pose_msg.pose.covariance[0] = base_covariance      # x
        pose_msg.pose.covariance[7] = base_covariance      # y
        pose_msg.pose.covariance[35] = base_covariance * 2  # yaw
        
        self.pose_pub.publish(pose_msg)

if __name__ == '__main__':
    try:
        robust_loc = RobustLidarLoc()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("鲁棒激光雷达定位算法关闭")
