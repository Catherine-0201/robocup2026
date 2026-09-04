#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
改进的激光雷达定位算法
基于原始算法框架，增加动态障碍物过滤能力
保持原有的精度和TF发布逻辑
"""

import rospy
import tf2_ros
import numpy as np
import math
import cv2
from collections import deque

# ROS消息类型
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
import tf.transformations as tf_trans

class ImprovedLidarLoc:
    def __init__(self):
        rospy.init_node('improved_lidar_loc', anonymous=False)
        
        # 参数配置
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param('~base_frame', 'base_footprint')
        self.laser_frame = rospy.get_param('~laser_frame', 'laser')
        
        # 滤波参数
        self.outlier_filter_enabled = rospy.get_param('~outlier_filter_enabled', True)
        self.outlier_ratio_threshold = rospy.get_param('~outlier_ratio_threshold', 0.3)  # 外点比例阈值
        self.consistency_check_radius = rospy.get_param('~consistency_check_radius', 2)  # 一致性检查半径
        
        # 匹配参数（沿用原始算法的成功设置）
        self.deg_to_rad = math.pi / 180.0
        self.search_offsets = [[0,0], [1,0], [-1,0], [0,1], [0,-1]]  # 像原算法一样的搜索偏移
        self.angle_offsets = [0, self.deg_to_rad, -self.deg_to_rad]  # 角度搜索
        
        # 状态变量
        self.map_data = None
        self.map_info = None
        self.map_cropped = None
        self.map_temp = None
        self.map_roi_info = {'x_offset': 0, 'y_offset': 0}
        
        self.current_pose = {'x': 250, 'y': 250, 'yaw': 0}  # 像原算法一样的初始位置
        self.scan_count = 0
        
        # 收敛检查机制（模拟原始算法的check函数）
        self.pose_history = []  # 位姿历史记录
        self.max_history_size = 10  # 最大历史记录数量
        self.convergence_threshold = rospy.get_param('~convergence_threshold', 3)  # 收敛阈值（栅格单位）
        self.angle_convergence_threshold = self.convergence_threshold * self.deg_to_rad  # 角度收敛阈值
        
        # 运动预测和平滑（提高移动时的稳定性）
        self.last_pose = None
        self.last_timestamp = None
        self.velocity_filter_window = rospy.get_param('~velocity_filter_window', 3)
        self.velocity_history = []  # 速度历史
        self.max_position_change_per_frame = rospy.get_param('~max_position_change_per_frame', 8)
        self.max_angle_change_per_frame = rospy.get_param('~max_angle_change_per_frame', 8) * self.deg_to_rad
        self.motion_prediction_enabled = rospy.get_param('~motion_prediction_enabled', True)
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # 发布器
        self.pose_pub = rospy.Publisher('/improved_pose', PoseWithCovarianceStamped, queue_size=1)
        
        # 订阅器
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.initial_pose_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.initial_pose_callback)
        
        # 定时发布TF
        self.tf_timer = rospy.Timer(rospy.Duration(1.0/50.0), self.publish_tf_timer)  # 50Hz
        
        rospy.loginfo("改进的激光雷达定位算法启动")

    def map_callback(self, msg):
        """地图回调函数（参考原始算法）"""
        self.map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        self.map_info = msg.info
        
        # 裁剪地图（类似原始算法的crop_map功能）
        self.crop_map()
        self.process_map()
        
        rospy.loginfo("地图处理完成: {}x{}, 分辨率: {}".format(
            msg.info.width, msg.info.height, msg.info.resolution))

    def crop_map(self):
        """裁剪地图（简化版本）"""
        if self.map_data is None:
            return
        
        # 找到所有障碍物的边界
        obstacle_points = np.where(self.map_data == 100)
        if len(obstacle_points[0]) == 0:
            rospy.logwarn("地图中没有找到障碍物")
            return
        
        min_y, max_y = np.min(obstacle_points[0]), np.max(obstacle_points[0])
        min_x, max_x = np.min(obstacle_points[1]), np.max(obstacle_points[1])
        
        # 添加边距
        margin = 50
        crop_min_x = max(0, min_x - margin)
        crop_max_x = min(self.map_data.shape[1], max_x + margin)
        crop_min_y = max(0, min_y - margin)
        crop_max_y = min(self.map_data.shape[0], max_y + margin)
        
        # 裁剪地图
        self.map_cropped = self.map_data[crop_min_y:crop_max_y, crop_min_x:crop_max_x].copy()
        self.map_roi_info = {
            'x_offset': crop_min_x,
            'y_offset': crop_min_y,
            'width': crop_max_x - crop_min_x,
            'height': crop_max_y - crop_min_y
        }

    def process_map(self):
        """处理地图（简化版本的梯度）"""
        if self.map_cropped is None:
            return
        
        self.map_temp = self.map_cropped.copy()
        
        # 为障碍物周围添加梯度（类似原始算法）
        kernel = np.ones((5, 5), np.uint8)
        obstacles = (self.map_cropped == 100).astype(np.uint8) * 255
        gradient = cv2.dilate(obstacles, kernel, iterations=1)
        
        # 将梯度应用到map_temp
        self.map_temp = np.maximum(self.map_temp, gradient // 3)  # 减弱梯度强度

    def initial_pose_callback(self, msg):
        """初始位姿回调"""
        if self.map_info is None:
            rospy.logwarn("地图信息无效，无法设置初始位姿")
            return
        
        # 转换到地图栅格坐标
        map_x = (msg.pose.pose.position.x - self.map_info.origin.position.x) / self.map_info.resolution
        map_y = (msg.pose.pose.position.y - self.map_info.origin.position.y) / self.map_info.resolution
        
        # 转换到裁剪地图坐标
        self.current_pose['x'] = map_x - self.map_roi_info['x_offset']
        self.current_pose['y'] = map_y - self.map_roi_info['y_offset']
        
        # 提取yaw角
        quat = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        euler = tf_trans.euler_from_quaternion(quat)
        self.current_pose['yaw'] = -euler[2]  # 注意符号，匹配原始算法
        
        rospy.loginfo("设置初始位姿: x={:.1f}, y={:.1f}, yaw={:.2f}".format(
            self.current_pose['x'], self.current_pose['y'], self.current_pose['yaw']))

    def scan_callback(self, msg):
        """激光扫描回调（改进的匹配算法，增加运动预测）"""
        if self.map_temp is None:
            return
        
        self.scan_count += 1
        current_time = msg.header.stamp.to_sec()
        
        try:
            # 获取激光雷达到base_link的变换
            laser_to_base = self.tf_buffer.lookup_transform(
                self.base_frame, self.laser_frame, msg.header.stamp, rospy.Duration(0.1))
        except Exception as e:
            rospy.logwarn_throttle(5.0, "无法获取激光雷达TF: {}".format(e))
            return
        
        # 运动预测（基于速度预测下一帧位置）
        if self.motion_prediction_enabled and self.last_pose is not None and self.last_timestamp is not None:
            dt = current_time - self.last_timestamp
            if dt > 0 and dt < 1.0:  # 合理的时间间隔
                self.predict_motion(dt)
        
        # 转换激光数据到base_link坐标系的点
        scan_points = self.convert_scan_to_points(msg, laser_to_base)
        
        # 应用动态障碍物过滤
        if self.outlier_filter_enabled:
            filtered_points, outlier_ratio = self.filter_outliers(scan_points)
            
            if outlier_ratio > self.outlier_ratio_threshold:
                rospy.loginfo_throttle(3.0, "检测到{:.1f}%外点，使用过滤后数据进行匹配".format(outlier_ratio * 100))
            
            # 如果过滤后点数太少，使用原始数据
            if len(filtered_points) < len(scan_points) * 0.5:
                rospy.logwarn_throttle(5.0, "过滤后点数过少，使用原始数据")
                match_points = scan_points
            else:
                match_points = filtered_points
        else:
            match_points = scan_points
        
        # 保存匹配前的位姿
        pose_before_match = self.current_pose.copy()
        
        # 进行位姿匹配（基于原始算法的逻辑）
        self.match_pose_with_points(match_points)
        
        # 位姿平滑和异常检测
        self.smooth_pose_update(pose_before_match, current_time)
        
        # 更新运动历史
        self.update_motion_history(current_time)

    def convert_scan_to_points(self, scan_msg, laser_to_base_tf):
        """转换激光扫描为栅格坐标（完全按照原始算法）"""
        points = []
        angle = scan_msg.angle_min
        
        for range_val in scan_msg.ranges:
            if scan_msg.range_min <= range_val <= scan_msg.range_max:
                # 1. 在激光雷达坐标系下计算点的坐标（完全按原始算法）
                x_laser = range_val * math.cos(angle)
                y_laser = -range_val * math.sin(angle)  # 注意负号！
                
                # 2. 使用tf2转换到base_link坐标系（模拟原始算法的tf2::doTransform）
                try:
                    import tf2_geometry_msgs
                    from geometry_msgs.msg import PointStamped
                    
                    point_laser = PointStamped()
                    point_laser.header.frame_id = self.laser_frame
                    point_laser.header.stamp = scan_msg.header.stamp
                    point_laser.point.x = x_laser
                    point_laser.point.y = y_laser
                    point_laser.point.z = 0.0
                    
                    # 使用tf2进行精确变换
                    point_base = tf2_geometry_msgs.do_transform_point(point_laser, laser_to_base_tf)
                    
                    # 3. 转换为栅格地图坐标并存储（完全按原始算法）
                    x_grid = point_base.point.x / self.map_info.resolution
                    y_grid = point_base.point.y / self.map_info.resolution
                    
                except ImportError:
                    # 备用方案：手动计算变换
                    laser_x = laser_to_base_tf.transform.translation.x
                    laser_y = laser_to_base_tf.transform.translation.y
                    laser_quat = [
                        laser_to_base_tf.transform.rotation.x,
                        laser_to_base_tf.transform.rotation.y,
                        laser_to_base_tf.transform.rotation.z,
                        laser_to_base_tf.transform.rotation.w
                    ]
                    laser_euler = tf_trans.euler_from_quaternion(laser_quat)
                    laser_yaw = laser_euler[2]
                    
                    cos_yaw = math.cos(laser_yaw)
                    sin_yaw = math.sin(laser_yaw)
                    
                    x_base = laser_x + x_laser * cos_yaw - y_laser * sin_yaw
                    y_base = laser_y + x_laser * sin_yaw + y_laser * cos_yaw
                    
                    x_grid = x_base / self.map_info.resolution
                    y_grid = y_base / self.map_info.resolution
                
                points.append([x_grid, y_grid])
            
            angle += scan_msg.angle_increment
        
        return np.array(points)

    def filter_outliers(self, points):
        """过滤外点（动态障碍物）"""
        if len(points) == 0:
            return points, 0.0
        
        inliers = []
        outliers = []
        
        for point in points:
            x_grid, y_grid = point
            
            # 转换到裁剪地图坐标
            x_crop = x_grid
            y_crop = y_grid
            
            if self.is_point_consistent_with_map(x_crop, y_crop):
                inliers.append(point)
            else:
                outliers.append(point)
        
        total_points = len(points)
        outlier_ratio = len(outliers) / total_points if total_points > 0 else 0.0
        
        return np.array(inliers) if inliers else np.empty((0, 2)), outlier_ratio

    def is_point_consistent_with_map(self, x, y):
        """检查点是否与地图一致"""
        # 转换到地图坐标
        map_x = int(x + self.map_roi_info['x_offset'])
        map_y = int(y + self.map_roi_info['y_offset'])
        
        # 检查边界
        if not (0 <= map_x < self.map_data.shape[1] and 0 <= map_y < self.map_data.shape[0]):
            return False
        
        # 在周围区域搜索障碍物
        radius = self.consistency_check_radius
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                check_x = map_x + dx
                check_y = map_y + dy
                
                if (0 <= check_x < self.map_data.shape[1] and 
                    0 <= check_y < self.map_data.shape[0]):
                    
                    if self.map_data[check_y, check_x] == 100:  # 静态障碍物
                        return True
        
        # 如果激光点在自由空间中，可能是动态障碍物
        current_cell = self.map_data[map_y, map_x]
        return current_cell != 0  # 不是自由空间

    def match_pose_with_points(self, points):
        """使用点进行位姿匹配（完全模拟原始算法的迭代收敛逻辑）"""
        if len(points) == 0:
            return
        
        # 迭代收敛循环（模拟原始算法的while循环）
        max_iterations = 50  # 防止无限循环
        iteration = 0
        
        while iteration < max_iterations:
            iteration += 1
            
            # 生成三种角度的点集（类似原始算法）
            transform_points = self.transform_points_to_map(points, 0)
            clockwise_points = self.transform_points_to_map(points, self.deg_to_rad)
            counter_points = self.transform_points_to_map(points, -self.deg_to_rad)
            
            point_sets = [transform_points, clockwise_points, counter_points]
            
            # 寻找最佳匹配
            max_score = 0
            best_dx, best_dy, best_dyaw = 0, 0, 0
            
            for i, offset in enumerate(self.search_offsets):
                for j, point_set in enumerate(point_sets):
                    score = self.calculate_match_score(point_set, offset)
                    
                    if score > max_score:
                        max_score = score
                        best_dx = offset[0]
                        best_dy = offset[1]
                        best_dyaw = self.angle_offsets[j]
            
            # 更新位姿
            self.current_pose['x'] += best_dx
            self.current_pose['y'] += best_dy
            self.current_pose['yaw'] += best_dyaw
            
            # 检查是否收敛（模拟原始算法的check函数）
            if self.check_convergence():
                rospy.loginfo_throttle(2.0, "位姿收敛: x={:.1f}, y={:.1f}, yaw={:.2f}, 迭代次数={}, 分数={}".format(
                    self.current_pose['x'], self.current_pose['y'], self.current_pose['yaw'], iteration, max_score))
                break
        
        if iteration >= max_iterations:
            rospy.logwarn_throttle(5.0, "位姿匹配未收敛，达到最大迭代次数: {}".format(max_iterations))

    def transform_points_to_map(self, points, angle_offset):
        """将点变换到地图坐标（完全按照原始算法的公式）"""
        transformed = []
        yaw = self.current_pose['yaw'] + angle_offset  # lidar_yaw + offset
        
        for point in points:
            x, y = point
            
            # 完全按照原始算法的公式：
            # rotated_x = point.x * cos(lidar_yaw) - point.y * sin(lidar_yaw);
            # rotated_y = point.x * sin(lidar_yaw) + point.y * cos(lidar_yaw);
            # transform_points.push_back(cv::Point2f(rotated_x + lidar_x, lidar_y - rotated_y));
            
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            
            rotated_x = x * cos_yaw - y * sin_yaw
            rotated_y = x * sin_yaw + y * cos_yaw
            
            # 关键：使用原始算法的公式！
            x_map = rotated_x + self.current_pose['x']  # rotated_x + lidar_x
            y_map = self.current_pose['y'] - rotated_y  # lidar_y - rotated_y
            
            transformed.append([x_map, y_map])
        
        return transformed

    def calculate_match_score(self, points, offset):
        """计算匹配分数"""
        score = 0
        
        for point in points:
            x = int(point[0] + offset[0])
            y = int(point[1] + offset[1])
            
            if (0 <= x < self.map_temp.shape[1] and 0 <= y < self.map_temp.shape[0]):
                score += self.map_temp[y, x]
        
        return score
    
    def check_convergence(self):
        """检查位姿是否收敛（模拟原始算法的check函数）"""
        current_pose_tuple = (self.current_pose['x'], self.current_pose['y'], self.current_pose['yaw'])
        
        # 添加当前位姿到历史记录
        self.pose_history.append(current_pose_tuple)
        
        # 如果历史记录超过最大大小，移除最旧的数据
        if len(self.pose_history) > self.max_history_size:
            self.pose_history.pop(0)
        
        # 如果历史记录已满，检查收敛性
        if len(self.pose_history) == self.max_history_size:
            first_pose = self.pose_history[0]
            last_pose = self.pose_history[-1]
            
            dx = abs(last_pose[0] - first_pose[0])
            dy = abs(last_pose[1] - first_pose[1])
            dyaw = abs(last_pose[2] - first_pose[2])
            
            # 如果所有差值都小于阈值，认为已收敛
            if (dx < self.convergence_threshold and 
                dy < self.convergence_threshold and 
                dyaw < self.angle_convergence_threshold):
                
                self.pose_history.clear()  # 清空历史记录
                return True
        
        return False
    
    def predict_motion(self, dt):
        """基于历史速度预测下一帧位置"""
        if len(self.velocity_history) == 0:
            return
        
        # 计算平均速度
        avg_vx = sum([v[0] for v in self.velocity_history]) / len(self.velocity_history)
        avg_vy = sum([v[1] for v in self.velocity_history]) / len(self.velocity_history)
        avg_vyaw = sum([v[2] for v in self.velocity_history]) / len(self.velocity_history)
        
        # 预测位置（小幅预测，提高初值）
        prediction_scale = 0.8  # 预测缩放因子，避免过度预测
        
        self.current_pose['x'] += avg_vx * dt * prediction_scale
        self.current_pose['y'] += avg_vy * dt * prediction_scale
        self.current_pose['yaw'] += avg_vyaw * dt * prediction_scale
        
        rospy.logdebug("运动预测: vx={:.2f}, vy={:.2f}, vyaw={:.2f}".format(avg_vx, avg_vy, avg_vyaw))
    
    def smooth_pose_update(self, pose_before_match, current_time):
        """平滑位姿更新，防止突跳"""
        # 计算位姿变化
        dx = self.current_pose['x'] - pose_before_match['x']
        dy = self.current_pose['y'] - pose_before_match['y']
        dyaw = self.current_pose['yaw'] - pose_before_match['yaw']
        
        # 标准化角度差
        while dyaw > math.pi:
            dyaw -= 2 * math.pi
        while dyaw < -math.pi:
            dyaw += 2 * math.pi
        
        # 检查是否超过合理范围
        position_change = math.sqrt(dx*dx + dy*dy)
        angle_change = abs(dyaw)
        
        # 如果变化过大，进行限制
        if position_change > self.max_position_change_per_frame:
            scale = self.max_position_change_per_frame / position_change
            dx *= scale
            dy *= scale
            rospy.logwarn_throttle(2.0, "位置变化过大，已限制: {:.1f} -> {:.1f}".format(
                position_change, self.max_position_change_per_frame))
        
        if angle_change > self.max_angle_change_per_frame:
            scale = self.max_angle_change_per_frame / angle_change
            dyaw *= scale
            rospy.logwarn_throttle(2.0, "角度变化过大，已限制: {:.3f} -> {:.3f}".format(
                angle_change, self.max_angle_change_per_frame))
        
        # 应用限制后的变化
        self.current_pose['x'] = pose_before_match['x'] + dx
        self.current_pose['y'] = pose_before_match['y'] + dy
        self.current_pose['yaw'] = pose_before_match['yaw'] + dyaw
    
    def update_motion_history(self, current_time):
        """更新运动历史，用于速度计算"""
        if self.last_pose is not None and self.last_timestamp is not None:
            dt = current_time - self.last_timestamp
            
            if dt > 0 and dt < 1.0:  # 合理的时间间隔
                # 计算速度
                vx = (self.current_pose['x'] - self.last_pose['x']) / dt
                vy = (self.current_pose['y'] - self.last_pose['y']) / dt
                vyaw = (self.current_pose['yaw'] - self.last_pose['yaw']) / dt
                
                # 添加到速度历史
                self.velocity_history.append((vx, vy, vyaw))
                
                # 限制历史记录大小
                if len(self.velocity_history) > self.velocity_filter_window:
                    self.velocity_history.pop(0)
        
        # 更新历史
        self.last_pose = self.current_pose.copy()
        self.last_timestamp = current_time

    def publish_tf_timer(self, event):
        """定时发布TF变换（完全按照原始算法逻辑）"""
        if self.scan_count == 0 or self.map_info is None:
            return
        
        try:
            # 尝试导入tf2_py，如果失败使用备用方案
            try:
                import tf2_py as tf2
            except ImportError:
                raise Exception("tf2_py不可用，使用简化版本")
            
            # 步骤1: 将机器人位姿从裁切地图像素坐标转换为完整地图米制坐标
            full_map_pixel_x = self.current_pose['x'] + self.map_roi_info['x_offset']
            full_map_pixel_y = self.current_pose['y'] + self.map_roi_info['y_offset']
            
            x_in_map_frame = full_map_pixel_x * self.map_info.resolution + self.map_info.origin.position.x
            y_in_map_frame = full_map_pixel_y * self.map_info.resolution + self.map_info.origin.position.y
            yaw_in_map_frame = -self.current_pose['yaw']  # 补偿匹配过程中的坐标系定义
            
            # 步骤2: 构建从map到base_frame的变换
            map_to_base = tf2.Transform()
            map_to_base.setOrigin(tf2.Vector3(x_in_map_frame, y_in_map_frame, 0.0))
            q = tf2.Quaternion()
            q.setRPY(0, 0, yaw_in_map_frame)
            map_to_base.setRotation(q)
            
            # 步骤3: 查询从odom到base_frame的变换
            odom_to_base_msg = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.1))
            
            # 步骤4: 计算从map到odom的变换
            # T_map_odom = T_map_base * (T_odom_base)^-1
            odom_to_base_tf2 = tf2.Transform()
            odom_to_base_tf2.setOrigin(tf2.Vector3(
                odom_to_base_msg.transform.translation.x,
                odom_to_base_msg.transform.translation.y,
                odom_to_base_msg.transform.translation.z
            ))
            odom_to_base_tf2.setRotation(tf2.Quaternion(
                odom_to_base_msg.transform.rotation.x,
                odom_to_base_msg.transform.rotation.y,
                odom_to_base_msg.transform.rotation.z,
                odom_to_base_msg.transform.rotation.w
            ))
            
            map_to_odom_tf2 = map_to_base * odom_to_base_tf2.inverse()
            
            # 步骤5: 发布map->odom的变换
            map_to_odom_msg = TransformStamped()
            map_to_odom_msg.header.stamp = rospy.Time.now()
            map_to_odom_msg.header.frame_id = self.map_frame
            map_to_odom_msg.child_frame_id = self.odom_frame
            
            # 转换tf2变换为ROS消息格式
            origin = map_to_odom_tf2.getOrigin()
            rotation = map_to_odom_tf2.getRotation()
            
            map_to_odom_msg.transform.translation.x = origin.x()
            map_to_odom_msg.transform.translation.y = origin.y()
            map_to_odom_msg.transform.translation.z = origin.z()
            
            map_to_odom_msg.transform.rotation.x = rotation.x()
            map_to_odom_msg.transform.rotation.y = rotation.y()
            map_to_odom_msg.transform.rotation.z = rotation.z()
            map_to_odom_msg.transform.rotation.w = rotation.w()
            
            self.tf_broadcaster.sendTransform(map_to_odom_msg)
            
        except Exception as e:
            # 如果tf2_py不可用，使用简化版本
            try:
                self.publish_tf_simplified()
            except Exception as e2:
                rospy.logwarn_throttle(5.0, "TF发布失败: {} / {}".format(e, e2))
                
    def publish_tf_simplified(self):
        """简化版TF发布（备用方案）"""
        # 获取odom到base_link的变换
        odom_to_base = self.tf_buffer.lookup_transform(
            self.odom_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.1))
        
        # 将当前位姿从栅格坐标转换为米制坐标
        full_map_x = self.current_pose['x'] + self.map_roi_info['x_offset']
        full_map_y = self.current_pose['y'] + self.map_roi_info['y_offset']
        
        x_meters = full_map_x * self.map_info.resolution + self.map_info.origin.position.x
        y_meters = full_map_y * self.map_info.resolution + self.map_info.origin.position.y
        yaw_radians = -self.current_pose['yaw']  # 注意符号转换
        
        # 使用齐次变换矩阵计算map到odom的变换
        # T_map_base
        cos_yaw = math.cos(yaw_radians)
        sin_yaw = math.sin(yaw_radians)
        T_map_base = np.array([
            [cos_yaw, -sin_yaw, x_meters],
            [sin_yaw,  cos_yaw, y_meters],
            [0,        0,       1]
        ])
        
        # T_odom_base
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
        
        cos_odom = math.cos(odom_yaw)
        sin_odom = math.sin(odom_yaw)
        T_odom_base = np.array([
            [cos_odom, -sin_odom, odom_x],
            [sin_odom,  cos_odom, odom_y],
            [0,         0,        1]
        ])
        
        # T_map_odom = T_map_base * inv(T_odom_base)
        T_base_odom = np.linalg.inv(T_odom_base)
        T_map_odom = np.dot(T_map_base, T_base_odom)
        
        # 提取位置和角度
        map_odom_x = T_map_odom[0, 2]
        map_odom_y = T_map_odom[1, 2]
        map_odom_yaw = math.atan2(T_map_odom[1, 0], T_map_odom[0, 0])
        
        # 发布变换
        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = rospy.Time.now()
        map_to_odom.header.frame_id = self.map_frame
        map_to_odom.child_frame_id = self.odom_frame
        
        map_to_odom.transform.translation.x = map_odom_x
        map_to_odom.transform.translation.y = map_odom_y
        map_to_odom.transform.translation.z = 0.0
        
        map_odom_quat = tf_trans.quaternion_from_euler(0, 0, map_odom_yaw)
        map_to_odom.transform.rotation.x = map_odom_quat[0]
        map_to_odom.transform.rotation.y = map_odom_quat[1]
        map_to_odom.transform.rotation.z = map_odom_quat[2]
        map_to_odom.transform.rotation.w = map_odom_quat[3]
        
        self.tf_broadcaster.sendTransform(map_to_odom)


if __name__ == '__main__':
    try:
        improved_loc = ImprovedLidarLoc()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("改进激光雷达定位算法关闭")
