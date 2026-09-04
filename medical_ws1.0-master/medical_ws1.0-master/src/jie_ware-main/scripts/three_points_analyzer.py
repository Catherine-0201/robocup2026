#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
import numpy as np
import math

class ThreePointsAnalyzer:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('three_points_analyzer', anonymous=True)
        
        # 从参数服务器读取配置参数
        self.detection_radius = rospy.get_param('~detection_radius', 0.20)  # 20厘米检测半径
        self.min_points = rospy.get_param('~min_points', 3)  # 最少需要检测到3个点
        self.wall_parallel_threshold = rospy.get_param('~wall_parallel_threshold', 10.0)  # 角度阈值（度）
        self.log_throttle_period = rospy.get_param('~log_throttle_period', 1.0)
        
        # 发布器
        self.result_pub = rospy.Publisher('/three_points_analysis_result', String, queue_size=10)
        self.is_parallel_pub = rospy.Publisher('/is_parallel_to_wall', Bool, queue_size=10)
        
        # 订阅原始激光雷达数据（过滤前的数据）
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
        rospy.loginfo("三点分析节点已启动，检测半径: {:.2f}m".format(self.detection_radius))
    
    def scan_callback(self, scan_data):
        """
        激光雷达数据回调函数
        检测半径20cm内的三个点，计算最短距离的两点连线与前方墙壁的平行性
        """
        # 提取半径内的点
        points_in_radius = self.extract_points_in_radius(scan_data)
        
        if len(points_in_radius) < self.min_points:
            rospy.logwarn_throttle(self.log_throttle_period, 
                                 "检测到的点数不足: {}/{}".format(len(points_in_radius), self.min_points))
            return
        
        # 如果点数超过3个，选择距离最近的3个点
        if len(points_in_radius) > 3:
            points_in_radius = self.select_closest_three_points(points_in_radius)
        
        # 计算两两距离，找到距离最小的两个点
        closest_pair = self.find_closest_pair(points_in_radius)
        
        if closest_pair is None:
            rospy.logwarn("无法找到有效的点对")
            return
        
        # 计算这两点连成的线的角度
        line_angle = self.calculate_line_angle(closest_pair[0], closest_pair[1])
        
        # 检测前方墙壁的角度
        wall_angle = self.detect_front_wall_angle(scan_data)
        
        if wall_angle is None:
            rospy.logwarn_throttle(self.log_throttle_period, "无法检测到前方墙壁")
            return
        
        # 判断是否平行
        is_parallel = self.is_parallel(line_angle, wall_angle)
        
        # 发布结果
        result_msg = String()
        result_msg.data = "两点连线角度: {:.1f}°, 前方墙壁角度: {:.1f}°, 平行性: {}".format(
            math.degrees(line_angle), math.degrees(wall_angle), "是" if is_parallel else "否")
        
        is_parallel_msg = Bool()
        is_parallel_msg.data = is_parallel
        
        self.result_pub.publish(result_msg)
        self.is_parallel_pub.publish(is_parallel_msg)
        
        rospy.loginfo_throttle(self.log_throttle_period, result_msg.data)
    
    def extract_points_in_radius(self, scan_data):
        """
        提取半径内的点
        """
        points = []
        for i, distance in enumerate(scan_data.ranges):
            # 跳过无效值
            if np.isnan(distance) or np.isinf(distance):
                continue
            
            # 检查是否在检测半径内
            if distance <= self.detection_radius:
                # 计算角度
                angle = scan_data.angle_min + i * scan_data.angle_increment
                # 转换为笛卡尔坐标
                x = distance * math.cos(angle)
                y = distance * math.sin(angle)
                points.append((x, y, angle, distance))
        
        return points
    
    def select_closest_three_points(self, points):
        """
        从多个点中选择距离原点最近的三个点
        """
        # 按距离排序
        points_sorted = sorted(points, key=lambda p: p[3])  # 按距离排序
        return points_sorted[:3]
    
    def find_closest_pair(self, points):
        """
        找到距离最小的两个点
        """
        if len(points) < 2:
            return None
        
        min_distance = float('inf')
        closest_pair = None
        
        for i in range(len(points)):
            for j in range(i + 1, len(points)):
                # 计算两点间距离
                p1, p2 = points[i], points[j]
                distance = math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
                
                if distance < min_distance:
                    min_distance = distance
                    closest_pair = (p1, p2)
        
        return closest_pair
    
    def calculate_line_angle(self, p1, p2):
        """
        计算两点连线的角度（相对于x轴）
        """
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        return math.atan2(dy, dx)
    
    def detect_front_wall_angle(self, scan_data):
        """
        检测前方墙壁的角度
        通过分析前方区域的激光点来估计墙壁角度
        """
        # 定义前方检测区域（-30度到+30度）
        front_angle_range = math.radians(30)
        wall_points = []
        
        for i, distance in enumerate(scan_data.ranges):
            if np.isnan(distance) or np.isinf(distance):
                continue
            
            angle = scan_data.angle_min + i * scan_data.angle_increment
            
            # 检查是否在前方区域内
            if abs(angle) <= front_angle_range:
                # 过滤掉太近或太远的点
                if 0.3 <= distance <= 3.0:  # 30cm到3m之间
                    x = distance * math.cos(angle)
                    y = distance * math.sin(angle)
                    wall_points.append((x, y))
        
        if len(wall_points) < 5:  # 需要足够的点来拟合直线
            return None
        
        # 使用最小二乘法拟合直线
        return self.fit_line_angle(wall_points)
    
    def fit_line_angle(self, points):
        """
        使用最小二乘法拟合直线并返回角度
        """
        if len(points) < 2:
            return None
        
        x_coords = [p[0] for p in points]
        y_coords = [p[1] for p in points]
        
        # 计算均值
        x_mean = np.mean(x_coords)
        y_mean = np.mean(y_coords)
        
        # 计算分子和分母
        numerator = sum((x_coords[i] - x_mean) * (y_coords[i] - y_mean) for i in range(len(points)))
        denominator = sum((x_coords[i] - x_mean) ** 2 for i in range(len(points)))
        
        if abs(denominator) < 1e-6:  # 避免除零
            return math.pi / 2  # 垂直线
        
        # 计算斜率和角度
        slope = numerator / denominator
        angle = math.atan(slope)
        
        return angle
    
    def is_parallel(self, angle1, angle2):
        """
        判断两个角度是否平行（在阈值范围内）
        """
        # 计算角度差
        angle_diff = abs(angle1 - angle2)
        
        # 考虑角度的周期性（180度等价）
        angle_diff = min(angle_diff, abs(angle_diff - math.pi), abs(angle_diff + math.pi))
        
        # 转换为度数并与阈值比较
        angle_diff_degrees = math.degrees(angle_diff)
        
        return angle_diff_degrees <= self.wall_parallel_threshold
    
    def run(self):
        """
        运行节点
        """
        rospy.loginfo("三点分析节点正在运行...")
        rospy.spin()

if __name__ == "__main__":
    try:
        analyzer = ThreePointsAnalyzer()
        analyzer.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("三点分析节点已停止")
