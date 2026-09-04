#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import numpy as np

class LidarLineDistanceNode:
    def __init__(self):
        # 中心过滤范围
        self.num_points_to_filter = 25
        # 过滤半径范围内的雷达点
        self.min_distance = 0.20
        # 双向搜索范围
        self.angle_range_deg = 10 
        # 相邻两点的阈值
        self.threshold = 0.02  

        # 初始化ROS节点
        rospy.init_node('lidar_line_distance_node', anonymous=True)

        # 创建发布器
        self.distances_pub = rospy.Publisher('/lidar_distances', Float32MultiArray, queue_size=10)

        # 订阅激光雷达数据
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.header = 0
        self.angle_min = 0
        self.angle_max = 0
        self.angle_increment = 0
        self.time_increment = 0
        self.scan_time = 0
        self.range_min = 0
        self.range_max = 0
        self.num_ranges = 0

    # 双向直线点集搜索
    def find_points_for_line(self, ranges, start_index, flag):
        points = []
        num_readings = len(ranges)

        # 从右边开始查找
        for offset in range(self.angle_range_deg):
            if not flag:
                i = (start_index + offset) % num_readings  # 环形索引
            else:
                i = (start_index + offset + self.num_points_to_filter) % num_readings  # 环形索引
            if self.range_min <= ranges[i] <= self.range_max:
                if not points or abs(ranges[i] - points[-1][1]) <= self.threshold:
                    angle = i * self.angle_increment
                    points.append((i, ranges[i], angle))
                else:
                    break

        # 从左边开始查找
        for offset in range(1, self.angle_range_deg + 1):
            if not flag:
                i = (start_index - offset + num_readings) % num_readings  # 环形索引
            else:
                i = (start_index - offset - self.num_points_to_filter + num_readings) % num_readings  # 环形索引
            if self.range_min <= ranges[i] <= self.range_max:
                if not points or abs(ranges[i] - points[0][1]) <= self.threshold:
                    angle = i * self.angle_increment
                    points.insert(0, (i, ranges[i], angle))
                else:
                    break
        return points

    # 最小二乘法计算直线距离
    def calculate_line_distance(self, points):
        if len(points) < 2:
            return float('inf')
        x = np.array([p[1] * np.cos(p[2]) for p in points])
        y = np.array([p[1] * np.sin(p[2]) for p in points])

        A = np.vstack([x, np.ones(len(x))]).T
        m, b = np.linalg.lstsq(A, y, rcond=None)[0]

        distance = abs(b) / np.sqrt(m**2 + 1)
        return distance

    # 激光雷达数据回调函数
    def scan_callback(self, scan_data):
        self.header = scan_data.header
        self.angle_min = scan_data.angle_min
        self.angle_max = scan_data.angle_max
        self.angle_increment = scan_data.angle_increment
        self.time_increment = scan_data.time_increment
        self.scan_time = scan_data.scan_time
        self.range_min = scan_data.range_min
        self.range_max = scan_data.range_max
        self.num_ranges = len(scan_data.ranges)

        # 过滤掉 最小距离内的雷达点
        filtered_ranges = []
        for i, r in enumerate(scan_data.ranges):
            if r >= self.min_distance:
                filtered_ranges.append(r)
            else:
                filtered_ranges.append(float('inf'))


        front_index = 0
        left_index = int((-np.pi / 2 - self.angle_min) / self.angle_increment)  
        right_index = int((np.pi / 2 - self.angle_min) / self.angle_increment) 

        front_points = self.find_points_for_line(filtered_ranges, front_index, True)
        front_distance = self.calculate_line_distance(front_points)

        left_points = self.find_points_for_line(filtered_ranges, left_index, True)
        left_distance = self.calculate_line_distance(left_points)

        right_points = self.find_points_for_line(filtered_ranges, right_index, True)
        right_distance = self.calculate_line_distance(right_points)

        # rospy.loginfo(f"Front Distance: {front_distance:.2f} m, Left Distance: {left_distance:.2f} m, Right Distance: {right_distance:.2f} m")

        distances_msg = Float32MultiArray()
        distances_msg.data = [front_distance, left_distance, right_distance]
        self.distances_pub.publish(distances_msg)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = LidarLineDistanceNode()
    node.run()
