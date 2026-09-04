#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarCenterFilterNode:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('lidar_center_filter_node', anonymous=True)
        
        # 从参数服务器读取配置参数
        self.filter_radius = rospy.get_param('~filter_radius', 0.20)  # 默认20厘米
        self.log_throttle_period = rospy.get_param('~log_throttle_period', 1.0)
        self.input_topic = rospy.get_param('~input_topic', '/scan')
        self.output_topic = rospy.get_param('~output_topic', '/scan_filtered')
        
        # 创建发布器 - 发布过滤后的激光雷达数据
        self.filtered_scan_pub = rospy.Publisher(self.output_topic, LaserScan, queue_size=10)
        
        # 订阅原始激光雷达数据
        rospy.Subscriber(self.input_topic, LaserScan, self.scan_callback)
        
        rospy.loginfo("雷达中心过滤节点已启动，过滤半径: {:.2f}m".format(self.filter_radius))
    
    def scan_callback(self, scan_data):
        """
        激光雷达数据回调函数
        过滤掉距离中心点20厘米内的所有障碍物
        """
        # 创建过滤后的激光雷达消息
        filtered_scan = LaserScan()
        
        # 复制原始扫描数据的头部信息
        filtered_scan.header = scan_data.header
        filtered_scan.angle_min = scan_data.angle_min
        filtered_scan.angle_max = scan_data.angle_max
        filtered_scan.angle_increment = scan_data.angle_increment
        filtered_scan.time_increment = scan_data.time_increment
        filtered_scan.scan_time = scan_data.scan_time
        filtered_scan.range_min = scan_data.range_min
        filtered_scan.range_max = scan_data.range_max
        
        # 过滤距离数据
        filtered_ranges = []
        filtered_count = 0
        
        for i, distance in enumerate(scan_data.ranges):
            # 如果距离小于过滤半径或者是无效值，则设置为无穷大
            if distance < self.filter_radius or np.isnan(distance) or np.isinf(distance):
                filtered_ranges.append(float('inf'))
                if distance < self.filter_radius and not (np.isnan(distance) or np.isinf(distance)):
                    filtered_count += 1
            else:
                filtered_ranges.append(distance)
        
        filtered_scan.ranges = filtered_ranges
        filtered_scan.intensities = scan_data.intensities
        
        # 发布过滤后的数据
        self.filtered_scan_pub.publish(filtered_scan)
        
        # 记录过滤信息
        if filtered_count > 0:
            rospy.loginfo_throttle(self.log_throttle_period, "已过滤 {} 个半径{}m内的障碍物点".format(
                filtered_count, self.filter_radius))
    
    def run(self):
        """
        运行节点
        """
        rospy.loginfo("雷达中心过滤节点正在运行...")
        rospy.spin()

if __name__ == "__main__":
    try:
        node = LidarCenterFilterNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("雷达中心过滤节点已停止")
