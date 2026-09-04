#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
真实速度和加速度监视节点 (修正版本)
功能：
1. 监视实际速度：从里程计(/odom)获取真实速度
2. 监视控制指令：从/cmd_vel获取期望速度  
3. 计算真实加速度：基于实际速度变化
4. 对比分析：实际vs期望的速度差异
5. 验证加速度限制：检查ruckig加速度控制效果
"""

import rospy
import numpy as np
import math
import time
from collections import deque

# ROS消息类型
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64MultiArray, Float64


class RealVelocityAccelerationMonitor:
    """真实速度加速度监视器"""
    
    def __init__(self):
        rospy.init_node('real_velocity_acceleration_monitor', anonymous=False)
        
        # 参数配置
        self.monitor_rate = rospy.get_param('~monitor_rate', 20)  # 监控频率 [Hz]
        self.history_size = rospy.get_param('~history_size', 10)  # 历史数据窗口大小
        self.odom_topic = rospy.get_param('~odom_topic', '/odom')  # 里程计话题
        self.ekf_topic = rospy.get_param('~ekf_topic', '/robot_pose_ekf/odom_combined')  # EKF话题
        self.use_ekf = rospy.get_param('~use_ekf', True)  # 是否使用EKF数据
        
        # 数据存储 - 实际速度
        self.actual_velocity_history = deque(maxlen=self.history_size)
        self.actual_time_history = deque(maxlen=self.history_size)
        
        # 数据存储 - 期望速度
        self.cmd_velocity_history = deque(maxlen=self.history_size)
        self.cmd_time_history = deque(maxlen=self.history_size)
        
        # 当前状态
        self.current_actual_vel = Twist()      # 实际速度
        self.current_cmd_vel = Twist()         # 期望速度
        self.current_actual_acc = Twist()      # 实际加速度
        self.current_cmd_acc = Twist()         # 期望加速度
        
        # 初始化ROS组件
        self.setup_ros_components()
        
        rospy.loginfo("真实速度加速度监视器已启动")
        rospy.loginfo(f"监控频率: {self.monitor_rate} Hz")
        rospy.loginfo(f"里程计话题: {self.odom_topic}")
        rospy.loginfo(f"EKF话题: {self.ekf_topic} (使用: {self.use_ekf})")
        rospy.loginfo("发布话题:")
        rospy.loginfo("  实际速度: /monitor/actual_velocity/*")
        rospy.loginfo("  期望速度: /monitor/cmd_velocity/*") 
        rospy.loginfo("  实际加速度: /monitor/actual_acceleration/*")
        rospy.loginfo("  速度差异: /monitor/velocity_error/*")

    def setup_ros_components(self):
        """设置ROS组件"""
        # 订阅器
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=10)
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback, queue_size=10)
        
        if self.use_ekf:
            self.ekf_sub = rospy.Subscriber(self.ekf_topic, PoseWithCovarianceStamped, self.ekf_callback, queue_size=10)
        
        # 发布器 - 实际速度
        self.actual_velocity_pub = rospy.Publisher('/monitor/actual_velocity', Float64MultiArray, queue_size=10)
        self.actual_vel_x_pub = rospy.Publisher('/monitor/actual_velocity/x', Float64, queue_size=10)
        self.actual_vel_y_pub = rospy.Publisher('/monitor/actual_velocity/y', Float64, queue_size=10)
        self.actual_vel_angular_pub = rospy.Publisher('/monitor/actual_velocity/angular', Float64, queue_size=10)
        self.actual_speed_magnitude_pub = rospy.Publisher('/monitor/actual_speed_magnitude', Float64, queue_size=10)
        
        # 发布器 - 期望速度
        self.cmd_velocity_pub = rospy.Publisher('/monitor/cmd_velocity', Float64MultiArray, queue_size=10)
        self.cmd_vel_x_pub = rospy.Publisher('/monitor/cmd_velocity/x', Float64, queue_size=10)
        self.cmd_vel_y_pub = rospy.Publisher('/monitor/cmd_velocity/y', Float64, queue_size=10)
        self.cmd_vel_angular_pub = rospy.Publisher('/monitor/cmd_velocity/angular', Float64, queue_size=10)
        self.cmd_speed_magnitude_pub = rospy.Publisher('/monitor/cmd_speed_magnitude', Float64, queue_size=10)
        
        # 发布器 - 实际加速度
        self.actual_acceleration_pub = rospy.Publisher('/monitor/actual_acceleration', Float64MultiArray, queue_size=10)
        self.actual_acc_x_pub = rospy.Publisher('/monitor/actual_acceleration/x', Float64, queue_size=10)
        self.actual_acc_y_pub = rospy.Publisher('/monitor/actual_acceleration/y', Float64, queue_size=10)
        self.actual_acc_angular_pub = rospy.Publisher('/monitor/actual_acceleration/angular', Float64, queue_size=10)
        self.actual_acceleration_magnitude_pub = rospy.Publisher('/monitor/actual_acceleration_magnitude', Float64, queue_size=10)
        
        # 发布器 - 速度误差
        self.velocity_error_pub = rospy.Publisher('/monitor/velocity_error', Float64MultiArray, queue_size=10)
        self.vel_error_x_pub = rospy.Publisher('/monitor/velocity_error/x', Float64, queue_size=10)
        self.vel_error_y_pub = rospy.Publisher('/monitor/velocity_error/y', Float64, queue_size=10)
        self.vel_error_angular_pub = rospy.Publisher('/monitor/velocity_error/angular', Float64, queue_size=10)
        
        # 定时器
        self.monitor_timer = rospy.Timer(rospy.Duration(1.0/self.monitor_rate), self.monitor_callback)

    def odom_callback(self, msg):
        """里程计回调函数 - 获取实际速度"""
        # 从里程计获取实际速度
        self.current_actual_vel = msg.twist.twist
        current_time = rospy.Time.now()
        
        # 记录历史数据
        self.actual_velocity_history.append(msg.twist.twist)
        self.actual_time_history.append(current_time)
        
        # 计算实际加速度
        if len(self.actual_velocity_history) >= 2:
            self.calculate_actual_acceleration()

    def ekf_callback(self, msg):
        """EKF回调函数 - 如果使用融合数据"""
        # 注意：PoseWithCovarianceStamped不包含速度信息
        # 如果需要EKF速度，需要订阅geometry_msgs/TwistWithCovarianceStamped
        pass

    def cmd_vel_callback(self, msg):
        """控制指令回调函数 - 获取期望速度"""
        self.current_cmd_vel = msg
        current_time = rospy.Time.now()
        
        # 记录历史数据
        self.cmd_velocity_history.append(msg)
        self.cmd_time_history.append(current_time)

    def calculate_actual_acceleration(self):
        """计算实际加速度"""
        if len(self.actual_velocity_history) < 2:
            return
            
        # 获取最近两个实际速度数据点
        vel_curr = self.actual_velocity_history[-1]
        vel_prev = self.actual_velocity_history[-2]
        time_curr = self.actual_time_history[-1]
        time_prev = self.actual_time_history[-2]
        
        # 计算时间差
        dt = (time_curr - time_prev).to_sec()
        if dt <= 0:
            return
        
        # 计算实际加速度
        self.current_actual_acc.linear.x = (vel_curr.linear.x - vel_prev.linear.x) / dt
        self.current_actual_acc.linear.y = (vel_curr.linear.y - vel_prev.linear.y) / dt
        self.current_actual_acc.angular.z = (vel_curr.angular.z - vel_prev.angular.z) / dt

    def monitor_callback(self, event):
        """监控定时器回调"""
        current_time = rospy.Time.now()
        
        # 发布实际速度数据
        actual_velocity_array = Float64MultiArray()
        actual_velocity_array.data = [
            self.current_actual_vel.linear.x,
            self.current_actual_vel.linear.y, 
            self.current_actual_vel.angular.z
        ]
        self.actual_velocity_pub.publish(actual_velocity_array)
        
        # 发布期望速度数据
        cmd_velocity_array = Float64MultiArray()
        cmd_velocity_array.data = [
            self.current_cmd_vel.linear.x,
            self.current_cmd_vel.linear.y,
            self.current_cmd_vel.angular.z
        ]
        self.cmd_velocity_pub.publish(cmd_velocity_array)
        
        # 发布实际加速度数据
        actual_acceleration_array = Float64MultiArray()
        actual_acceleration_array.data = [
            self.current_actual_acc.linear.x,
            self.current_actual_acc.linear.y,
            self.current_actual_acc.angular.z
        ]
        self.actual_acceleration_pub.publish(actual_acceleration_array)
        
        # 计算并发布速度误差
        vel_error_x = self.current_actual_vel.linear.x - self.current_cmd_vel.linear.x
        vel_error_y = self.current_actual_vel.linear.y - self.current_cmd_vel.linear.y
        vel_error_angular = self.current_actual_vel.angular.z - self.current_cmd_vel.angular.z
        
        velocity_error_array = Float64MultiArray()
        velocity_error_array.data = [vel_error_x, vel_error_y, vel_error_angular]
        self.velocity_error_pub.publish(velocity_error_array)
        
        # 发布各分量（便于rqt_plot）
        self.actual_vel_x_pub.publish(Float64(self.current_actual_vel.linear.x))
        self.actual_vel_y_pub.publish(Float64(self.current_actual_vel.linear.y))
        self.actual_vel_angular_pub.publish(Float64(self.current_actual_vel.angular.z))
        
        self.cmd_vel_x_pub.publish(Float64(self.current_cmd_vel.linear.x))
        self.cmd_vel_y_pub.publish(Float64(self.current_cmd_vel.linear.y))
        self.cmd_vel_angular_pub.publish(Float64(self.current_cmd_vel.angular.z))
        
        self.actual_acc_x_pub.publish(Float64(self.current_actual_acc.linear.x))
        self.actual_acc_y_pub.publish(Float64(self.current_actual_acc.linear.y))
        self.actual_acc_angular_pub.publish(Float64(self.current_actual_acc.angular.z))
        
        self.vel_error_x_pub.publish(Float64(vel_error_x))
        self.vel_error_y_pub.publish(Float64(vel_error_y))
        self.vel_error_angular_pub.publish(Float64(vel_error_angular))
        
        # 发布速度和加速度大小
        actual_speed_magnitude = math.sqrt(
            self.current_actual_vel.linear.x**2 + self.current_actual_vel.linear.y**2
        )
        cmd_speed_magnitude = math.sqrt(
            self.current_cmd_vel.linear.x**2 + self.current_cmd_vel.linear.y**2
        )
        actual_acc_magnitude = math.sqrt(
            self.current_actual_acc.linear.x**2 + self.current_actual_acc.linear.y**2
        )
        
        self.actual_speed_magnitude_pub.publish(Float64(actual_speed_magnitude))
        self.cmd_speed_magnitude_pub.publish(Float64(cmd_speed_magnitude))
        self.actual_acceleration_magnitude_pub.publish(Float64(actual_acc_magnitude))
        
        # 每5秒打印一次状态信息
        if hasattr(self, 'last_print_time'):
            if (current_time - self.last_print_time).to_sec() > 5.0:
                self.print_status()
                self.last_print_time = current_time
        else:
            self.last_print_time = current_time

    def print_status(self):
        """打印当前状态"""
        actual_speed_mag = math.sqrt(self.current_actual_vel.linear.x**2 + self.current_actual_vel.linear.y**2)
        cmd_speed_mag = math.sqrt(self.current_cmd_vel.linear.x**2 + self.current_cmd_vel.linear.y**2)
        actual_acc_mag = math.sqrt(self.current_actual_acc.linear.x**2 + self.current_actual_acc.linear.y**2)
        
        rospy.loginfo("=== 真实vs期望运动状态监控 ===")
        rospy.loginfo(f"实际速度: x={self.current_actual_vel.linear.x:.3f}, y={self.current_actual_vel.linear.y:.3f}, "
                     f"angular={self.current_actual_vel.angular.z:.3f}, 大小={actual_speed_mag:.3f} m/s")
        rospy.loginfo(f"期望速度: x={self.current_cmd_vel.linear.x:.3f}, y={self.current_cmd_vel.linear.y:.3f}, "
                     f"angular={self.current_cmd_vel.angular.z:.3f}, 大小={cmd_speed_mag:.3f} m/s")
        rospy.loginfo(f"实际加速度: x={self.current_actual_acc.linear.x:.3f}, y={self.current_actual_acc.linear.y:.3f}, "
                     f"angular={self.current_actual_acc.angular.z:.3f}, 大小={actual_acc_mag:.3f} m/s²")
        
        # 计算跟踪误差
        vel_error_mag = math.sqrt((self.current_actual_vel.linear.x - self.current_cmd_vel.linear.x)**2 + 
                                 (self.current_actual_vel.linear.y - self.current_cmd_vel.linear.y)**2)
        rospy.loginfo(f"速度跟踪误差: {vel_error_mag:.3f} m/s")

    def run(self):
        """运行监视器"""
        rospy.loginfo("真实速度加速度监视器开始运行...")
        rospy.loginfo("监控数据:")
        rospy.loginfo("  实际速度: 来自里程计 " + self.odom_topic)
        rospy.loginfo("  期望速度: 来自控制指令 /cmd_vel")
        rospy.loginfo("  实际加速度: 基于实际速度计算")
        rospy.spin()


if __name__ == '__main__':
    try:
        monitor = RealVelocityAccelerationMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("真实速度加速度监视器已停止")
    except Exception as e:
        rospy.logerr(f"监视器运行异常: {e}")
        import traceback
        traceback.print_exc()





