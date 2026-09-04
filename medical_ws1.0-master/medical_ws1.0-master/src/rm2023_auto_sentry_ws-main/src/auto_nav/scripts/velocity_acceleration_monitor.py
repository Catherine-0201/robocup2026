#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
速度和加速度监视节点
功能：
1. 订阅/cmd_vel话题，计算实时加速度
2. 发布速度和加速度监视数据到rqt可视化的话题
3. 提供详细的运动状态监控
"""

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Float64
from collections import deque


class VelocityAccelerationMonitor:
    """速度加速度监视器"""
    
    def __init__(self):
        rospy.init_node('velocity_acceleration_monitor', anonymous=False)
        
        # 参数配置
        self.monitor_rate = rospy.get_param('~monitor_rate', 20)  # 监控频率 [Hz]
        self.history_size = rospy.get_param('~history_size', 10)  # 历史数据窗口大小
        
        # 数据存储
        self.velocity_history = deque(maxlen=self.history_size)
        self.time_history = deque(maxlen=self.history_size)
        
        # 当前状态
        self.current_vel = Twist()
        self.current_acc = Twist()
        
        # ROS组件
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback, queue_size=10)
        
        # 发布器 - 用于rqt监视
        self.velocity_pub = rospy.Publisher('/monitor/velocity', Float64MultiArray, queue_size=10)
        self.acceleration_pub = rospy.Publisher('/monitor/acceleration', Float64MultiArray, queue_size=10)
        self.speed_magnitude_pub = rospy.Publisher('/monitor/speed_magnitude', Float64, queue_size=10)
        self.acceleration_magnitude_pub = rospy.Publisher('/monitor/acceleration_magnitude', Float64, queue_size=10)
        
        # 分量发布器 - 便于rqt_plot单独监视
        self.vel_x_pub = rospy.Publisher('/monitor/velocity/x', Float64, queue_size=10)
        self.vel_y_pub = rospy.Publisher('/monitor/velocity/y', Float64, queue_size=10)
        self.vel_angular_pub = rospy.Publisher('/monitor/velocity/angular', Float64, queue_size=10)
        
        self.acc_x_pub = rospy.Publisher('/monitor/acceleration/x', Float64, queue_size=10)
        self.acc_y_pub = rospy.Publisher('/monitor/acceleration/y', Float64, queue_size=10)
        self.acc_angular_pub = rospy.Publisher('/monitor/acceleration/angular', Float64, queue_size=10)
        
        # 定时器
        self.monitor_timer = rospy.Timer(rospy.Duration(1.0/self.monitor_rate), self.monitor_callback)
        
        rospy.loginfo("速度加速度监视器已启动")
        rospy.loginfo(f"监控频率: {self.monitor_rate} Hz")
        rospy.loginfo("发布话题:")
        rospy.loginfo("  /monitor/velocity/* - 速度分量")
        rospy.loginfo("  /monitor/acceleration/* - 加速度分量")
        rospy.loginfo("  /monitor/speed_magnitude - 速度大小")
        rospy.loginfo("  /monitor/acceleration_magnitude - 加速度大小")

    def cmd_vel_callback(self, msg):
        """速度指令回调函数"""
        self.current_vel = msg
        current_time = rospy.Time.now()
        
        # 记录历史数据
        self.velocity_history.append(msg)
        self.time_history.append(current_time)
        
        # 计算加速度（需要至少2个数据点）
        if len(self.velocity_history) >= 2:
            self.calculate_acceleration()

    def calculate_acceleration(self):
        """计算加速度"""
        if len(self.velocity_history) < 2:
            return
            
        # 获取最近两个速度数据点
        vel_curr = self.velocity_history[-1]
        vel_prev = self.velocity_history[-2]
        time_curr = self.time_history[-1]
        time_prev = self.time_history[-2]
        
        # 计算时间差
        dt = (time_curr - time_prev).to_sec()
        if dt <= 0:
            return
        
        # 计算加速度
        self.current_acc.linear.x = (vel_curr.linear.x - vel_prev.linear.x) / dt
        self.current_acc.linear.y = (vel_curr.linear.y - vel_prev.linear.y) / dt
        self.current_acc.angular.z = (vel_curr.angular.z - vel_prev.angular.z) / dt

    def monitor_callback(self, event):
        """监控定时器回调"""
        current_time = rospy.Time.now()
        
        # 发布速度数据
        velocity_array = Float64MultiArray()
        velocity_array.data = [
            self.current_vel.linear.x,
            self.current_vel.linear.y, 
            self.current_vel.angular.z
        ]
        self.velocity_pub.publish(velocity_array)
        
        # 发布加速度数据
        acceleration_array = Float64MultiArray()
        acceleration_array.data = [
            self.current_acc.linear.x,
            self.current_acc.linear.y,
            self.current_acc.angular.z
        ]
        self.acceleration_pub.publish(acceleration_array)
        
        # 发布速度大小
        speed_magnitude = math.sqrt(
            self.current_vel.linear.x**2 + self.current_vel.linear.y**2
        )
        self.speed_magnitude_pub.publish(Float64(speed_magnitude))
        
        # 发布加速度大小
        acc_magnitude = math.sqrt(
            self.current_acc.linear.x**2 + self.current_acc.linear.y**2
        )
        self.acceleration_magnitude_pub.publish(Float64(acc_magnitude))
        
        # 发布各分量（便于rqt_plot）
        self.vel_x_pub.publish(Float64(self.current_vel.linear.x))
        self.vel_y_pub.publish(Float64(self.current_vel.linear.y))
        self.vel_angular_pub.publish(Float64(self.current_vel.angular.z))
        
        self.acc_x_pub.publish(Float64(self.current_acc.linear.x))
        self.acc_y_pub.publish(Float64(self.current_acc.linear.y))
        self.acc_angular_pub.publish(Float64(self.current_acc.angular.z))
        
        # 每5秒打印一次状态信息
        if hasattr(self, 'last_print_time'):
            if (current_time - self.last_print_time).to_sec() > 5.0:
                self.print_status()
                self.last_print_time = current_time
        else:
            self.last_print_time = current_time

    def print_status(self):
        """打印当前状态"""
        speed_mag = math.sqrt(self.current_vel.linear.x**2 + self.current_vel.linear.y**2)
        acc_mag = math.sqrt(self.current_acc.linear.x**2 + self.current_acc.linear.y**2)
        
        rospy.loginfo("=== 运动状态监控 ===")
        rospy.loginfo(f"速度: x={self.current_vel.linear.x:.3f}, y={self.current_vel.linear.y:.3f}, "
                     f"angular={self.current_vel.angular.z:.3f}, 大小={speed_mag:.3f} m/s")
        rospy.loginfo(f"加速度: x={self.current_acc.linear.x:.3f}, y={self.current_acc.linear.y:.3f}, "
                     f"angular={self.current_acc.angular.z:.3f}, 大小={acc_mag:.3f} m/s²")

    def run(self):
        """运行监视器"""
        rospy.loginfo("速度加速度监视器开始运行...")
        rospy.spin()


if __name__ == '__main__':
    try:
        monitor = VelocityAccelerationMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("速度加速度监视器已停止")
    except Exception as e:
        rospy.logerr(f"监视器运行异常: {e}")
        import traceback
        traceback.print_exc()

