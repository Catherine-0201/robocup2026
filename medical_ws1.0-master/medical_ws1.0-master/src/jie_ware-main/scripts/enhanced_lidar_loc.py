#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
增强的激光雷达定位包装器
结合动态障碍物过滤，提高定位鲁棒性
"""

import rospy
import subprocess
import signal
import os
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String

class EnhancedLidarLoc:
    def __init__(self):
        rospy.init_node('enhanced_lidar_loc', anonymous=False)
        
        # 原始激光雷达定位进程
        self.lidar_loc_process = None
        self.dynamic_filter_process = None
        
        # 配置参数
        self.use_dynamic_filtering = rospy.get_param('~use_dynamic_filtering', True)
        self.fallback_to_original = rospy.get_param('~fallback_to_original', True)
        
        # 监控原始激光数据和过滤后数据
        self.original_scan_active = False
        self.filtered_scan_active = False
        
        # 订阅器
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.filtered_scan_sub = rospy.Subscriber('/scan_filtered', LaserScan, self.filtered_scan_callback)
        self.localization_status_sub = rospy.Subscriber('/localization_status', Bool, self.status_callback)
        
        # 发布器
        self.system_status_pub = rospy.Publisher('/enhanced_lidar_status', String, queue_size=1)
        
        rospy.loginfo("增强激光雷达定位系统启动")
        
        # 启动各个组件
        self.start_components()

    def start_components(self):
        """启动各个组件"""
        try:
            # 启动动态障碍物过滤器
            if self.use_dynamic_filtering:
                rospy.loginfo("启动动态障碍物过滤器...")
                self.dynamic_filter_process = subprocess.Popen([
                    'rosrun', 'jie_ware', 'dynamic_obstacle_filter.py'
                ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                rospy.sleep(2)  # 等待过滤器启动
            
            # 启动原始激光雷达定位节点（修改为使用过滤后的数据）
            rospy.loginfo("启动激光雷达定位节点...")
            # 注意：这里假设原始的lidar_loc节点可以通过重映射话题来使用过滤后的数据
            if self.use_dynamic_filtering:
                # 使用过滤后的激光数据
                self.lidar_loc_process = subprocess.Popen([
                    'rosrun', 'jie_ware', 'lidar_loc',
                    '/scan:=/scan_filtered'  # 重映射到过滤后的数据
                ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            else:
                # 使用原始激光数据
                self.lidar_loc_process = subprocess.Popen([
                    'rosrun', 'jie_ware', 'lidar_loc'
                ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                
        except Exception as e:
            rospy.logerr("启动组件失败: {}".format(e))

    def scan_callback(self, msg):
        """原始激光数据回调"""
        self.original_scan_active = True

    def filtered_scan_callback(self, msg):
        """过滤后激光数据回调"""
        self.filtered_scan_active = True

    def status_callback(self, msg):
        """定位状态回调"""
        status = "可靠" if msg.data else "备用模式"
        
        # 发布系统状态
        status_info = {
            'localization_reliable': msg.data,
            'original_scan_active': self.original_scan_active,
            'filtered_scan_active': self.filtered_scan_active,
            'dynamic_filtering_enabled': self.use_dynamic_filtering
        }
        
        status_msg = String()
        status_msg.data = str(status_info)
        self.system_status_pub.publish(status_msg)

    def cleanup(self):
        """清理资源"""
        rospy.loginfo("正在关闭增强激光雷达定位系统...")
        
        if self.lidar_loc_process:
            self.lidar_loc_process.terminate()
            self.lidar_loc_process.wait()
            
        if self.dynamic_filter_process:
            self.dynamic_filter_process.terminate()
            self.dynamic_filter_process.wait()

    def run(self):
        """运行主循环"""
        # 注册信号处理器
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # 定时检查进程状态
        rate = rospy.Rate(1)  # 1Hz
        while not rospy.is_shutdown():
            # 检查进程是否正在运行
            if self.lidar_loc_process and self.lidar_loc_process.poll() is not None:
                rospy.logwarn("激光雷达定位进程意外退出，重启中...")
                self.start_components()
            
            if (self.use_dynamic_filtering and self.dynamic_filter_process and 
                self.dynamic_filter_process.poll() is not None):
                rospy.logwarn("动态障碍物过滤器进程意外退出，重启中...")
                self.start_components()
            
            rate.sleep()

    def signal_handler(self, signum, frame):
        """信号处理器"""
        rospy.loginfo("接收到停止信号，正在清理...")
        self.cleanup()
        rospy.signal_shutdown("用户中断")


if __name__ == '__main__':
    try:
        enhanced_loc = EnhancedLidarLoc()
        enhanced_loc.run()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("增强激光雷达定位系统错误: {}".format(e))
    finally:
        if 'enhanced_loc' in locals():
            enhanced_loc.cleanup()
