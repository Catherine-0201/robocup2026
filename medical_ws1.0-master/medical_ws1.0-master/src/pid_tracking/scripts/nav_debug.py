#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
导航调试工具 - 检查move_base配置和状态
"""

import rospy
import subprocess
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray

class NavigationDebugger:
    def __init__(self):
        rospy.init_node('nav_debugger', anonymous=True)
        
        self.current_pose = None
        self.goal_status = None
        
        # 订阅位置和状态
        try:
            rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.pose_callback)
        except:
            try:
                rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)
            except:
                try:
                    rospy.Subscriber("/odom", Odometry, self.odom_callback)
                except:
                    pass
        
        try:
            rospy.Subscriber("/move_base/status", GoalStatusArray, self.status_callback)
        except:
            pass
        
        rospy.sleep(2.0)  # 等待订阅生效
        
    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose
        
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        
    def status_callback(self, msg):
        self.goal_status = msg
        
    def check_topics(self):
        """检查相关话题"""
        print("🔍 检查ROS话题...")
        
        try:
            topics = subprocess.check_output(["rostopic", "list"]).decode('utf-8').split('\n')
            
            important_topics = [
                "/move_base/goal",
                "/move_base/result", 
                "/move_base/status",
                "/move_base_simple/goal",
                "/cmd_vel",
                "/odom",
                "/robot_pose_ekf/odom_combined",
                "/amcl_pose",
                "/scan",
                "/map"
            ]
            
            print("\n📋 重要话题状态:")
            for topic in important_topics:
                status = "✅" if topic in topics else "❌"
                print(f"   {status} {topic}")
                
        except Exception as e:
            print(f"❌ 无法检查话题: {e}")
    
    def check_parameters(self):
        """检查move_base参数"""
        print("\n🔧 检查move_base参数...")
        
        important_params = [
            "/move_base/base_local_planner",
            "/move_base/base_global_planner", 
            "/move_base/controller_frequency",
            "/move_base/planner_frequency",
            "/move_base/TebLocalPlannerROS/xy_goal_tolerance",
            "/move_base/TebLocalPlannerROS/yaw_goal_tolerance",
            "/move_base/DWAPlannerROS/xy_goal_tolerance",
            "/move_base/DWAPlannerROS/yaw_goal_tolerance",
            "/move_base/TrajectoryPlannerROS/xy_goal_tolerance", 
            "/move_base/TrajectoryPlannerROS/yaw_goal_tolerance"
        ]
        
        for param in important_params:
            try:
                value = rospy.get_param(param)
                print(f"   ✅ {param}: {value}")
            except:
                print(f"   ❌ {param}: 未设置")
    
    def check_current_status(self):
        """检查当前状态"""
        print("\n📡 当前状态:")
        
        if self.current_pose:
            x = self.current_pose.position.x
            y = self.current_pose.position.y
            print(f"   🤖 机器人位置: ({x:.3f}, {y:.3f})")
        else:
            print("   ❌ 无法获取机器人位置")
            
        if self.goal_status and self.goal_status.status_list:
            for status in self.goal_status.status_list:
                print(f"   🎯 目标状态: {status.status} ({status.text})")
        else:
            print("   ✅ 当前无活动目标")
    
    def run_diagnostics(self):
        """运行完整诊断"""
        print("🔍 ROS导航系统诊断")
        print("="*50)
        
        self.check_topics()
        self.check_parameters()
        self.check_current_status()
        
        print("\n💡 建议:")
        print("1. 确保所有重要话题都存在")
        print("2. 检查goal_tolerance参数（建议0.2-0.5米）")
        print("3. 验证机器人定位正常")
        print("4. 检查costmap和路径规划器运行状态")

def main():
    try:
        debugger = NavigationDebugger()
        debugger.run_diagnostics()
    except Exception as e:
        print(f"❌ 调试工具运行失败: {e}")

if __name__ == '__main__':
    main()




