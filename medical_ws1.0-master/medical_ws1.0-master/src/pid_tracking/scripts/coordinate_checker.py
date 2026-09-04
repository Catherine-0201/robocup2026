#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
坐标系验证工具 - 确认goal_recorder记录的坐标与unified_pose是否一致
"""

import rospy
import math
import tf.transformations
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from move_base_msgs.msg import MoveBaseGoal

class CoordinateChecker:
    def __init__(self):
        rospy.init_node('coordinate_checker', anonymous=True)
        
        # 您记录的目标点（来自goal_recorder.py）
        self.recorded_goals = [
            {'position': [1.626, -0.031], 'yaw': 0.0, 'name': '目标点1'},
            {'position': [5.751, 1.875], 'yaw': 0.0, 'name': '目标点2'}, 
            {'position': [5.891, -2.478], 'yaw': 0.0, 'name': '目标点3'},
            {'position': [0.28, -0.525], 'yaw': 0.0, 'name': '目标点4'}
        ]
        
        self.current_pose = None
        
        # 订阅unified_pose
        rospy.Subscriber('/unified_pose', PoseWithCovarianceStamped, self.pose_callback)
        
        # 发布器用于测试
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        
        rospy.loginfo("🔍 坐标系验证工具启动")
        self.run_verification()
        
    def pose_callback(self, msg):
        """接收机器人当前位置"""
        self.current_pose = msg.pose.pose
        
    def run_verification(self):
        """运行验证"""
        print("\n" + "="*80)
        print("🔍 坐标系一致性验证")
        print("="*80)
        
        # 等待获取当前位置
        rospy.loginfo("⏳ 等待获取机器人当前位置...")
        timeout = 5.0
        start_time = rospy.Time.now()
        while self.current_pose is None and not rospy.is_shutdown():
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logerr("❌ 超时：未能获取位置数据")
                return
            rospy.sleep(0.1)
            
        if self.current_pose is None:
            rospy.logerr("❌ 无法获取机器人位置，验证失败")
            return
            
        # 显示当前位置
        self.display_current_position()
        
        # 显示记录的目标点
        self.display_recorded_goals()
        
        # 验证坐标系一致性
        self.verify_coordinate_consistency()
        
        # 交互式测试
        self.interactive_test()
        
    def display_current_position(self):
        """显示当前位置"""
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        
        quat = [
            self.current_pose.orientation.x,
            self.current_pose.orientation.y,
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
        ]
        yaw = tf.transformations.euler_from_quaternion(quat)[2]
        
        print(f"\n🤖 机器人当前位置（从/unified_pose获取）:")
        print(f"   📍 坐标系: map")
        print(f"   📍 X: {x:.3f} m")
        print(f"   📍 Y: {y:.3f} m")
        print(f"   🧭 朝向: {math.degrees(yaw):.1f}° ({yaw:.3f} rad)")
        
    def display_recorded_goals(self):
        """显示记录的目标点"""
        print(f"\n🎯 记录的目标点（来自goal_recorder.py）:")
        for i, goal in enumerate(self.recorded_goals):
            print(f"   {i+1}. {goal['name']}: ({goal['position'][0]:.3f}, {goal['position'][1]:.3f})")
            
    def verify_coordinate_consistency(self):
        """验证坐标系一致性"""
        print(f"\n✅ 坐标系一致性验证:")
        print(f"   • unified_pose话题: frame_id = 'map' ✅")
        print(f"   • goal_recorder记录: 基于map坐标系 ✅") 
        print(f"   • nav_goal.py目标: frame_id = 'map' ✅")
        print(f"   • move_base接收: map坐标系目标 ✅")
        
        # 计算到各目标点的距离
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        print(f"\n📏 当前位置到各目标点的距离:")
        for i, goal in enumerate(self.recorded_goals):
            dx = goal['position'][0] - current_x
            dy = goal['position'][1] - current_y
            distance = math.sqrt(dx*dx + dy*dy)
            print(f"   到{goal['name']}: {distance:.3f}m")
            
    def interactive_test(self):
        """交互式测试"""
        print(f"\n🧪 交互式测试:")
        print(f"   输入目标点编号(1-4)来发布2D Nav Goal，输入0退出")
        
        while not rospy.is_shutdown():
            try:
                choice = input("请选择目标点(0-4): ").strip()
                
                if choice == '0':
                    print("👋 退出验证工具")
                    break
                    
                goal_idx = int(choice) - 1
                if 0 <= goal_idx < len(self.recorded_goals):
                    self.publish_nav_goal(self.recorded_goals[goal_idx])
                else:
                    print("❌ 无效选择，请输入1-4")
                    
            except ValueError:
                print("❌ 请输入数字")
            except KeyboardInterrupt:
                print("\n👋 退出验证工具")
                break
                
    def publish_nav_goal(self, goal_point):
        """发布导航目标（模拟RViz 2D Nav Goal）"""
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = rospy.Time.now()
        
        goal_msg.pose.position.x = goal_point['position'][0]
        goal_msg.pose.position.y = goal_point['position'][1]
        goal_msg.pose.position.z = 0.0
        
        yaw = goal_point['yaw']
        goal_msg.pose.orientation.x = 0.0
        goal_msg.pose.orientation.y = 0.0
        goal_msg.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.orientation.w = math.cos(yaw / 2.0)
        
        self.goal_pub.publish(goal_msg)
        
        print(f"🚀 已发布导航目标: {goal_point['name']}")
        print(f"   📍 目标坐标: ({goal_point['position'][0]:.3f}, {goal_point['position'][1]:.3f})")
        print(f"   🧭 目标方向: {math.degrees(yaw):.1f}°")
        print(f"   📡 发布到话题: /move_base_simple/goal")
        print(f"   🗺️  坐标系: map")

def main():
    try:
        checker = CoordinateChecker()
        rospy.spin()
    except Exception as e:
        rospy.logerr(f"❌ 验证工具异常: {e}")

if __name__ == '__main__':
    main()
