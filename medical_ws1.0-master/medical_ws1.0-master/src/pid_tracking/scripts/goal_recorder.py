#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
目标点记录工具 - 监听RViz 2D Nav Goal并记录坐标
使用方法：
1. 启动导航系统
2. 运行此脚本：python goal_recorder.py
3. 在RViz中设置2D Nav Goal
4. 脚本会自动记录并显示坐标
"""

import rospy
import math
import yaml
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

class GoalRecorder:
    def __init__(self):
        rospy.init_node('goal_recorder', anonymous=True)
        
        # 记录的目标点列表
        self.recorded_goals = []
        self.goal_count = 0
        
        # 订阅2D Nav Goal话题
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)
        
        rospy.loginfo("🎯 目标点记录器已启动!")
        rospy.loginfo("📍 在RViz中使用2D Nav Goal设置目标点")
        rospy.loginfo("💾 按Ctrl+C保存并退出")
        
    def goal_callback(self, msg):
        """处理接收到的目标点"""
        self.goal_count += 1
        
        # 提取位置信息
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        
        # 转换四元数到欧拉角
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        
        # 计算yaw角（绕Z轴旋转）
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        
        # 记录目标点
        goal_data = {
            'index': self.goal_count - 1,
            'position': [round(x, 3), round(y, 3)],
            'position_with_z': [round(x, 3), round(y, 3), round(z, 3)],
            'yaw_radians': round(yaw, 3),
            'yaw_degrees': round(math.degrees(yaw), 1),
            'quaternion': [round(qx, 3), round(qy, 3), round(qz, 3), round(qw, 3)]
        }
        
        self.recorded_goals.append(goal_data)
        
        # 实时显示
        self.print_goal_info(goal_data)
        self.print_current_summary()
    
    def print_goal_info(self, goal_data):
        """打印单个目标点信息"""
        print(f"\n🎯 目标点 {goal_data['index']}:")
        print(f"   📍 坐标: [{goal_data['position'][0]}, {goal_data['position'][1]}]")
        print(f"   🧭 方向: {goal_data['yaw_degrees']}° ({goal_data['yaw_radians']} rad)")
        print(f"   🔢 四元数: {goal_data['quaternion']}")
    
    def print_current_summary(self):
        """打印当前记录的所有目标点汇总"""
        print(f"\n📊 当前已记录 {len(self.recorded_goals)} 个目标点:")
        
        # YAML格式（用于配置文件）
        yaml_goals = []
        for goal in self.recorded_goals:
            yaml_goals.append(goal['position'])
        
        print("\n📄 YAML格式（复制到配置文件）:")
        print(f"goal_point: {yaml_goals}")
        print(f"goal_point_num: {len(self.recorded_goals)}")
        
        # Python格式（用于脚本）
        print("\n🐍 Python格式（复制到脚本）:")
        python_goals = {}
        for i, goal in enumerate(self.recorded_goals):
            key = f"goal_{i}"
            python_goals[key] = {
                'position': goal['position'],
                'yaw': goal['yaw_radians']
            }
        print(f"key_positions = {python_goals}")
    
    def save_to_file(self):
        """保存到文件"""
        try:
            # 保存为YAML格式
            yaml_data = {
                'goal_point': [goal['position'] for goal in self.recorded_goals],
                'goal_point_num': len(self.recorded_goals),
                'detailed_goals': self.recorded_goals
            }
            
            filename = f"/home/jetson/code_files/robocup/recorded_goals_{rospy.get_time():.0f}.yaml"
            with open(filename, 'w') as f:
                yaml.dump(yaml_data, f, default_flow_style=False, allow_unicode=True)
            
            rospy.loginfo(f"💾 目标点已保存到: {filename}")
            
            # 同时保存为Python格式
            py_filename = filename.replace('.yaml', '.py')
            with open(py_filename, 'w') as f:
                f.write("# -*- coding: utf-8 -*-\n")
                f.write("# 自动生成的目标点配置\n\n")
                f.write(f"# 记录时间: {rospy.get_time()}\n")
                f.write(f"# 目标点数量: {len(self.recorded_goals)}\n\n")
                
                f.write("# YAML格式（用于配置文件）\n")
                f.write(f"goal_point = {[goal['position'] for goal in self.recorded_goals]}\n")
                f.write(f"goal_point_num = {len(self.recorded_goals)}\n\n")
                
                f.write("# 详细信息（包含方向）\n")
                f.write("detailed_goals = {\n")
                for i, goal in enumerate(self.recorded_goals):
                    f.write(f"    {i}: {{\n")
                    f.write(f"        'position': {goal['position']},\n")
                    f.write(f"        'yaw': {goal['yaw_radians']},\n")
                    f.write(f"        'yaw_degrees': {goal['yaw_degrees']}\n")
                    f.write(f"    }},\n")
                f.write("}\n")
            
            rospy.loginfo(f"🐍 Python格式已保存到: {py_filename}")
            
        except Exception as e:
            rospy.logerr(f"保存文件失败: {e}")
    
    def run(self):
        """运行记录器"""
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("\n\n🛑 停止记录...")
            self.print_current_summary()
            self.save_to_file()
            print("\n✅ 目标点记录完成!")

def main():
    try:
        recorder = GoalRecorder()
        recorder.run()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()





