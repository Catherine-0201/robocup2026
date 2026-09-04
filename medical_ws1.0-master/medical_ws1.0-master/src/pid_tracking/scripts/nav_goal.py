#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
导航目标点脚本 - 按顺序导航到指定的目标点
功能：
1. 依次发布4个目标点
2. 机器人按顺序导航过去
3. 每个点停留10秒
4. 到达最后一个点后程序结束

使用前提：
- 底盘系统已启动
- 雷达定位系统已启动  
- 导航系统已启动

使用方法：
python nav_goal.py
"""

import rospy
import actionlib
import math
import time
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf.transformations

class SequentialNavigator:
    def __init__(self):
        rospy.init_node('sequential_navigator', anonymous=True)
        
        # 记录的目标点数据
        self.goal_points = [
            {'position': [1.626, -0.000], 'yaw': 0.0, 'name': '目标点1'},
            {'position': [5.750, 2.400], 'yaw': 0.0, 'name': '目标点2'}, 
            {'position': [5.750, -2.400], 'yaw': 0.0, 'name': '目标点3'},
            {'position': [0.000, 0.000], 'yaw': 0.0, 'name': '目标点4'}
        ]
        
        # 每个点的停留时间（秒）
        self.stay_duration = 10.0
        
        # 导航超时时间（秒）
        self.nav_timeout = 20.0
        
        # 目标到达容忍度（与C++控制器完全一致）
        self.position_tolerance = 0.25  # 与C++控制器goal_dist_tolerance一致
        self.angle_tolerance = 1.0      # 放宽角度检查，更接近C++只检查位置的逻辑
        
        # 机器人当前位置
        self.current_pose = None
        self.current_goal = None
        self.last_goal_state = None
        
        # 初始化move_base客户端
        self.move_base_client = None
        self.init_move_base_client()
        
        # 订阅机器人位置信息
        self.init_pose_subscriber()
        
        rospy.loginfo("🤖 顺序导航器已初始化")
        rospy.loginfo(f"📍 共有 {len(self.goal_points)} 个目标点")
        rospy.loginfo(f"⏰ 每个点停留 {self.stay_duration} 秒")

    def init_move_base_client(self):
        """初始化move_base行动客户端"""
        rospy.loginfo("🔗 连接move_base服务器...")
        
        try:
            self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            
            # 等待move_base服务器启动
            rospy.loginfo("⏳ 等待move_base服务器...")
            if self.move_base_client.wait_for_server(rospy.Duration(30)):
                rospy.loginfo("✅ move_base服务器连接成功!")
            else:
                rospy.logerr("❌ move_base服务器连接超时!")
                rospy.signal_shutdown("move_base服务器不可用")
                return False
                
        except Exception as e:
            rospy.logerr(f"❌ move_base客户端初始化失败: {e}")
            return False
            
        return True

    def init_pose_subscriber(self):
        """初始化机器人位置订阅器"""
        try:
            # 等待定位节点启动
            rospy.loginfo("⏳ 等待定位节点启动...")
            rospy.sleep(5.0)  # 等待5秒确保节点启动
            
            # 支持命名空间
            ns = rospy.get_namespace()
            pose_topic = f"{ns}unified_pose"
            
            rospy.loginfo(f"⏳ 尝试订阅位置话题: {pose_topic}")
            rospy.wait_for_message(pose_topic, PoseWithCovarianceStamped, timeout=5.0)
            rospy.Subscriber(pose_topic, PoseWithCovarianceStamped, self.pose_callback)
            rospy.loginfo(f"✅ 订阅位置话题: {pose_topic}")
                
        except Exception as e:
            rospy.logerr(f"❌ 位置话题 {pose_topic} 不可用: {e}")
            rospy.logerr("❌ 位置信息对验证目标到达至关重要，无法继续运行。")
            rospy.signal_shutdown("无可用位置话题")

    def pose_callback(self, msg):
        """处理PoseWithCovarianceStamped消息"""
        self.current_pose = msg.pose.pose
        
    def calculate_distance_to_goal(self, goal_point):
        """计算当前位置到目标点的距离"""
        if self.current_pose is None:
            return float('inf')
            
        dx = self.current_pose.position.x - goal_point['position'][0]
        dy = self.current_pose.position.y - goal_point['position'][1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        return distance

    def calculate_angle_difference(self, goal_point):
        """计算当前角度与目标角度的差值"""
        if self.current_pose is None:
            return float('inf')
            
        # 获取当前yaw角
        current_quat = [
            self.current_pose.orientation.x,
            self.current_pose.orientation.y, 
            self.current_pose.orientation.z,
            self.current_pose.orientation.w
        ]
        current_yaw = tf.transformations.euler_from_quaternion(current_quat)[2]
        
        # 计算角度差
        target_yaw = goal_point['yaw']
        angle_diff = abs(current_yaw - target_yaw)
        
        # 处理角度跨越±π的情况
        if angle_diff > math.pi:
            angle_diff = 2*math.pi - angle_diff
            
        return angle_diff

    def check_goal_reached_manual(self, goal_point):
        """手动检查是否到达目标点"""
        if self.current_pose is None:
            return False
            
        distance = self.calculate_distance_to_goal(goal_point)
        angle_diff = self.calculate_angle_difference(goal_point)
        
        position_ok = distance <= self.position_tolerance
        angle_ok = angle_diff <= self.angle_tolerance
        
        rospy.loginfo(f"📏 距离检查: {distance:.3f}m (容忍度: {self.position_tolerance}m) {'✅' if position_ok else '❌'}")
        rospy.loginfo(f"🧭 角度检查: {math.degrees(angle_diff):.1f}° (容忍度: {math.degrees(self.angle_tolerance):.1f}°) {'✅' if angle_ok else '❌'}")
        
        return position_ok and angle_ok

    def create_goal_message(self, goal_point):
        """创建MoveBase目标消息"""
        goal = MoveBaseGoal()
        
        # 设置目标帧
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # 设置位置
        goal.target_pose.pose.position.x = goal_point['position'][0]
        goal.target_pose.pose.position.y = goal_point['position'][1]
        goal.target_pose.pose.position.z = 0.0
        
        # 设置方向（从yaw角转换为四元数）
        yaw = goal_point['yaw']
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return goal

    def print_detailed_goal_info(self, goal_point, goal_index):
        """显示详细的目标点信息"""
        print("\n" + "🚀"*30 + " 导航开始 " + "🚀"*30)
        rospy.loginfo(f"🎯 目标点编号: {goal_index + 1}/{len(self.goal_points)}")
        rospy.loginfo(f"📝 目标名称: {goal_point['name']}")
        
        # 目标点详细信息
        target_x = goal_point['position'][0]
        target_y = goal_point['position'][1]
        target_yaw = goal_point['yaw']
        
        # 当前位置信息和快速验证格式
        if self.current_pose:
            current_x = self.current_pose.position.x
            current_y = self.current_pose.position.y
            
            # 获取当前yaw角
            current_quat = [
                self.current_pose.orientation.x,
                self.current_pose.orientation.y,
                self.current_pose.orientation.z,
                self.current_pose.orientation.w
            ]
            current_yaw = tf.transformations.euler_from_quaternion(current_quat)[2]
            
            # 计算需要移动的距离和角度
            distance = self.calculate_distance_to_goal(goal_point)
            angle_diff = self.calculate_angle_difference(goal_point)
            
            # 您要求的简洁验证格式
            rospy.loginfo(f"🔍 导航前验证: 当前p({current_x:.2f},{current_y:.2f}),y({math.degrees(current_yaw):.0f}°), 目标p({target_x:.2f},{target_y:.2f}),y({math.degrees(target_yaw):.0f}°), 差距p {distance:.2f}米，y {math.degrees(angle_diff):.0f}度")
            
            # 详细信息
            rospy.loginfo(f"📍 目标位置: X={target_x:.3f}m, Y={target_y:.3f}m")
            rospy.loginfo(f"🧭 目标方向: {math.degrees(target_yaw):.1f}°")
            rospy.loginfo(f"🤖 当前位置: X={current_x:.3f}m, Y={current_y:.3f}m")
            rospy.loginfo(f"🔄 当前方向: {math.degrees(current_yaw):.1f}°")
            
            # 预估时间（简单估算）
            estimated_time = max(distance / 0.3, math.degrees(angle_diff) / 30)  # 假设0.3m/s移动，30°/s转动
            rospy.loginfo(f"⏰ 预估到达时间: {estimated_time:.1f}秒")
        else:
            rospy.logwarn("❌ 无法获取当前位置，无法计算距离")
        
        rospy.loginfo(f"🎯 容忍度设置: 位置±{self.position_tolerance:.3f}m, 角度±{math.degrees(self.angle_tolerance):.1f}°")
        print("🚀"*80)

    def navigate_to_goal(self, goal_point, goal_index):
        """导航到指定目标点，支持重试"""
        max_retries = 2  # 最大重试次数
        for attempt in range(max_retries + 1):  # 包括初始尝试
            rospy.loginfo(f"\n🎯 开始导航到 {goal_point['name']} ({goal_index + 1}/{len(self.goal_points)}) - 第 {attempt + 1} 次尝试")
            navigation_success = self._navigate_to_goal(goal_point, goal_index)
            if navigation_success:
                return True
            if attempt < max_retries:
                rospy.logwarn(f"⚠️ 第{attempt + 1}次尝试失败，重试...")
                rospy.sleep(1.0)
            else:
                rospy.logerr(f"❌ 达到最大重试次数，放弃目标: {goal_point['name']}")
        return False

    def _navigate_to_goal(self, goal_point, goal_index):
        """内部导航逻辑"""
        rospy.loginfo(f"📍 目标坐标: ({goal_point['position'][0]:.3f}, {goal_point['position'][1]:.3f})")
        rospy.loginfo(f"🧭 目标方向: {math.degrees(goal_point['yaw']):.1f}°")
        
        # 显示完整的初始状态信息
        self.print_detailed_goal_info(goal_point, goal_index)
        
        # 创建目标消息
        goal_msg = self.create_goal_message(goal_point)
        self.current_goal = goal_point
        
        try:
            # 发送目标
            rospy.loginfo("🚀 发送导航目标...")
            self.move_base_client.send_goal(goal_msg)
            
            # 增强的等待和监控循环
            start_time = rospy.Time.now()
            last_status_time = start_time
            check_interval = 2.0  # 每2秒检查一次状态
            
            while not rospy.is_shutdown():
                current_time = rospy.Time.now()
                elapsed_time = (current_time - start_time).to_sec()
                
                # 检查超时
                if elapsed_time > self.nav_timeout:
                    rospy.logwarn("⚠️ 导航超时，取消当前目标")
                    self.move_base_client.cancel_goal()
                    return False
                
                # 检查move_base状态
                goal_state = self.move_base_client.get_state()
                
                # 详细状态输出（每2秒或状态变化时）
                if (current_time - last_status_time).to_sec() >= check_interval or goal_state != self.last_goal_state:
                    self.print_navigation_status(goal_point, elapsed_time, goal_state)
                    self.last_goal_state = goal_state
                    last_status_time = current_time
                
                # 检查是否完成
                if goal_state in [GoalStatus.SUCCEEDED, GoalStatus.ABORTED, 
                                 GoalStatus.REJECTED, GoalStatus.RECALLED, 
                                 GoalStatus.PREEMPTED]:
                    break
                
                # 手动检查是否到达（备用方案）
                if self.current_pose and self.check_goal_reached_manual(goal_point):
                    rospy.loginfo("✅ 手动检查确认已到达目标点!")
                    # 给move_base一点时间完成状态更新
                    rospy.sleep(1.0)
                    goal_state = self.move_base_client.get_state()
                    if goal_state not in [GoalStatus.SUCCEEDED]:
                        rospy.loginfo("🔄 move_base状态未更新，手动确认到达")
                        return True
                
                rospy.sleep(0.5)  # 减少CPU使用
            
            # 最终状态检查
            final_state = self.move_base_client.get_state()
            rospy.loginfo(f"🏁 最终导航状态: {self.get_status_name(final_state)}")
            
            if final_state == GoalStatus.SUCCEEDED:
                rospy.loginfo(f"✅ 成功到达 {goal_point['name']}!")
                return True
            elif final_state == GoalStatus.ABORTED:
                rospy.logwarn(f"❌ 导航被中止: {goal_point['name']}")
                # 最后一次手动检查
                if self.current_pose and self.check_goal_reached_manual(goal_point):
                    rospy.loginfo("✅ 尽管move_base报告中止，但手动检查确认已到达目标!")
                    return True
                return False
            elif final_state == GoalStatus.REJECTED:
                rospy.logwarn(f"❌ 目标被拒绝: {goal_point['name']}")
                return False
            else:
                rospy.logwarn(f"❌ 导航失败，状态码: {final_state}")
                # 最后一次手动检查
                if self.current_pose and self.check_goal_reached_manual(goal_point):
                    rospy.loginfo("✅ 尽管move_base报告失败，但手动检查确认已到达目标!")
                    return True
                return False
                
        except Exception as e:
            rospy.logerr(f"❌ 导航过程中发生异常: {e}")
            return False

    def get_status_name(self, status):
        """获取状态名称"""
        status_names = {
            GoalStatus.PENDING: "PENDING",
            GoalStatus.ACTIVE: "ACTIVE", 
            GoalStatus.PREEMPTED: "PREEMPTED",
            GoalStatus.SUCCEEDED: "SUCCEEDED",
            GoalStatus.ABORTED: "ABORTED",
            GoalStatus.REJECTED: "REJECTED",
            GoalStatus.PREEMPTING: "PREEMPTING",
            GoalStatus.RECALLING: "RECALLING",
            GoalStatus.RECALLED: "RECALLED",
            GoalStatus.LOST: "LOST"
        }
        return status_names.get(status, f"UNKNOWN({status})")

    def print_navigation_status(self, goal_point, elapsed_time, goal_state):
        """打印导航状态信息"""
        status_name = self.get_status_name(goal_state)
        
        # 目标点信息
        target_x = goal_point['position'][0]
        target_y = goal_point['position'][1] 
        target_yaw = goal_point['yaw']
        
        # 当前位置信息和实时输出
        if self.current_pose:
            current_x = self.current_pose.position.x
            current_y = self.current_pose.position.y
            
            # 获取当前yaw角
            current_quat = [
                self.current_pose.orientation.x,
                self.current_pose.orientation.y, 
                self.current_pose.orientation.z,
                self.current_pose.orientation.w
            ]
            current_yaw = tf.transformations.euler_from_quaternion(current_quat)[2]
            
            # 计算差距
            distance = self.calculate_distance_to_goal(goal_point)
            angle_diff = self.calculate_angle_difference(goal_point)
            
            # 您要求的简洁格式输出
            rospy.loginfo(f"当前p({current_x:.2f},{current_y:.2f}),y({math.degrees(current_yaw):.0f}°), 目标p({target_x:.2f},{target_y:.2f}),y({math.degrees(target_yaw):.0f}°), 差距p {distance:.2f}米，y {math.degrees(angle_diff):.0f}度")
            
            # 详细状态输出
            print("\n" + "="*80)
            rospy.loginfo(f"🎯 {goal_point['name']} | ⏰ {elapsed_time:.1f}s | 状态: {status_name}")
            
            # 显示到达状态
            position_ok = distance <= self.position_tolerance
            angle_ok = angle_diff <= self.angle_tolerance
            rospy.loginfo(f"✅ 位置达标: {'是' if position_ok else '否'} (容忍度: {self.position_tolerance:.2f}m) | 角度达标: {'是' if angle_ok else '否'} (容忍度: {math.degrees(self.angle_tolerance):.0f}°)")
            print("="*80)
        else:
            rospy.logwarn("❌ 无法获取当前位置信息")

    def stay_at_goal(self, goal_point, goal_index):
        """在目标点停留指定时间"""
        print("\n" + "⏸️"*30 + " 停留阶段 " + "⏸️"*30)
        rospy.loginfo(f"✅ 已到达 {goal_point['name']}")
        rospy.loginfo(f"⏸️ 开始停留 {self.stay_duration} 秒...")
        
        # 显示当前最终位置
        if self.current_pose:
            current_x = self.current_pose.position.x
            current_y = self.current_pose.position.y
            current_quat = [
                self.current_pose.orientation.x,
                self.current_pose.orientation.y,
                self.current_pose.orientation.z,
                self.current_pose.orientation.w
            ]
            current_yaw = tf.transformations.euler_from_quaternion(current_quat)[2]
            
            rospy.loginfo(f"🎯 目标位置: ({goal_point['position'][0]:.3f}, {goal_point['position'][1]:.3f})")
            rospy.loginfo(f"🤖 实际位置: ({current_x:.3f}, {current_y:.3f})")
            rospy.loginfo(f"🧭 目标方向: {math.degrees(goal_point['yaw']):.1f}°")
            rospy.loginfo(f"🔄 实际方向: {math.degrees(current_yaw):.1f}°")
            
            # 显示最终精度
            distance = self.calculate_distance_to_goal(goal_point)
            angle_diff = self.calculate_angle_difference(goal_point)
            rospy.loginfo(f"📏 位置误差: {distance:.3f}m")
            rospy.loginfo(f"📐 角度误差: {math.degrees(angle_diff):.1f}°")
        
        print("⏸️"*80)
        
        # 显示倒计时
        remaining_time = self.stay_duration
        while remaining_time > 0 and not rospy.is_shutdown():
            if remaining_time <= 5 or remaining_time % 2 == 0:
                rospy.loginfo(f"⏰ 在 {goal_point['name']} 剩余停留时间: {remaining_time:.0f} 秒")
            
            rospy.sleep(1.0)
            remaining_time -= 1.0
        
        if goal_index < len(self.goal_points) - 1:
            next_goal = self.goal_points[goal_index + 1]
            rospy.loginfo(f"✅ {goal_point['name']} 停留完成")
            rospy.loginfo(f"➡️ 准备前往下一个目标: {next_goal['name']}")
        else:
            rospy.loginfo(f"🏁 {goal_point['name']} 停留完成，所有任务完成!")

    def run_sequential_navigation(self):
        """执行顺序导航任务"""
        rospy.loginfo("🚀 开始顺序导航任务!")
        
        total_goals = len(self.goal_points)
        successful_goals = 0
        
        for goal_index, goal_point in enumerate(self.goal_points):
            if rospy.is_shutdown():
                rospy.loginfo("🛑 接收到关闭信号，终止导航")
                break
            
            # 导航到目标点
            navigation_success = self.navigate_to_goal(goal_point, goal_index)
            
            if navigation_success:
                successful_goals += 1
                
                # 在目标点停留
                self.stay_at_goal(goal_point, goal_index)
                
            else:
                # 导航失败，继续下一个目标
                rospy.logwarn(f"⚠️ 导航到 {goal_point['name']} 失败! 继续下一个目标")
        
        # 任务完成总结
        self.print_navigation_summary(successful_goals, total_goals)

    def print_navigation_summary(self, successful_goals, total_goals):
        """打印导航任务总结"""
        rospy.loginfo("\n" + "="*50)
        rospy.loginfo("📊 导航任务完成总结")
        rospy.loginfo("="*50)
        rospy.loginfo(f"🎯 总目标点数: {total_goals}")
        rospy.loginfo(f"✅ 成功到达: {successful_goals}")
        rospy.loginfo(f"❌ 失败次数: {total_goals - successful_goals}")
        rospy.loginfo(f"📈 成功率: {(successful_goals/total_goals*100):.1f}%")
        
        if successful_goals == total_goals:
            rospy.loginfo("🎉 恭喜！所有目标点都成功到达!")
        else:
            rospy.logwarn(f"⚠️ 有 {total_goals - successful_goals} 个目标点未能到达")
        
        rospy.loginfo("="*50)

    def emergency_stop(self):
        """紧急停止"""
        rospy.logwarn("🚨 执行紧急停止!")
        if self.move_base_client:
            self.move_base_client.cancel_all_goals()
        rospy.loginfo("✅ 所有导航目标已取消")

def main():
    """主函数"""
    try:
        # 创建导航器实例
        navigator = SequentialNavigator()
        
        # 等待用户确认开始
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("🤖 顺序导航器准备就绪!")
        rospy.loginfo("📋 即将按顺序导航到以下目标点:")
        
        for i, goal in enumerate(navigator.goal_points):
            rospy.loginfo(f"   {i+1}. {goal['name']}: ({goal['position'][0]:.3f}, {goal['position'][1]:.3f})")
        
        rospy.loginfo(f"⏰ 每个点停留 {navigator.stay_duration} 秒")
        rospy.loginfo("="*60)
        
        # 等待2秒让用户看到信息
        rospy.loginfo("⏳ 3秒后开始导航...")
        for i in range(3, 0, -1):
            rospy.loginfo(f"🔢 {i}...")
            rospy.sleep(1.0)
        
        # 开始导航
        navigator.run_sequential_navigation()
        
        # 完成提示
        rospy.loginfo("🏁 顺序导航程序结束")
        
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 接收到中断信号，正在退出...")
    except KeyboardInterrupt:
        rospy.loginfo("🛑 用户中断，正在退出...")
    except Exception as e:
        rospy.logerr(f"❌ 程序异常: {e}")
    finally:
        if 'navigator' in locals():
            navigator.emergency_stop()
        rospy.loginfo("👋 再见!")

if __name__ == '__main__':
    main()