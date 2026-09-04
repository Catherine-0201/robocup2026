#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
简洁版导航脚本 - 依赖C++控制器判断目标到达
功能：按顺序发送导航目标，监听C++控制器的到达确认信号
"""

import rospy
import actionlib
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import Int32

class SimpleNavigator:
    def __init__(self):
        rospy.init_node('simple_navigator', anonymous=True)
        
        # 目标点数据
        self.goal_points = [
            {'position': [1.600, 0.000], 'yaw': 0.025, 'name': '目标点1'},
            {'position': [5.500, 2.500], 'yaw': 0.040, 'name': '目标点2'}, 
            {'position': [5.500, -1.600], 'yaw': -0.015, 'name': '目标点3'},
            {'position': [0.000, 0.000], 'yaw': 0.0, 'name': '目标点4'}
        ]
        
        # 配置参数
        self.stay_duration = 12.0  # 每个点停留时间
        self.nav_timeout = 30.0    # 导航超时时间
        
        # 状态变量
        self.completed_targets = 0  # 已完成的目标数量
        self.current_goal_index = 0  # 当前目标索引
        self.move_base_client = None
        
        # 初始化组件
        self.init_move_base_client()
        self.init_target_done_subscriber()
        
        rospy.loginfo("简洁导航器初始化完成")
        rospy.loginfo(f"共有 {len(self.goal_points)} 个目标点")
        rospy.loginfo(f"每个点停留 {self.stay_duration} 秒")

    def init_move_base_client(self):
        """初始化move_base客户端"""
        rospy.loginfo("连接move_base服务器...")
        
        try:
            self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            
            if self.move_base_client.wait_for_server(rospy.Duration(30)):
                rospy.loginfo("move_base服务器连接成功")
            else:
                rospy.logerr("move_base服务器连接超时")
                rospy.signal_shutdown("move_base服务器不可用")
                
        except Exception as e:
            rospy.logerr(f"move_base客户端初始化失败: {e}")
            rospy.signal_shutdown("move_base初始化失败")

    def init_target_done_subscriber(self):
        """订阅target_done话题，监听C++控制器的目标完成计数"""
        rospy.loginfo("订阅target_done话题，监听目标完成信号...")
        rospy.Subscriber('/target_done', Int32, self.target_done_callback)

    def target_done_callback(self, msg):
        """target_done回调函数，检查C++控制器的目标完成计数"""
        if msg.data > self.completed_targets:
            self.completed_targets = msg.data
            rospy.loginfo(f"收到目标完成信号，总完成数: {msg.data}")
            
            # 检查是否是当前目标完成
            if msg.data == self.current_goal_index + 1:
                rospy.loginfo(f"当前目标 {self.current_goal_index + 1} 已完成")

    def create_goal_message(self, goal_point):
        """创建move_base目标消息"""
        goal = MoveBaseGoal()
        
        # 设置目标帧
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # 设置位置
        goal.target_pose.pose.position.x = goal_point['position'][0]
        goal.target_pose.pose.position.y = goal_point['position'][1]
        goal.target_pose.pose.position.z = 0.0
        
        # 设置方向
        yaw = goal_point['yaw']
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        return goal

    def navigate_to_goal(self, goal_point, goal_index):
        """导航到指定目标点"""
        rospy.loginfo(f"\n开始导航到 {goal_point['name']} ({goal_index + 1}/{len(self.goal_points)})")
        rospy.loginfo(f"目标坐标: ({goal_point['position'][0]:.3f}, {goal_point['position'][1]:.3f})")
        
        # 设置当前目标索引
        self.current_goal_index = goal_index
        expected_count = goal_index + 1
        
        # 创建并发送目标
        goal_msg = self.create_goal_message(goal_point)
        
        try:
            rospy.loginfo("发送导航目标...")
            self.move_base_client.send_goal(goal_msg)
            
            # 等待C++控制器确认到达
            start_time = rospy.Time.now()
            
            while not rospy.is_shutdown() and self.completed_targets < expected_count:
                # 检查超时
                elapsed_time = (rospy.Time.now() - start_time).to_sec()
                if elapsed_time > self.nav_timeout:
                    rospy.logwarn("导航超时，取消目标")
                    self.move_base_client.cancel_goal()
                    return False
                
                # 检查move_base状态
                goal_state = self.move_base_client.get_state()
                if goal_state in [GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.RECALLED]:
                    rospy.logwarn(f"导航失败，move_base状态: {goal_state}")
                    return False
                
                # 每2秒输出一次状态
                if int(elapsed_time) % 2 == 0 and elapsed_time > 0:
                    if int(elapsed_time) != getattr(self, '_last_log_time', -1):
                        rospy.loginfo(f"导航进行中... 已用时 {elapsed_time:.0f}s，等待目标完成计数达到 {expected_count}")
                        self._last_log_time = int(elapsed_time)
                
                rospy.sleep(0.1)
            
            if self.completed_targets >= expected_count:
                rospy.loginfo(f"成功到达 {goal_point['name']} (完成计数: {self.completed_targets})")
                return True
            else:
                rospy.logwarn(f"导航被中断: {goal_point['name']}")
                return False
                
        except Exception as e:
            rospy.logerr(f"导航过程异常: {e}")
            return False

    def stay_at_goal(self, goal_point, goal_index):
        """在目标点停留指定时间"""
        rospy.loginfo(f"在 {goal_point['name']} 停留 {self.stay_duration} 秒...")
        
        # 停留倒计时
        remaining_time = self.stay_duration
        while remaining_time > 0 and not rospy.is_shutdown():
            if remaining_time <= 5 or int(remaining_time) % 2 == 0:
                rospy.loginfo(f"剩余停留时间: {remaining_time:.0f} 秒")
            
            rospy.sleep(1.0)
            remaining_time -= 1.0
        
        if goal_index < len(self.goal_points) - 1:
            next_goal = self.goal_points[goal_index + 1]
            rospy.loginfo(f"{goal_point['name']} 停留完成")
            rospy.loginfo(f"准备前往下一个目标: {next_goal['name']}")
        else:
            rospy.loginfo(f"{goal_point['name']} 停留完成，所有任务完成!")

    def run_navigation(self):
        """执行完整的导航任务"""
        rospy.loginfo("开始顺序导航任务")
        
        total_goals = len(self.goal_points)
        successful_goals = 0
        
        for goal_index, goal_point in enumerate(self.goal_points):
            if rospy.is_shutdown():
                rospy.loginfo("接收到关闭信号，终止导航")
                break
            
            # 导航到目标点
            success = self.navigate_to_goal(goal_point, goal_index)
            
            if success:
                successful_goals += 1
                # 在目标点停留
                self.stay_at_goal(goal_point, goal_index)
            else:
                rospy.logwarn(f"导航到 {goal_point['name']} 失败，继续下一个目标")
        
        # 任务总结
        self.print_summary(successful_goals, total_goals)

    def print_summary(self, successful_goals, total_goals):
        """打印任务总结"""
        rospy.loginfo("\n" + "="*50)
        rospy.loginfo("导航任务完成总结")
        rospy.loginfo("="*50)
        rospy.loginfo(f"总目标点数: {total_goals}")
        rospy.loginfo(f"成功到达: {successful_goals}")
        rospy.loginfo(f"失败次数: {total_goals - successful_goals}")
        rospy.loginfo(f"成功率: {(successful_goals/total_goals*100):.1f}%")
        
        if successful_goals == total_goals:
            rospy.loginfo("恭喜！所有目标点都成功到达!")
        else:
            rospy.logwarn(f"有 {total_goals - successful_goals} 个目标点未能到达")
        
        rospy.loginfo("="*50)

    def emergency_stop(self):
        """紧急停止"""
        rospy.logwarn("执行紧急停止")
        if self.move_base_client:
            self.move_base_client.cancel_all_goals()

def main():
    """主函数"""
    try:
        # 创建导航器
        navigator = SimpleNavigator()
        
        # 显示启动信息
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("简洁版导航器准备就绪")
        rospy.loginfo("即将按顺序导航到以下目标点:")
        
        for i, goal in enumerate(navigator.goal_points):
            rospy.loginfo(f"   {i+1}. {goal['name']}: ({goal['position'][0]:.3f}, {goal['position'][1]:.3f})")
        
        rospy.loginfo(f"每个点停留 {navigator.stay_duration} 秒")
        rospy.loginfo("="*60)
        
        # 等待启动
        rospy.loginfo("3秒后开始导航...")
        for i in range(3, 0, -1):
            rospy.loginfo(f"{i}...")
            rospy.sleep(1.0)
        
        # 执行导航任务
        navigator.run_navigation()
        
        rospy.loginfo("导航程序结束")
        
    except rospy.ROSInterruptException:
        rospy.loginfo("接收到中断信号，正在退出...")
    except KeyboardInterrupt:
        rospy.loginfo("用户中断，正在退出...")
    except Exception as e:
        rospy.logerr(f"程序异常: {e}")
    finally:
        if 'navigator' in locals():
            navigator.emergency_stop()
        rospy.loginfo("再见!")

if __name__ == '__main__':
    main()
