#!/usr/bin/env python
from pid_node import PIDTrackingNode
import smach
import rospy
import smach_ros
import sys
import argparse
import numpy as np

class WallAlignmentState(smach.State):
    """墙壁角度对齐状态，确保机器人垂直墙壁方向"""
    def __init__(self, node, target_angle=0.0, tolerance=0.1):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.node = node
        self.target_angle = target_angle  # 目标角度（弧度）
        self.tolerance = tolerance        # 角度容差（弧度）

    def execute(self, userdata):
        rospy.loginfo(f"Starting wall alignment to angle {self.target_angle:.3f} rad")
        
        # 等待初始化完成
        while not self.node.init_flag and not rospy.is_shutdown():
            rospy.sleep(0.1)
        
        if rospy.is_shutdown():
            return 'failed'
        
        # 角度对齐循环
        while not rospy.is_shutdown():
            current_angle = self.node.currant_distance_state[2]
            angle_error = current_angle - self.target_angle
            
            # 处理角度环绕问题
            if angle_error > np.pi:
                angle_error -= 2 * np.pi
            elif angle_error < -np.pi:
                angle_error += 2 * np.pi
            
            rospy.loginfo(f"Current angle: {current_angle:.3f}, Target: {self.target_angle:.3f}, Error: {angle_error:.3f}")
            
            # 检查是否达到目标角度
            if abs(angle_error) < self.tolerance:
                self.node.publish_vel(0, 0, 0)
                rospy.loginfo(f"Wall alignment completed. Final angle: {current_angle:.3f}")
                return 'succeeded'
            
            # 使用角度PID控制器调整角度
            angle_pid_value = self.node.angle_pid.calc(self.target_angle, current_angle)
            self.node.publish_vel(0, 0, -angle_pid_value)
            
            rospy.sleep(0.1)
        
        return 'failed'

class TaskState(smach.State):
    def __init__(self, node, goal_idx):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.node = node
        self.goal_idx = goal_idx

    def execute(self, userdata):
        rospy.loginfo(f"Executing task to goal {self.goal_idx}")
        self.node.task(self.node.goal_point[self.goal_idx][0], self.node.goal_point[self.goal_idx][1])
        return 'succeeded'

class PositionLoopState(smach.State):
    def __init__(self, node, distance, axis):
        smach.State.__init__(self, outcomes=['succeeded', 'in_progress'])
        self.node = node
        self.distance = distance  # 目标距离
        self.axis = axis          # 用来选择距离检查的维度
        # 采用与task_new.py一致的策略：循环直到达到目标距离

    def execute(self, userdata):
        current_distance = self.node.distance[self.axis]
        rospy.loginfo(f"Position loop on axis {self.axis}, target: {self.distance}, current: {current_distance}")
        
        # 采用与task_new.py一致的策略：循环直到达到目标距离
        if abs(current_distance - self.distance) > 0.01:
            self.node.distance_state = self.axis + 1
            self.node.position_loop(self.distance)
            return 'in_progress'
        
        self.node.publish_vel(0, 0, 0)  # 停止运动
        rospy.loginfo(f"Position {self.axis} reached, distance: {current_distance}")
        return 'succeeded'

class WaitState(smach.State):
    def __init__(self, wait_time):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.wait_time = wait_time

    def execute(self, userdata):
        rospy.loginfo(f"Waiting for {self.wait_time} seconds...")
        rospy.sleep(self.wait_time)
        rospy.loginfo("Wait completed")
        return 'succeeded'

class TestTask:
    def __init__(self, task_type):
        # 初始化PIDTrackingNode实例，让它自己处理节点初始化
        self.node = PIDTrackingNode()
        self.task_type = task_type
        
        # 设置目标距离参数（单位：米）
        # 前方墙壁110cm，左/右方墙壁30cm，安全距离8cm
        self.front_distance = 1.10    # 110cm
        self.side_distance = 0.22     # 30cm - 8cm = 22cm
        
        # 创建状态机
        self.sm = smach.StateMachine(outcomes=['TASK_COMPLETED'])

    def config(self):
        if self.task_type == 1:  # 先左后右
            rospy.loginfo("Configuring task: Left first, then Right")
            with self.sm:
                # 第一步：移动到距离前向墙壁2m的位置
                smach.StateMachine.add('MOVE_TO_2M', PositionLoopState(self.node, 2.0, axis=0), 
                                     transitions={'succeeded': 'WALL_ALIGNMENT_FRONT_2M', 'in_progress': 'MOVE_TO_2M'})
                
                # 第二步：在2m位置进行墙壁角度垂直对齐（确保机器人垂直前方墙壁）
                smach.StateMachine.add('WALL_ALIGNMENT_FRONT_2M', WallAlignmentState(self.node, target_angle=0.0, tolerance=0.05), 
                                     transitions={'succeeded': 'FORWARD_TO_110CM', 'failed': 'TASK_COMPLETED'})
                
                # 第三步：前向移动到距离110cm
                smach.StateMachine.add('FORWARD_TO_110CM', PositionLoopState(self.node, self.front_distance, axis=0), 
                                     transitions={'succeeded': 'LEFT_ALIGN_50CM', 'in_progress': 'FORWARD_TO_110CM'})
                
                # 第四步：向左对齐至50cm
                smach.StateMachine.add('LEFT_ALIGN_50CM', PositionLoopState(self.node, 0.50, axis=1), 
                                     transitions={'succeeded': 'WALL_ALIGNMENT_LEFT_50CM', 'in_progress': 'LEFT_ALIGN_50CM'})
                
                # 第五步：在50cm位置进行左侧墙壁角度垂直对齐
                smach.StateMachine.add('WALL_ALIGNMENT_LEFT_50CM', WallAlignmentState(self.node, target_angle=np.pi/2, tolerance=0.05), 
                                     transitions={'succeeded': 'LEFT_ALIGN_30CM', 'failed': 'TASK_COMPLETED'})
                
                # 第六步：向左移动对齐至30cm，到达目标点1
                smach.StateMachine.add('LEFT_ALIGN_30CM', PositionLoopState(self.node, 0.30, axis=1), 
                                     transitions={'succeeded': 'WAIT_AT_LEFT', 'in_progress': 'LEFT_ALIGN_30CM'})
                
                # 第七步：在左侧停留10秒
                smach.StateMachine.add('WAIT_AT_LEFT', WaitState(10), 
                                     transitions={'succeeded': 'LEFT_ALIGN_50CM_AGAIN'})
                
                # 第八步：向左对齐至50cm
                smach.StateMachine.add('LEFT_ALIGN_50CM_AGAIN', PositionLoopState(self.node, 0.50, axis=1), 
                                     transitions={'succeeded': 'FORWARD_TO_2M_AGAIN', 'in_progress': 'LEFT_ALIGN_50CM_AGAIN'})
                
                # 第九步：向前对齐至2m
                smach.StateMachine.add('FORWARD_TO_2M_AGAIN', PositionLoopState(self.node, 2.0, axis=0), 
                                     transitions={'succeeded': 'RIGHT_ALIGN_50CM', 'in_progress': 'FORWARD_TO_2M_AGAIN'})
                
                # 第十步：向右移动对齐至50cm
                smach.StateMachine.add('RIGHT_ALIGN_50CM', PositionLoopState(self.node, 0.50, axis=2), 
                                     transitions={'succeeded': 'WALL_ALIGNMENT_FRONT_2M_AGAIN', 'in_progress': 'RIGHT_ALIGN_50CM'})
                
                # 第十一步：在2m位置进行前向墙壁角度垂直对齐
                smach.StateMachine.add('WALL_ALIGNMENT_FRONT_2M_AGAIN', WallAlignmentState(self.node, target_angle=0.0, tolerance=0.05), 
                                     transitions={'succeeded': 'FORWARD_TO_110CM_AGAIN', 'failed': 'TASK_COMPLETED'})
                
                # 第十二步：向前距离至110cm
                smach.StateMachine.add('FORWARD_TO_110CM_AGAIN', PositionLoopState(self.node, self.front_distance, axis=0), 
                                     transitions={'succeeded': 'WALL_ALIGNMENT_RIGHT_110CM', 'in_progress': 'FORWARD_TO_110CM_AGAIN'})
                
                # 第十三步：在110cm位置进行右侧墙壁角度垂直对齐
                smach.StateMachine.add('WALL_ALIGNMENT_RIGHT_110CM', WallAlignmentState(self.node, target_angle=-np.pi/2, tolerance=0.05), 
                                     transitions={'succeeded': 'RIGHT_ALIGN_30CM', 'failed': 'TASK_COMPLETED'})
                
                # 第十四步：向右墙壁对齐至30cm，到达任务点2
                smach.StateMachine.add('RIGHT_ALIGN_30CM', PositionLoopState(self.node, 0.30, axis=2), 
                                     transitions={'succeeded': 'TASK_COMPLETED', 'in_progress': 'RIGHT_ALIGN_30CM'})

        elif self.task_type == 2:  # 先右后左
            rospy.loginfo("Configuring task: Right first, then Left")
            with self.sm:
                # 第一步：移动到距离前向墙壁2m的位置
                smach.StateMachine.add('MOVE_TO_2M', PositionLoopState(self.node, 2.0, axis=0), 
                                     transitions={'succeeded': 'WALL_ALIGNMENT_FRONT_2M', 'in_progress': 'MOVE_TO_2M'})
                
                # 第二步：在2m位置进行墙壁角度垂直对齐（确保机器人垂直前方墙壁）
                smach.StateMachine.add('WALL_ALIGNMENT_FRONT_2M', WallAlignmentState(self.node, target_angle=0.0, tolerance=0.05), 
                                     transitions={'succeeded': 'FORWARD_TO_110CM', 'failed': 'TASK_COMPLETED'})
                
                # 第三步：前向移动到距离110cm
                smach.StateMachine.add('FORWARD_TO_110CM', PositionLoopState(self.node, self.front_distance, axis=0), 
                                     transitions={'succeeded': 'RIGHT_ALIGN_50CM', 'in_progress': 'FORWARD_TO_110CM'})
                
                # 第四步：向右对齐至50cm
                smach.StateMachine.add('RIGHT_ALIGN_50CM', PositionLoopState(self.node, 0.50, axis=2), 
                                     transitions={'succeeded': 'WALL_ALIGNMENT_RIGHT_50CM', 'in_progress': 'RIGHT_ALIGN_50CM'})
                
                # 第五步：在50cm位置进行右侧墙壁角度垂直对齐
                smach.StateMachine.add('WALL_ALIGNMENT_RIGHT_50CM', WallAlignmentState(self.node, target_angle=-np.pi/2, tolerance=0.05), 
                                     transitions={'succeeded': 'RIGHT_ALIGN_30CM', 'failed': 'TASK_COMPLETED'})
                
                # 第六步：向右移动对齐至30cm，到达目标点1
                smach.StateMachine.add('RIGHT_ALIGN_30CM', PositionLoopState(self.node, 0.30, axis=2), 
                                     transitions={'succeeded': 'WAIT_AT_RIGHT', 'in_progress': 'RIGHT_ALIGN_30CM'})
                
                # 第七步：在右侧停留10秒
                smach.StateMachine.add('WAIT_AT_RIGHT', WaitState(10), 
                                     transitions={'succeeded': 'RIGHT_ALIGN_50CM_AGAIN'})
                
                # 第八步：向右对齐至50cm
                smach.StateMachine.add('RIGHT_ALIGN_50CM_AGAIN', PositionLoopState(self.node, 0.50, axis=2), 
                                     transitions={'succeeded': 'FORWARD_TO_2M_AGAIN', 'in_progress': 'RIGHT_ALIGN_50CM_AGAIN'})
                
                # 第九步：向前对齐至2m
                smach.StateMachine.add('FORWARD_TO_2M_AGAIN', PositionLoopState(self.node, 2.0, axis=0), 
                                     transitions={'succeeded': 'LEFT_ALIGN_50CM', 'in_progress': 'FORWARD_TO_2M_AGAIN'})
                
                # 第十步：向左移动对齐至50cm
                smach.StateMachine.add('LEFT_ALIGN_50CM', PositionLoopState(self.node, 0.50, axis=1), 
                                     transitions={'succeeded': 'WALL_ALIGNMENT_FRONT_2M_AGAIN', 'in_progress': 'LEFT_ALIGN_50CM'})
                
                # 第十一步：在2m位置进行前向墙壁角度垂直对齐
                smach.StateMachine.add('WALL_ALIGNMENT_FRONT_2M_AGAIN', WallAlignmentState(self.node, target_angle=0.0, tolerance=0.05), 
                                     transitions={'succeeded': 'FORWARD_TO_110CM_AGAIN', 'failed': 'TASK_COMPLETED'})
                
                # 第十二步：向前距离至110cm
                smach.StateMachine.add('FORWARD_TO_110CM_AGAIN', PositionLoopState(self.node, self.front_distance, axis=0), 
                                     transitions={'succeeded': 'WALL_ALIGNMENT_LEFT_110CM', 'in_progress': 'FORWARD_TO_110CM_AGAIN'})
                
                # 第十三步：在110cm位置进行左侧墙壁角度垂直对齐
                smach.StateMachine.add('WALL_ALIGNMENT_LEFT_110CM', WallAlignmentState(self.node, target_angle=np.pi/2, tolerance=0.05), 
                                     transitions={'succeeded': 'LEFT_ALIGN_30CM', 'failed': 'TASK_COMPLETED'})
                
                # 第十四步：向左墙壁对齐至30cm，到达任务点2
                smach.StateMachine.add('LEFT_ALIGN_30CM', PositionLoopState(self.node, 0.30, axis=1), 
                                     transitions={'succeeded': 'TASK_COMPLETED', 'in_progress': 'LEFT_ALIGN_30CM'})

    def run(self):
        rospy.loginfo("Starting test task...")
        
        # 配置状态机
        self.config()
        
        # 创建并启动 introspection 服务来可视化状态机
        self.sis = smach_ros.IntrospectionServer('test_task_server', self.sm, '/SM_ROOT')
        self.sis.start()
        
        # 执行状态机
        rospy.loginfo("Executing state machine...")
        outcome = self.sm.execute()
        
        # 关闭 introspection 服务
        self.sis.stop()
        
        rospy.loginfo(f"Task completed with outcome: {outcome}")

def main():
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='Test task for medical robot navigation')
    parser.add_argument('task_type', type=int, choices=[1, 2], 
                       help='Task type: 1 for Left first then Right, 2 for Right first then Left')
    
    # 允许未知参数，这样ROS的系统参数就不会导致错误
    args, unknown = parser.parse_known_args()
    
    # 创建并运行任务，PIDTrackingNode会自己处理节点初始化
    task = TestTask(args.task_type)
    task.run()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
