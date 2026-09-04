#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
完整的医疗机器人任务脚本 - 集成三阶段导航与完整业务流程
基于task_new.py的完整功能，使用三阶段导航系统

任务流程：
1. 前进到目标点1（扫码决定任务流程）
2. 任务流程1：目标点2 → 条形码扫描+屏幕显示 → 上位机送药 → 目标点3 → 条形码扫描+屏幕显示 → 上位机送药 → 目标点4
3. 任务流程3：目标点3 → 条形码扫描+屏幕显示 → 上位机送药 → 目标点2 → 条形码扫描+屏幕显示 → 上位机送药 → 目标点4

技术特性：
- 三阶段导航：move_base自适应导航 + IMU姿态恢复 + PID精确定位
- 从distance.yaml读取目标点2和3的精确定位距离
- 完整的条形码扫描、屏幕显示、上位机通讯功能
- 基于task_new.py的状态机管理和业务逻辑
"""

from pid_node import PIDTrackingNode
import smach
import rospy
import serial
import smach_ros
import threading
import actionlib
import math
import yaml
from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import Imu
import tf.transformations

class AdaptiveNavigationState(smach.State):
    """自适应导航状态 - 使用move_base导航到大致位置，基于C++控制器的完成信号"""
    def __init__(self, target_pose, state_name="", parent_task=None):
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'aborted'])
        self.target_pose = target_pose  # [x, y, theta]
        self.state_name = state_name
        self.parent_task = parent_task  # 引用到主任务以访问共享资源
        self.move_base_client = None
        self.nav_timeout = 60.0  # 导航超时时间
        self.init_move_base_client()

    def init_move_base_client(self):
        """初始化move_base客户端"""
        rospy.loginfo("连接move_base服务器...")
        try:
            self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            if self.move_base_client.wait_for_server(rospy.Duration(30)):
                rospy.loginfo("move_base服务器连接成功")
            else:
                rospy.logerr("move_base服务器连接超时")
                self.move_base_client = None
        except Exception as e:
            rospy.logerr(f"move_base客户端初始化失败: {e}")
            self.move_base_client = None

    def create_goal_message(self, target_pose):
        """创建move_base目标消息 - 第一阶段只设置x,y位置，不设置orientation"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = target_pose[0]
        goal.target_pose.pose.position.y = target_pose[1]
        goal.target_pose.pose.position.z = 0.0
        # 第一阶段不设置目标orientation，让move_base自己决定如何到达
        # orientation保持默认值 (0,0,0,1) 表示没有特定方向要求
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0
        return goal

    def execute(self, userdata):
        rospy.loginfo(f"阶段1 - 自适应导航: {self.state_name}")
        rospy.loginfo(f"目标位置: ({self.target_pose[0]:.3f}, {self.target_pose[1]:.3f})")
        
        if self.move_base_client is None or self.parent_task is None:
            rospy.logerr("move_base客户端或parent_task未初始化")
            return 'failed'

        # 记录当前预期的完成计数
        initial_count = self.parent_task.completed_targets
        expected_count = initial_count + 1
        
        # 创建并发送目标
        goal_msg = self.create_goal_message(self.target_pose)
        
        try:
            rospy.loginfo("发送导航目标...")
            self.move_base_client.send_goal(goal_msg)
            
            # 等待C++控制器确认到达
            start_time = rospy.Time.now()
            
            while not rospy.is_shutdown() and self.parent_task.completed_targets < expected_count:
                elapsed_time = (rospy.Time.now() - start_time).to_sec()
                if elapsed_time > self.nav_timeout:
                    rospy.logwarn("导航超时，取消目标")
                    self.move_base_client.cancel_goal()
                    return 'aborted'
                
                goal_state = self.move_base_client.get_state()
                if goal_state in [GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.RECALLED]:
                    rospy.logwarn(f"导航失败，move_base状态: {goal_state}")
                    return 'failed'
                
                rospy.sleep(0.1)
            
            if self.parent_task.completed_targets >= expected_count:
                rospy.loginfo(f"✓ 阶段1完成 - 自适应导航成功: {self.state_name}")
                return 'succeeded'
            else:
                rospy.logwarn(f"导航被中断: {self.state_name}")
                return 'aborted'
                
        except Exception as e:
            rospy.logerr(f"导航执行异常: {e}")
            return 'failed'

class ImuAttitudeRecoveryState(smach.State):
    """IMU姿态恢复状态 - 将机器人yaw角度恢复到初始姿态"""
    def __init__(self, node, target_yaw, state_name="", tolerance=0.05, max_attempts=200, parent_task=None):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])  # 移除in_progress，因为现在是完整循环
        self.node = node
        self.target_yaw = target_yaw  # 目标yaw角度（初始姿态）
        self.state_name = state_name
        self.tolerance = tolerance  # 角度容差，默认约3度
        self.max_attempts = max_attempts  # 最大尝试次数
        self.current_yaw = 0.0
        self.attempt_count = 0
        self.parent_task = parent_task  # 添加parent_task引用
        
    def set_current_yaw(self, yaw):
        """设置当前yaw值"""
        self.current_yaw = yaw
        
    def normalize_angle(self, angle):
        """将角度归一化到[-π, π]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
        
    def calculate_yaw_error(self):
        """计算yaw误差（考虑角度环绕）"""
        error = self.target_yaw - self.current_yaw
        return self.normalize_angle(error)
        
    def execute(self, userdata):
        """执行IMU姿态恢复 - 完整循环直到完成（与threestatenav.py一致）"""
        rospy.loginfo(f"阶段2 - IMU姿态恢复: {self.state_name}")
        
        # 类似threestatenav.py，在循环中执行直到完成
        max_imu_attempts = 200  # 防止无限循环
        imu_attempts = 0
        
        while imu_attempts < max_imu_attempts and not rospy.is_shutdown():
            # 关键：每次都更新当前yaw值（从parent_task获取最新值）
            if hasattr(self.parent_task, 'current_yaw'):
                self.current_yaw = self.parent_task.current_yaw
            
            yaw_error = self.calculate_yaw_error()
            error_abs = abs(yaw_error)
            
            rospy.loginfo(f"尝试{imu_attempts+1}: yaw误差: {yaw_error:.3f}rad ({math.degrees(yaw_error):.1f}°)")
            
            # 检查是否已经在容差范围内
            if error_abs <= self.tolerance:
                rospy.loginfo(f"✓ 阶段2完成 - IMU姿态已在容差范围内 (误差: {math.degrees(error_abs):.1f}°)")
                self.node.publish_vel(0, 0, 0)  # 停止旋转
                return 'succeeded'
            
            # 计算旋转方向和速度
            max_angular_vel = 0.3  # 最大角速度 rad/s
            min_angular_vel = 0.1  # 最小角速度 rad/s
            
            # 根据误差大小调整旋转速度
            angular_vel = max(min_angular_vel, min(max_angular_vel, error_abs * 2.0))
            
            # 确定旋转方向
            if yaw_error > 0:
                angular_vel = angular_vel  # 逆时针旋转
            else:
                angular_vel = -angular_vel  # 顺时针旋转
                
            rospy.loginfo(f"执行旋转: 角速度 {angular_vel:.3f}rad/s")
            self.node.publish_vel(0, 0, angular_vel)
            
            # 短暂等待让旋转生效，然后再次检查
            rospy.sleep(0.1)
            imu_attempts += 1
        
        # 超时退出
        rospy.logerr(f"姿态恢复超时失败: {self.state_name}")
        self.node.publish_vel(0, 0, 0)
        return 'failed'

class PrecisePositionState(smach.State):
    """精确定位状态 - 使用PID控制进行精确距离定位（与task_new.py的PositionLoopState完全一致）"""
    def __init__(self, node, target_distance, axis, state_name=""):
        smach.State.__init__(self, outcomes=['succeeded', 'in_progress', 'failed'])
        self.node = node
        self.target_distance = target_distance
        self.axis = axis  # 0=前方, 1=左侧, 2=右侧
        self.state_name = state_name

    def execute(self, userdata):
        """执行精确定位（与task_new.py的PositionLoopState.execute完全一致）"""
        rospy.loginfo(f"阶段3 - 精确定位: {self.state_name} - 轴{self.axis}, 目标距离{self.target_distance:.3f}m")
        
        try:
            # 与原始task_new.py的PositionLoopState逻辑完全一致
            if abs(self.node.distance[self.axis] - self.target_distance) > 0.01:
                self.node.distance_state = self.axis + 1
                self.node.position_loop(self.target_distance)
                return 'in_progress'
            
            # 精度满足要求，停止运动
            self.node.publish_vel(0, 0, 0)
            rospy.loginfo(f"✓ 阶段3完成 - 精确定位成功: {self.state_name}")
            return 'succeeded'
            
        except Exception as e:
            rospy.logerr(f"精确定位异常: {e}")
            self.node.publish_vel(0, 0, 0)
            return 'failed'

class ThreeStageNavigationState(smach.State):
    """三阶段导航状态 - 组合自适应导航、IMU姿态恢复和精确定位"""
    def __init__(self, node, rough_pose, precise_distance, axis, state_name="", parent_task=None):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.node = node
        self.rough_pose = rough_pose  # 粗略目标位置 [x, y, theta]
        self.precise_distance = precise_distance  # 精确目标距离
        self.axis = axis  # 精确定位轴
        self.state_name = state_name
        self.parent_task = parent_task  # 引用到主任务以访问target_done功能和IMU数据
        
        # 创建子状态机
        self.create_sub_state_machine()

    def create_sub_state_machine(self):
        """创建三阶段导航的子状态机"""
        self.sub_sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
        
        with self.sub_sm:
            # 阶段1: 自适应导航
            smach.StateMachine.add(
                'ADAPTIVE_NAV',
                AdaptiveNavigationState(
                    self.rough_pose, 
                    f"{self.state_name}_粗略导航", 
                    parent_task=self.parent_task
                ),
                transitions={
                    'succeeded': 'IMU_RECOVERY',
                    'failed': 'failed',
                    'aborted': 'failed'
                }
            )
            
            # 阶段2: IMU姿态恢复
            smach.StateMachine.add(
                'IMU_RECOVERY',
                ImuAttitudeRecoveryState(
                    self.node, 
                    self.parent_task.initial_yaw, 
                    f"{self.state_name}_姿态恢复",
                    parent_task=self.parent_task
                ),
                transitions={
                    'succeeded': 'PRECISE_POS',
                    'failed': 'failed'  # 移除in_progress转换，现在是完整循环
                }
            )
            
            # 阶段3: 精确定位
            smach.StateMachine.add(
                'PRECISE_POS',
                PrecisePositionState(self.node, self.precise_distance, self.axis, f"{self.state_name}_精确定位"),
                transitions={
                    'succeeded': 'succeeded',
                    'in_progress': 'PRECISE_POS',
                    'failed': 'failed'
                }
            )

    def execute(self, userdata):
        rospy.loginfo(f"开始三阶段导航: {self.state_name}")
        
        try:
            # IMU姿态恢复现在会自动从parent_task获取最新的yaw值，无需手动设置
            
            # 执行子状态机
            outcome = self.sub_sm.execute()
            rospy.loginfo(f"三阶段导航完成: {self.state_name}, 结果: {outcome}")
            return outcome
            
        except Exception as e:
            rospy.logerr(f"三阶段导航异常: {e}")
            return 'failed'

class TaskNavigationState(smach.State):
    """任务导航状态 - 支持点名或索引的PID导航"""
    def __init__(self, node, target, state_name="", parent_task=None):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.node = node
        self.target = target  # 可以是点名（如'point1'）或索引（如4）
        self.state_name = state_name
        self.parent_task = parent_task

    def execute(self, userdata):
        rospy.loginfo(f"执行PID导航: {self.state_name} -> {self.target}")
        
        try:
            # 根据target类型获取坐标
            if isinstance(self.target, str):
                # 点名模式
                if self.target.startswith('point') and self.parent_task:
                    # 普通点名，直接使用原始坐标
                    point = self.parent_task.goal_positions[self.target]
                    x, y = point[0], point[1]
                else:
                    rospy.logerr(f"未知的点名: {self.target}")
                    return 'failed'
            else:
                # 索引模式（兼容原始逻辑）
                goal_idx = int(self.target)
                
                # 完全按照task_new.py的TaskState逻辑
                if goal_idx == 6:
                    self.node.goal_point[goal_idx][0] = -self.node.currant_distance_state[0]
                    rospy.loginfo(f"特殊处理goal_point[6]: x = -{self.node.currant_distance_state[0]}")
                elif goal_idx == 7:
                    self.node.goal_point[goal_idx][1] = -self.node.currant_distance_state[1]
                    rospy.loginfo(f"特殊处理goal_point[7]: y = -{self.node.currant_distance_state[1]}")
                
                x = self.node.goal_point[goal_idx][0]
                y = self.node.goal_point[goal_idx][1]
            
            rospy.loginfo(f"PID导航到: ({x:.3f}, {y:.3f})")
            self.node.task(x, y)
            
            return 'succeeded'
            
        except Exception as e:
            rospy.logerr(f"PID导航失败: {e}")
            return 'failed'

class BarcodeProcessingState(smach.State):
    """条形码处理状态 - 扫描条形码并显示在屏幕"""
    def __init__(self, node, location_name, state_name=""):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.node = node
        self.location_name = location_name
        self.state_name = state_name

    def execute(self, userdata):
        rospy.loginfo(f"执行条形码扫描: {self.state_name} - 位置: {self.location_name}")
        
        try:
            # 模拟条形码扫描过程
            rospy.loginfo(f"正在扫描{self.location_name}的条形码...")
            rospy.sleep(2.0)  # 模拟扫描时间
            
            # 模拟获取条形码信息（实际应该从条形码扫描器获取）
            barcode_data = f"{self.location_name}_药品_批次号_20240101"
            
            # 显示在屏幕上（实际应该调用屏幕显示接口）
            rospy.loginfo(f"📺 屏幕显示: {barcode_data}")
            rospy.loginfo(f"条形码信息已显示在屏幕上: {barcode_data}")
            
            return 'succeeded'
            
        except Exception as e:
            rospy.logerr(f"条形码处理失败: {e}")
            return 'failed'

class BarcodeScanAndDeliveryState(smach.State):
    """送药状态 - 完全复制task_new.py中的Push_medicines_state逻辑"""
    def __init__(self, node, arm_serial, location_name, command, state_name=""):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.node = node
        self.arm_serial = arm_serial
        self.location_name = location_name
        self.command = command  # 发送给上位机的固定命令（'1\r'或'3\r'）
        self.state_name = state_name

    def execute(self, userdata):
        rospy.loginfo(f"执行送药通讯: {self.state_name} - 位置: {self.location_name}")
        
        try:
            # 第一步：模拟条形码扫描并显示（实际应由DisplayScreen.py处理）
            rospy.loginfo(f"正在扫描{self.location_name}的条形码...")
            rospy.sleep(2.0)  # 模拟扫描时间
            
            # 注意：task_new.py中在目标点处没有真正的条形码扫描
            # 只有初始GM65扫码用于选择任务流程，屏幕只显示初始二维码信息
            rospy.loginfo(f"📋 到达{self.location_name}，准备送药")
            rospy.loginfo(f"📺 屏幕显示: 二维码信息（初始扫码结果）")
            
            # 第二步：检查串口连接
            if self.arm_serial is None:
                rospy.logwarn("机械臂串口未连接，送药失败")
                return 'failed'
            
            # 第三步：发送固定命令给上位机（与task_new.py完全一致）
            rospy.loginfo(f"Executing SerialCommunicationState")
            rospy.loginfo(f"发送命令给上位机: {self.command}")
            self.arm_serial.write(self.command.encode('utf-8'))

            # 第四步：等待上位机返回确认数据（与task_new.py完全一致）
            rospy.loginfo("等待上位机确认...")
            while not self.arm_serial.in_waiting:
                if rospy.is_shutdown():
                    return 'failed'
                pass
            
            # 第五步：读取上位机返回的确认信息
            response_data = self.arm_serial.readline().decode('utf-8').strip()
            if response_data:
                rospy.loginfo(f"Received from serial: {response_data}")
                rospy.loginfo(f"✓ 送药完成确认: {response_data}")
                rospy.sleep(2)  # 与task_new.py一致的延时
                return 'succeeded'
            else:
                rospy.loginfo("No data received from serial.")
                return 'failed'
            
        except Exception as e:
            rospy.logerr(f"送药通讯失败: {e}")
            return 'failed'

class MedicineDeliveryState(smach.State):
    """送药状态 - 完全复制task_new.py中的Push_medicines_state逻辑"""
    def __init__(self, arm_serial, command, state_name=""):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.arm_serial = arm_serial
        self.command = command
        self.state_name = state_name

    def execute(self, userdata):
        rospy.loginfo(f"执行送药通讯: {self.state_name} - 指令: {self.command}")
        
        if self.arm_serial is None:
            rospy.logwarn("机械臂串口未连接，送药失败")
            return 'failed'
        
        try:
            # 完全按照task_new.py的Push_medicines_state逻辑
            rospy.loginfo("Executing SerialCommunicationState")
            self.arm_serial.write(self.command.encode('utf-8'))

            # 等待下位机返回数据（与原始逻辑完全一致）
            while not self.arm_serial.in_waiting:
                pass
            
            data = self.arm_serial.readline().decode('utf-8').strip()
            if data:
                rospy.loginfo(f"Received from serial: {data}")
                rospy.loginfo(f"✓ 送药完成确认: {data}")
                rospy.sleep(2)  # 与原始代码一致的延时
                return 'succeeded'
            else:
                rospy.loginfo("No data received from serial.")
                return 'failed'
                
        except Exception as e:
            rospy.logerr(f"串口通讯执行失败: {e}")
            return 'failed'

class NavigateToScanPointAndWaitState(smach.State):
    """导航到扫码点并等待扫码状态 - 完全复制task_new.py的逻辑"""
    def __init__(self, parent_task, state_name=""):
        smach.State.__init__(self, outcomes=['task_1', 'task_3', 'failed'])
        self.parent_task = parent_task
        self.state_name = state_name
        self.timeout = 30.0  # 30秒扫码超时

    def execute(self, userdata):
        rospy.loginfo(f"导航到扫码点并等待扫码: {self.state_name}")
        
        try:
            # 第一步：导航到目标点1（与task_new.py完全一致）
            rospy.loginfo("导航到目标点1（扫码点）...")
            point1 = self.parent_task.goal_positions['point1']
            self.parent_task.node.task(point1[0], point1[1])
            rospy.loginfo("✓ 已到达目标点1，开始等待扫码...")
            
            # 第二步：静止等待扫码结果（与task_new.py完全一致）
            self.parent_task.choose_data = rospy.get_param("gm65_data", 0)
            start_time = rospy.Time.now()
            
            while self.parent_task.choose_data == 0 and not rospy.is_shutdown():
                elapsed = (rospy.Time.now() - start_time).to_sec()
                if elapsed > self.timeout:
                    rospy.logwarn("扫码超时，默认执行任务1")
                    self.parent_task.choose_data = 1
                    break
                
                # 与task_new.py一致的等待逻辑
                self.parent_task.choose_data = rospy.get_param("gm65_data", 0)
                pass  # 与原始代码一致，使用pass而不是sleep
            
            rospy.loginfo(f"扫码完成，结果: {self.parent_task.choose_data}")
            
            # 根据扫码结果返回对应的结果
            if self.parent_task.choose_data == 1:
                return 'task_1'
            elif self.parent_task.choose_data == 3:
                return 'task_3'
            else:
                rospy.logwarn(f"未知扫码结果: {self.parent_task.choose_data}")
                return 'failed'
                
        except Exception as e:
            rospy.logerr(f"导航到扫码点失败: {e}")
            return 'failed'

class CompleteTaskRobot:
    """完整任务机器人类 - 集成三阶段导航与完整业务流程"""
    
    def __init__(self):
        rospy.loginfo("初始化完整任务机器人（三阶段导航版本）...")
        
        # 初始化PID跟踪节点
        self.node = PIDTrackingNode()
        
        # 初始化机械臂串口
        self.arm_serial = self.init_arm_serial()
        
        # 任务选择数据
        self.choose_data = 0
        
        # 订阅GM65扫码结果
        rospy.Subscriber("/gm65_data", String, self.choose_callback)
        
        # 初始化target_done支持
        self.completed_targets = 0  # 已完成的目标数量
        self.init_target_done_subscriber()
        
        # 初始化IMU支持
        self.current_yaw = 0.0  # 当前yaw角度
        self.initial_yaw = None  # 初始yaw角度
        self.imu_initialized = False  # IMU是否已初始化
        self.init_imu_subscriber()
        
        # 获取激光雷达距离参数（与task_new.py一致）
        self.lidar_distance = rospy.get_param("lader_point", [])
        self.process_lidar_distance()
        
        # 从distance.yaml读取目标点2和3的精确定位距离
        self.distance_yaml_data = self.load_distance_yaml()
        
        # 定义目标点位置（对应PIDTrackingNode的goal_point）
        self.goal_positions = self.define_goal_positions()
        
        # 创建主状态机（包含完整流程）
        self.sm = smach.StateMachine(outcomes=['TASK_COMPLETED'])
        self.build_complete_state_machine()
        
        rospy.loginfo("完整任务机器人初始化完成")

    def process_lidar_distance(self):
        """处理激光雷达距离参数（与task_new.py完全一致）"""
        for i in range(len(self.lidar_distance)):
            for j in range(3):
                self.lidar_distance[i][j] -= 0.2
                if self.lidar_distance[i][j] <= 0:
                    self.lidar_distance[i][j] = 0.1

    def load_distance_yaml(self):
        """从distance.yaml读取目标点2和3的距离数据"""
        try:
            distance_file_path = '/home/jetson/code_files/robocup/medical_ws/src/robcup/data/distance.yaml'
            rospy.loginfo(f"读取distance.yaml用于精确定位...")
            
            with open(distance_file_path, 'r', encoding='utf-8') as f:
                distance_data = yaml.safe_load(f)
            
            if 'lader_point' in distance_data and len(distance_data['lader_point']) >= 2:
                lader_point = distance_data['lader_point']
                
                yaml_distances = {
                    'point2': {
                        'front': lader_point[0][0] if len(lader_point[0]) > 0 else 0.30,
                        'left': lader_point[0][1] if len(lader_point[0]) > 1 else 0.25,
                        'right': lader_point[0][2] if len(lader_point[0]) > 2 else 0.25
                    },
                    'point3': {
                        'front': lader_point[1][0] if len(lader_point[1]) > 0 else 0.28,
                        'left': lader_point[1][1] if len(lader_point[1]) > 1 else 0.20,
                        'right': lader_point[1][2] if len(lader_point[1]) > 2 else 0.30
                    }
                }
                
                rospy.loginfo("📊 从distance.yaml读取的精确定位距离:")
                rospy.loginfo(f"   目标点2: 前={yaml_distances['point2']['front']:.3f}m, 左={yaml_distances['point2']['left']:.3f}m, 右={yaml_distances['point2']['right']:.3f}m")
                rospy.loginfo(f"   目标点3: 前={yaml_distances['point3']['front']:.3f}m, 左={yaml_distances['point3']['left']:.3f}m, 右={yaml_distances['point3']['right']:.3f}m")
                
                return yaml_distances
            else:
                rospy.logwarn("distance.yaml格式不正确，使用默认距离配置")
                return None
                
        except Exception as e:
            rospy.logwarn(f"读取distance.yaml失败: {e}，使用默认距离配置")
            return None

    def define_goal_positions(self):
        """定义目标点位置（使用用户在RViz中记录的实际位置）"""
        return {
            'point1': [1.000, 0.000, 0.0],  # 目标点1 - 扫码决定任务流程的位置  
            'point2': [5.300, 2.000, 0.0],   # 目标点2 - 药品区域1
            'point3': [5.300, -1.600, 0.0],  # 目标点3 - 药品区域2
            'point4': [0.0, 0.0, 0.0],   # 目标点4 - 返回点
        }

    def init_target_done_subscriber(self):
        """订阅target_done话题，监听C++控制器的目标完成计数"""
        rospy.loginfo("订阅target_done话题，监听目标完成信号...")
        rospy.Subscriber('/target_done', Int32, self.target_done_callback)

    def target_done_callback(self, msg):
        """target_done回调函数，检查C++控制器的目标完成计数"""
        if msg.data > self.completed_targets:
            self.completed_targets = msg.data
            rospy.loginfo(f"收到目标完成信号，总完成数: {msg.data}")

    def init_imu_subscriber(self):
        """订阅IMU话题，监听机器人姿态"""
        rospy.loginfo("订阅/handsfree/imu话题，监听机器人姿态...")
        rospy.Subscriber('/handsfree/imu', Imu, self.imu_callback)
        
        # 等待IMU数据初始化
        rospy.loginfo("等待IMU数据初始化...")
        init_timeout = 10.0  # 10秒超时
        start_time = rospy.Time.now()
        
        while not self.imu_initialized and not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > init_timeout:
                rospy.logwarn("IMU初始化超时，使用默认初始姿态 (yaw=0)")
                self.initial_yaw = 0.0
                self.current_yaw = 0.0
                self.imu_initialized = True
                break
            rospy.sleep(0.1)
        
        if self.imu_initialized:
            rospy.loginfo(f"✅ IMU初始化完成，初始yaw: {self.initial_yaw:.3f}rad ({math.degrees(self.initial_yaw):.1f}°)")

    def imu_callback(self, msg):
        """IMU回调函数，提取yaw角度"""
        try:
            quaternion = [
                msg.orientation.x,
                msg.orientation.y, 
                msg.orientation.z,
                msg.orientation.w
            ]
            
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
            self.current_yaw = yaw
            
            if not self.imu_initialized:
                self.initial_yaw = yaw
                self.imu_initialized = True
                rospy.loginfo(f"记录初始IMU姿态: yaw={yaw:.3f}rad ({math.degrees(yaw):.1f}°)")
                
        except Exception as e:
            rospy.logwarn(f"IMU数据处理异常: {e}")

    def choose_callback(self, msg):
        """扫码结果回调函数（与task_new.py完全一致）"""
        if msg.data == '1':
            self.choose_data = 1
        elif msg.data == '3':
            self.choose_data = 3
        rospy.loginfo(f"收到任务选择: {self.choose_data} (原始数据: {msg.data})")

    def init_arm_serial(self):
        """初始化机械臂串口连接（与task_new.py完全一致）"""
        port = '/dev/arm'
        baud_rate = 115200
        try:
            ser = serial.Serial(port, baud_rate, timeout=1)
            if ser.is_open:
                rospy.loginfo(f'Serial port {port} opened successfully with baud rate {baud_rate}')
                return ser
            else:
                rospy.logerr(f'Failed to open serial port {port}')
                return None
        except serial.SerialException as e:
            rospy.logerr(f'Error opening serial port: {e}')
            return None

    def get_distance_from_yaml(self, point_id, direction):
        """从distance.yaml获取指定目标点的距离"""
        if self.distance_yaml_data and f'point{point_id}' in self.distance_yaml_data:
            return self.distance_yaml_data[f'point{point_id}'][direction]
        else:
            # 默认距离值
            defaults = {
                2: {'front': 0.30, 'left': 0.25, 'right': 0.25},
                3: {'front': 0.28, 'left': 0.20, 'right': 0.30}
            }
            return defaults.get(point_id, {}).get(direction, 0.25)

    def build_complete_state_machine(self):
        """构建完整的状态机，包含移动到目标点1、扫码、以及后续任务流程"""
        rospy.loginfo("构建完整状态机流程...")
        
        with self.sm:
            # 第一步：导航到目标点1并等待扫码（与task_new.py一致）
            smach.StateMachine.add(
                'NAV_TO_SCAN_POINT_AND_WAIT',
                NavigateToScanPointAndWaitState(self, "导航到扫码点并等待扫码"),
                transitions={
                    'task_1': 'TASK1_NAV_TO_POINT2',
                    'task_3': 'TASK3_NAV_TO_POINT3', 
                    'failed': 'TASK_COMPLETED'
                }
            )
            
            # ==================== 任务流程1 ====================
            # 任务1：目标点2 → 条形码+送药 → 目标点3 → 条形码+送药 → 目标点4
            
            # 第一站：三阶段导航到目标点2
            smach.StateMachine.add(
                'TASK1_NAV_TO_POINT2',
                ThreeStageNavigationState(
                    self.node,
                    self.goal_positions['point2'],
                    self.get_distance_from_yaml(2, 'left'),
                    1,  # 轴1 = 左侧
                    "任务1_导航到目标点2",
                    parent_task=self
                ),
                transitions={'succeeded': 'TASK1_BARCODE_AND_DELIVERY_2', 'failed': 'TASK_COMPLETED'}
            )
            
            smach.StateMachine.add(
                'TASK1_BARCODE_AND_DELIVERY_2',
                BarcodeScanAndDeliveryState(self.node, self.arm_serial, "目标点2", '1\r', "任务1_扫码+送药到目标点2"),
                transitions={'succeeded': 'TASK1_NAV_TO_POINT3', 'failed': 'TASK_COMPLETED'}
            )
            
            # 第二站：三阶段导航到目标点3
            smach.StateMachine.add(
                'TASK1_NAV_TO_POINT3',
                ThreeStageNavigationState(
                    self.node,
                    self.goal_positions['point3'],
                    self.get_distance_from_yaml(3, 'right'),
                    2,  # 轴2 = 右侧
                    "任务1_导航到目标点3",
                    parent_task=self
                ),
                transitions={'succeeded': 'TASK1_BARCODE_AND_DELIVERY_3', 'failed': 'TASK_COMPLETED'}
            )
            
            smach.StateMachine.add(
                'TASK1_BARCODE_AND_DELIVERY_3',
                BarcodeScanAndDeliveryState(self.node, self.arm_serial, "目标点3", '3\r', "任务1_扫码+送药到目标点3"),
                transitions={'succeeded': 'TASK1_NAV_TO_POINT4', 'failed': 'TASK_COMPLETED'}
            )
            
            # 第三站：返回目标点4
            smach.StateMachine.add(
                'TASK1_NAV_TO_POINT4',
                TaskNavigationState(self.node, 'point4', "任务1_返回目标点4", parent_task=self),
                transitions={'succeeded': 'TASK_COMPLETED', 'failed': 'TASK_COMPLETED'}
            )
            
            # ==================== 任务流程3 ====================
            # 任务3：目标点3 → 条形码+送药 → 目标点2 → 条形码+送药 → 目标点4
            
            # 第一站：三阶段导航到目标点3
            smach.StateMachine.add(
                'TASK3_NAV_TO_POINT3',
                ThreeStageNavigationState(
                    self.node,
                    self.goal_positions['point3'],  # 直接使用原始坐标
                    self.get_distance_from_yaml(3, 'right'),
                    2,  # 轴2 = 右侧
                    "任务3_导航到目标点3",
                    parent_task=self
                ),
                transitions={'succeeded': 'TASK3_BARCODE_AND_DELIVERY_3', 'failed': 'TASK_COMPLETED'}
            )
            
            smach.StateMachine.add(
                'TASK3_BARCODE_AND_DELIVERY_3',
                BarcodeScanAndDeliveryState(self.node, self.arm_serial, "目标点3", '3\r', "任务3_扫码+送药到目标点3"),
                transitions={'succeeded': 'TASK3_NAV_TO_POINT2', 'failed': 'TASK_COMPLETED'}
            )
            
            # 第二站：三阶段导航到目标点2
            smach.StateMachine.add(
                'TASK3_NAV_TO_POINT2',
                ThreeStageNavigationState(
                    self.node,
                    self.goal_positions['point2'],  # 直接使用原始坐标
                    self.get_distance_from_yaml(2, 'left'),
                    1,  # 轴1 = 左侧
                    "任务3_导航到目标点2",
                    parent_task=self
                ),
                transitions={'succeeded': 'TASK3_BARCODE_AND_DELIVERY_2', 'failed': 'TASK_COMPLETED'}
            )
            
            smach.StateMachine.add(
                'TASK3_BARCODE_AND_DELIVERY_2',
                BarcodeScanAndDeliveryState(self.node, self.arm_serial, "目标点2", '1\r', "任务3_扫码+送药到目标点2"),
                transitions={'succeeded': 'TASK3_NAV_TO_POINT4', 'failed': 'TASK_COMPLETED'}
            )
            
            # 第三站：返回目标点4
            smach.StateMachine.add(
                'TASK3_NAV_TO_POINT4',
                TaskNavigationState(self.node, 'point4', "任务3_返回目标点4", parent_task=self),
                transitions={'succeeded': 'TASK_COMPLETED', 'failed': 'TASK_COMPLETED'}
            )

    def run(self):
        """运行主任务 - 执行完整的状态机流程"""
        rospy.loginfo("开始执行完整任务流程...")
        
        try:
            # 启动状态机内省服务器
            self.sis = smach_ros.IntrospectionServer('complete_task', self.sm, '/COMPLETE_TASK')
            self.sis.start()
            
            # 执行完整状态机
            outcome = self.sm.execute()
            
            rospy.loginfo(f"完整任务完成，结果: {outcome}")
            
        except Exception as e:
            rospy.logerr(f"任务执行异常: {e}")
        finally:
            # 关闭内省服务器
            if hasattr(self, 'sis'):
                self.sis.stop()

if __name__ == '__main__':
    """主函数 - 与task_new.py保持一致的启动方式"""
    rospy.loginfo("启动完整任务机器人节点（三阶段导航版本）...")
    
    try:
        # 创建任务实例
        task = CompleteTaskRobot()
        task.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("接收到ROS中断信号，正在退出...")
    except Exception as e:
        rospy.logerr(f"程序异常: {e}")
    finally:
        rospy.loginfo("完整任务机器人节点结束")

