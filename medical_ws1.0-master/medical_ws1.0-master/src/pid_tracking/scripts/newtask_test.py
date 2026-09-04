#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
新任务脚本 - 集成三阶段导航策略
阶段1：使用move_base进行自适应导航到大致位置
阶段2：使用IMU进行姿态恢复到初始yaw角度  
阶段3：使用PID跟踪进行精确距离定位

特性:
- 完整的三阶段导航流程（粗导航+姿态恢复+精确定位）
- 从distance.yaml动态读取目标点2和3的精确定位距离
- 基于C++控制器的可靠完成检测（阶段1）
- 基于IMU数据的姿态误差检测（阶段2）
- 基于雷达距离的精确定位检测（阶段3）
"""

from pid_node import PIDTrackingNode
import smach
import rospy
import serial
import smach_ros
import threading
import actionlib
import math
from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import Imu
import tf.transformations
import yaml

class AdaptiveNavigationState(smach.State):
    """自适应导航状态 - 使用move_base导航到大致位置，基于C++控制器的完成信号"""
    def __init__(self, target_pose, state_name="", parent_task=None):
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'aborted'])
        self.target_pose = target_pose  # [x, y, theta]
        self.state_name = state_name
        self.parent_task = parent_task  # 引用到NewRaceTask以访问共享资源
        self.move_base_client = None
        self.nav_timeout = 60.0  # 导航超时时间
        self.init_move_base_client()

    def init_move_base_client(self):
        """改进的move_base客户端初始化"""
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
        """创建move_base目标消息"""
        goal = MoveBaseGoal()
        
        # 设置目标帧
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # 设置位置
        goal.target_pose.pose.position.x = target_pose[0]
        goal.target_pose.pose.position.y = target_pose[1]
        goal.target_pose.pose.position.z = 0.0
        
        # 设置方向
        theta = target_pose[2] if len(target_pose) > 2 else 0.0
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = math.sin(theta / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(theta / 2.0)
        
        return goal

    def execute(self, userdata):
        rospy.loginfo(f"执行自适应导航: {self.state_name}")
        rospy.loginfo(f"目标位置: ({self.target_pose[0]:.3f}, {self.target_pose[1]:.3f})")
        
        if self.move_base_client is None:
            rospy.logerr("move_base客户端未初始化")
            return 'failed'

        if self.parent_task is None:
            rospy.logerr("未提供parent_task引用")
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
                # 检查超时
                elapsed_time = (rospy.Time.now() - start_time).to_sec()
                if elapsed_time > self.nav_timeout:
                    rospy.logwarn("导航超时，取消目标")
                    self.move_base_client.cancel_goal()
                    return 'aborted'
                
                # 检查move_base状态
                goal_state = self.move_base_client.get_state()
                if goal_state in [GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.RECALLED]:
                    rospy.logwarn(f"导航失败，move_base状态: {goal_state}")
                    return 'failed'
                
                # 每2秒输出一次状态
                if int(elapsed_time) % 2 == 0 and elapsed_time > 0:
                    if int(elapsed_time) != getattr(self, '_last_log_time', -1):
                        rospy.loginfo(f"导航进行中... 已用时 {elapsed_time:.0f}s，等待目标完成计数达到 {expected_count}")
                        self._last_log_time = int(elapsed_time)
                
                rospy.sleep(0.1)
            
            if self.parent_task.completed_targets >= expected_count:
                rospy.loginfo(f"自适应导航成功: {self.state_name} (完成计数: {self.parent_task.completed_targets})")
                return 'succeeded'
            else:
                rospy.logwarn(f"导航被中断: {self.state_name}")
                return 'aborted'
                
        except Exception as e:
            rospy.logerr(f"导航执行异常: {e}")
            return 'failed'

class ImuAttitudeRecoveryState(smach.State):
    """IMU姿态恢复状态 - 将机器人yaw角度恢复到初始姿态"""
    def __init__(self, node, target_yaw, state_name="", tolerance=0.05, max_attempts=200):
        smach.State.__init__(self, outcomes=['succeeded', 'in_progress', 'failed'])
        self.node = node
        self.target_yaw = target_yaw  # 目标yaw角度（初始姿态）
        self.state_name = state_name
        self.tolerance = tolerance  # 角度容差，默认约3度
        self.max_attempts = max_attempts  # 最大尝试次数
        self.current_yaw = 0.0
        self.attempt_count = 0
        
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
        """执行IMU姿态恢复"""
        rospy.loginfo(f"阶段2 - IMU姿态恢复: {self.state_name}")
        rospy.loginfo(f"目标yaw: {self.target_yaw:.3f}rad ({math.degrees(self.target_yaw):.1f}°)")
        rospy.loginfo(f"当前yaw: {self.current_yaw:.3f}rad ({math.degrees(self.current_yaw):.1f}°)")
        
        yaw_error = self.calculate_yaw_error()
        error_abs = abs(yaw_error)
        
        rospy.loginfo(f"yaw误差: {yaw_error:.3f}rad ({math.degrees(yaw_error):.1f}°)")
        
        # 检查是否已经在容差范围内
        if error_abs <= self.tolerance:
            rospy.loginfo(f"✓ IMU姿态已在容差范围内 (误差: {math.degrees(error_abs):.1f}°)")
            self.node.publish_vel(0, 0, 0)  # 停止旋转
            return 'succeeded'
        
        # 检查最大尝试次数
        self.attempt_count += 1
        if self.attempt_count >= self.max_attempts:
            rospy.logwarn(f"达到最大尝试次数，姿态恢复失败: {self.state_name}")
            self.node.publish_vel(0, 0, 0)
            return 'failed'
        
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
        
        return 'in_progress'

class PrecisePositionState(smach.State):
    """精确定位状态 - 使用PID控制进行精确距离定位"""
    def __init__(self, node, target_distance, axis, state_name="", tolerance=0.01):
        smach.State.__init__(self, outcomes=['succeeded', 'in_progress', 'failed'])
        self.node = node
        self.target_distance = target_distance
        self.axis = axis  # 0=前方, 1=左侧, 2=右侧
        self.state_name = state_name
        self.tolerance = tolerance  # 定位精度
        self.max_attempts = 100  # 最大尝试次数
        self.attempt_count = 0
        self.stable_count = 0  # 稳定计数
        self.required_stable = 5  # 需要连续稳定的次数

    def execute(self, userdata):
        rospy.loginfo(f"阶段3 - 精确定位: {self.state_name} - 轴{self.axis}, 目标距离{self.target_distance:.3f}m")
        
        try:
            # 获取当前距离
            if self.axis >= len(self.node.distance):
                rospy.logerr(f"距离传感器轴{self.axis}不存在")
                return 'failed'
                
            current_distance = self.node.distance[self.axis]
            error = abs(current_distance - self.target_distance)
            
            rospy.loginfo(f"当前距离: {current_distance:.3f}m, 目标: {self.target_distance:.3f}m, 误差: {error:.3f}m")
            
            # 检查是否达到精度要求
            if error <= self.tolerance:
                self.stable_count += 1
                rospy.loginfo(f"距离满足精度要求 ({self.stable_count}/{self.required_stable})")
                
                if self.stable_count >= self.required_stable:
                    rospy.loginfo(f"精确定位成功: {self.state_name}")
                    self.node.publish_vel(0, 0, 0)  # 停止运动
                    return 'succeeded'
                else:
                    rospy.sleep(0.1)  # 短暂等待确认稳定
                    return 'in_progress'
            else:
                self.stable_count = 0  # 重置稳定计数
            
            # 检查最大尝试次数
            self.attempt_count += 1
            if self.attempt_count >= self.max_attempts:
                rospy.logwarn(f"达到最大尝试次数，精确定位失败: {self.state_name}")
                self.node.publish_vel(0, 0, 0)
                return 'failed'
            
            # 执行PID位置控制
            self.node.distance_state = self.axis + 1
            self.node.position_loop(self.target_distance)
            rospy.sleep(0.05)  # 控制循环频率
            
            return 'in_progress'
            
        except Exception as e:
            rospy.logerr(f"精确定位异常: {e}")
            self.node.publish_vel(0, 0, 0)
            return 'failed'

class TaskNavigationState(smach.State):
    """任务导航状态 - 完全复制task_new.py中的TaskState逻辑"""
    def __init__(self, node, goal_idx, state_name=""):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.node = node
        self.goal_idx = goal_idx
        self.state_name = state_name

    def execute(self, userdata):
        rospy.loginfo(f"执行PID导航: {self.state_name} -> goal_point[{self.goal_idx}]")
        
        try:
            # 完全按照task_new.py的TaskState逻辑
            if self.goal_idx == 6:
                self.node.goal_point[self.goal_idx][0] = -self.node.currant_distance_state[0]
                rospy.loginfo(f"特殊处理goal_point[6]: x = -{self.node.currant_distance_state[0]}")
            elif self.goal_idx == 7:
                self.node.goal_point[self.goal_idx][1] = -self.node.currant_distance_state[1]
                rospy.loginfo(f"特殊处理goal_point[7]: y = -{self.node.currant_distance_state[1]}")
            
            # 执行PID导航（与原始TaskState完全一致）
            x = self.node.goal_point[self.goal_idx][0]
            y = self.node.goal_point[self.goal_idx][1]
            rospy.loginfo(f"PID导航到: ({x:.3f}, {y:.3f})")
            self.node.task(x, y)
            
            return 'succeeded'
            
        except Exception as e:
            rospy.logerr(f"PID导航失败: {e}")
            return 'failed'

class PushMedicineState(smach.State):
    """推药状态 - 完全复制task_new.py中的Push_medicines_state逻辑"""
    def __init__(self, arm_serial, command, state_name=""):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.arm_serial = arm_serial
        self.command = command
        self.state_name = state_name

    def execute(self, userdata):
        rospy.loginfo(f"执行推药通讯: {self.state_name} - 指令: {self.command}")
        
        if self.arm_serial is None:
            rospy.logwarn("机械臂串口未连接，跳过推药指令")
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
                rospy.sleep(2)  # 与原始代码一致的延时
                return 'succeeded'
            else:
                rospy.loginfo("No data received from serial.")
                return 'failed'
                
        except Exception as e:
            rospy.logerr(f"串口通讯执行失败: {e}")
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
        self.parent_task = parent_task  # 引用到NewRaceTask以访问target_done功能和IMU数据
        
        # 创建子状态机
        self.create_sub_state_machine()

    def create_sub_state_machine(self):
        """创建三阶段导航的子状态机"""
        self.sub_sm = smach.StateMachine(outcomes=['succeeded', 'failed'])
        
        with self.sub_sm:
            # 阶段1: 自适应导航（使用来自nav_goal_new.py的成熟导航逻辑）
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
                    f"{self.state_name}_姿态恢复"
                ),
                transitions={
                    'succeeded': 'PRECISE_POS',
                    'in_progress': 'IMU_RECOVERY',
                    'failed': 'failed'
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
            # 在执行前更新IMU姿态恢复状态的当前yaw值
            if hasattr(self.parent_task, 'current_yaw'):
                for state_name, state in self.sub_sm._states.items():
                    if isinstance(state, ImuAttitudeRecoveryState):
                        state.set_current_yaw(self.parent_task.current_yaw)
            
            # 执行子状态机
            outcome = self.sub_sm.execute()
            rospy.loginfo(f"三阶段导航完成: {self.state_name}, 结果: {outcome}")
            return outcome
            
        except Exception as e:
            rospy.logerr(f"三阶段导航异常: {e}")
            return 'failed'

class NewRaceTask:
    """新比赛任务类 - 集成三阶段导航的任务控制器"""
    
    def __init__(self):
        rospy.loginfo("初始化新比赛任务控制器（三阶段导航版本）...")
        
        # 初始化PID跟踪节点
        self.node = PIDTrackingNode()
        
        # 初始化机械臂串口
        self.arm_serial = self.init_arm_serial()
        
        # 任务选择数据
        self.choose_data = 0
        
        # 订阅GM65扫码结果
        rospy.Subscriber("/gm65_data", String, self.choose_callback)
        
        # 初始化target_done支持（来自nav_goal_new.py的成熟功能）
        self.completed_targets = 0  # 已完成的目标数量
        self.init_target_done_subscriber()
        
        # 初始化IMU支持（三阶段导航新增）
        self.current_yaw = 0.0  # 当前yaw角度
        self.initial_yaw = None  # 初始yaw角度
        self.imu_initialized = False  # IMU是否已初始化
        self.init_imu_subscriber()
        
        # 获取激光雷达距离参数
        self.lidar_distance = rospy.get_param("lader_point", [])
        self.process_lidar_distance()
        
        # 定义关键位置点（用于三阶段导航）
        self.key_positions = self.define_key_positions()
        
        # 定义精确距离参数（保留用于未来扩展）
        self.precise_distances = self.define_precise_distances()
        
        # 创建状态机
        self.sm = smach.StateMachine(outcomes=['TASK_COMPLETED', 'TASK_FAILED'])
        
        rospy.loginfo("新比赛任务控制器初始化完成")

    def process_lidar_distance(self):
        """处理激光雷达距离参数"""
        for i in range(len(self.lidar_distance)):
            for j in range(3):
                self.lidar_distance[i][j] -= 0.2
                if self.lidar_distance[i][j] <= 0:
                    self.lidar_distance[i][j] = 0.1

    def define_key_positions(self):
        """定义关键位置点 - 对应task_new.py中的goal_point位置（使用三阶段导航的第一阶段）"""
        # 注意：这些位置应该与PIDTrackingNode中的goal_point保持一致
        # 这里使用相对粗略的位置，精确定位由第三阶段完成
        return {
            'clinic_start': [0.0, 0.0, 0.0],           # goal_point[0] - 巡诊台/扫码位置
            'medicine_area_entry': [1.5, 1.0, 0.0],   # goal_point[1] - 药品区域入口
            'medicine_area_precise': [2.0, 1.5, 0.0], # goal_point[2] - 药品区域精确位置
            'transition_point': [2.5, 0.0, 0.0],      # goal_point[3] - 过渡点
            'delivery_area_entry': [3.5, -1.0, 0.0],  # goal_point[4] - 送药区域入口  
            'delivery_area_precise': [4.0, -1.5, 0.0],# goal_point[5] - 送药区域精确位置
            'return_point_x': [0.0, 0.0, 3.14],       # goal_point[6] - 返回点X（有特殊处理）
            'return_point_y': [0.0, 0.0, 3.14]        # goal_point[7] - 返回点Y（有特殊处理）
        }
    
    def define_precise_distances(self):
        """定义精确距离参数 - 第三阶段精确定位的目标距离
        目标点2和3的距离从distance.yaml读取"""
        
        # 默认距离配置
        distances = {
            'medicine_area_1': {
                'front': 0.30,   # 前方30cm
                'left': 0.25,    # 左侧25cm  
                'right': 0.25    # 右侧25cm
            },
            'medicine_area_2': {
                'front': 0.28,   # 前方28cm
                'left': 0.20,    # 左侧20cm
                'right': 0.30    # 右侧30cm
            },
            'delivery_point_1': {
                'front': 0.35,   # 前方35cm
                'left': 0.30,    # 左侧30cm
                'right': 0.20    # 右侧20cm
            }
        }
        
        # 从distance.yaml读取目标点2和3的距离
        try:
            distance_file_path = '/home/jetson/code_files/robocup/medical_ws/src/robcup/data/distance.yaml'
            rospy.loginfo(f"尝试从 {distance_file_path} 读取距离配置...")
            
            with open(distance_file_path, 'r', encoding='utf-8') as f:
                distance_data = yaml.safe_load(f)
            
            if 'lader_point' in distance_data and len(distance_data['lader_point']) >= 2:
                lader_point = distance_data['lader_point']
                
                # 目标点2的距离 (对应medicine_area_1)
                if len(lader_point[0]) >= 3:
                    point2_distances = lader_point[0]
                    distances['medicine_area_1'] = {
                        'front': point2_distances[0],  # 前方
                        'left': point2_distances[1],   # 左侧
                        'right': point2_distances[2]   # 右侧
                    }
                    rospy.loginfo(f"✅ 从distance.yaml读取目标点2距离: 前={point2_distances[0]:.3f}m, 左={point2_distances[1]:.3f}m, 右={point2_distances[2]:.3f}m")
                
                # 目标点3的距离 (对应medicine_area_2)
                if len(lader_point[1]) >= 3:
                    point3_distances = lader_point[1]
                    distances['medicine_area_2'] = {
                        'front': point3_distances[0],  # 前方
                        'left': point3_distances[1],   # 左侧
                        'right': point3_distances[2]   # 右侧
                    }
                    rospy.loginfo(f"✅ 从distance.yaml读取目标点3距离: 前={point3_distances[0]:.3f}m, 左={point3_distances[1]:.3f}m, 右={point3_distances[2]:.3f}m")
                
                rospy.loginfo("✅ distance.yaml距离配置读取成功")
            else:
                rospy.logwarn("distance.yaml格式不正确，使用默认距离配置")
                
        except FileNotFoundError:
            rospy.logwarn(f"未找到distance.yaml文件，使用默认距离配置")
        except Exception as e:
            rospy.logwarn(f"读取distance.yaml失败: {e}，使用默认距离配置")
        
        return distances

    def init_target_done_subscriber(self):
        """订阅target_done话题，监听C++控制器的目标完成计数（来自nav_goal_new.py）"""
        rospy.loginfo("订阅target_done话题，监听目标完成信号...")
        rospy.Subscriber('/target_done', Int32, self.target_done_callback)

    def target_done_callback(self, msg):
        """target_done回调函数，检查C++控制器的目标完成计数（来自nav_goal_new.py）"""
        if msg.data > self.completed_targets:
            self.completed_targets = msg.data
            rospy.loginfo(f"收到目标完成信号，总完成数: {msg.data}")

    def init_imu_subscriber(self):
        """订阅IMU话题，监听机器人姿态（三阶段导航新增）"""
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
        """IMU回调函数，提取yaw角度（三阶段导航新增）"""
        try:
            # 从四元数中提取yaw角度
            quaternion = [
                msg.orientation.x,
                msg.orientation.y, 
                msg.orientation.z,
                msg.orientation.w
            ]
            
            # 转换为欧拉角
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
            self.current_yaw = yaw
            
            # 第一次接收到IMU数据时，记录为初始姿态
            if not self.imu_initialized:
                self.initial_yaw = yaw
                self.imu_initialized = True
                rospy.loginfo(f"记录初始IMU姿态: yaw={yaw:.3f}rad ({math.degrees(yaw):.1f}°)")
                
        except Exception as e:
            rospy.logwarn(f"IMU数据处理异常: {e}")



    def choose_callback(self, msg):
        """扫码结果回调函数"""
        if msg.data == '1':
            self.choose_data = 1
        elif msg.data == '3':
            self.choose_data = 3
        rospy.loginfo(f"收到任务选择: {self.choose_data} (原始数据: {msg.data})")

    def init_arm_serial(self):
        """初始化机械臂串口连接"""
        port = '/dev/arm'
        baud_rate = 115200
        try:
            ser = serial.Serial(port, baud_rate, timeout=1)
            if ser.is_open:
                rospy.loginfo(f'机械臂串口 {port} 连接成功，波特率: {baud_rate}')
                return ser
            else:
                rospy.logwarn(f'机械臂串口 {port} 连接失败')
                return None
        except serial.SerialException as e:
            rospy.logwarn(f'机械臂串口连接异常: {e}')
            return None

    def config(self):
        """配置状态机 - 与task_new.py的config方法完全一致的逻辑"""
        if not hasattr(self, 'sm'): 
            self.sm = smach.StateMachine(outcomes=['TASK_COMPLETED'])  # 初始化 self.sm
        
        if self.choose_data == 1:
            self.config_task_sequence_1()
            # 创建并启动 introspection 服务来可视化状态机（与原始代码一致）
            self.sis = smach_ros.IntrospectionServer('server_name', self.sm, '/SM_ROOT')
        elif self.choose_data == 3:  
            self.config_task_sequence_3()
            # 创建并启动 introspection 服务来可视化状态机（与原始代码一致）
            self.sis = smach_ros.IntrospectionServer('server_name', self.sm, '/SM_ROOT')

    def run(self):
        """运行主任务 - 完全按照task_new.py的run方法逻辑"""
        # 巡诊台（与原始代码完全一致的注释和逻辑）
        rospy.loginfo("移动到巡诊台（goal_point[0]）...")
        self.node.task(self.node.goal_point[0][0], self.node.goal_point[0][1])
        
        # 等待扫码结果（与原始代码完全一致）
        rospy.loginfo("等待扫码结果...")
        self.choose_data = rospy.get_param("gm65_data", 0)
        while self.choose_data == 0:
            self.choose_data = rospy.get_param("gm65_data", 0)
            pass  # 与原始代码一致，使用pass而不是sleep
        
        rospy.loginfo(f"收到任务选择: {self.choose_data}")
        
        # 根据选择配置状态机（与原始代码一致）
        if self.choose_data == 1 or self.choose_data == 3:
            self.config()
            self.sis.start()
            # 执行状态机
            outcome = self.sm.execute()
            # 关闭 introspection 服务
            self.sis.stop()
            
            rospy.loginfo(f"三阶段导航任务完成，结果: {outcome}")
        else:
            rospy.logwarn(f"未知任务选择: {self.choose_data}")
            
    def test(self):
        """测试方法 - 保留与原始代码一致的测试功能"""
        self.choose_data = 1
        if not hasattr(self, 'sm'):
            self.sm = smach.StateMachine(outcomes=['TASK_COMPLETED'])
        try:
            with self.sm:
                smach.StateMachine.add(
                    'SEND_1', 
                    PushMedicineState(self.arm_serial, '1\r', "测试推药"),
                    transitions={'succeeded': 'TASK_COMPLETED', 'failed': 'TASK_COMPLETED'}
                )
                self.sis = smach_ros.IntrospectionServer('server_name', self.sm, '/SM_ROOT')
            self.sis.start()
            # 执行状态机
            outcome = self.sm.execute()
        except NameError:
            self.sm = smach.StateMachine(outcomes=['TASK_COMPLETED'])

    def config_task_sequence_1(self):
        """配置任务序列1 - 完全按照task_new.py的choose_data==1流程，使用三阶段导航替换TaskState"""
        rospy.loginfo("配置任务序列1（与task_new.py完全一致的流程）...")
        
        with self.sm:
            # TASK2: 三阶段导航到 goal_point[1] (药品区域入口)
            smach.StateMachine.add(
                'TASK2', 
                ThreeStageNavigationState(
                    self.node,
                    self.key_positions['medicine_area_entry'],  # 对应goal_point[1] 
                    self.lidar_distance[0][1],  # 与原始代码一致的距离参数
                    1,  # 轴1 = 左侧
                    "TASK2_药品区域入口",
                    parent_task=self
                ),
                transitions={'succeeded': 'TASK3', 'failed': 'TASK_COMPLETED'}
            )
            
            # TASK3: 三阶段导航到 goal_point[2] (药品区域精确位置)
            smach.StateMachine.add(
                'TASK3', 
                ThreeStageNavigationState(
                    self.node,
                    self.key_positions['medicine_area_precise'],  # 对应goal_point[2]
                    self.lidar_distance[1][1],  # 左侧精确定位
                    1,  # 轴1 = 左侧
                    "TASK3_药品区域精确位置",
                    parent_task=self
                ),
                transitions={'succeeded': 'FRONT_POSITION_LOOP', 'failed': 'TASK_COMPLETED'}
            )

            # FRONT_POSITION_LOOP: 前方精确定位（对应原始代码）
            smach.StateMachine.add(
                'FRONT_POSITION_LOOP', 
                PrecisePositionState(self.node, self.lidar_distance[1][0], axis=0, state_name="前方精确定位"),
                transitions={'succeeded': 'SEND_1', 'in_progress': 'FRONT_POSITION_LOOP', 'failed': 'TASK_COMPLETED'}
            )
            
            # SEND_1: 推药1（与原始代码完全一致）
            smach.StateMachine.add(
                'SEND_1', 
                PushMedicineState(self.arm_serial, '1\r', "推药1"),
                transitions={'succeeded': 'TASK4', 'failed': 'TASK_COMPLETED'}
            )
            
            # TASK4: PID导航到 goal_point[3] (过渡点)
            smach.StateMachine.add(
                'TASK4', 
                TaskNavigationState(self.node, 3, "TASK4_过渡点"),
                transitions={'succeeded': 'TASK5', 'failed': 'TASK_COMPLETED'}
            )
            
            # TASK5: PID导航到 goal_point[4] (送药区域入口)
            smach.StateMachine.add(
                'TASK5', 
                TaskNavigationState(self.node, 4, "TASK5_送药区域入口"),
                transitions={'succeeded': 'RIGHT_POSITION_LOOP', 'failed': 'TASK_COMPLETED'}
            )

            # RIGHT_POSITION_LOOP: 右侧精确定位
            smach.StateMachine.add(
                'RIGHT_POSITION_LOOP', 
                PrecisePositionState(self.node, self.lidar_distance[2][2], axis=2, state_name="右侧精确定位"),
                transitions={'succeeded': 'TASK6', 'in_progress': 'RIGHT_POSITION_LOOP', 'failed': 'TASK_COMPLETED'}
            )
            
            # TASK6: PID导航到 goal_point[5] (送药区域精确位置)
            smach.StateMachine.add(
                'TASK6', 
                TaskNavigationState(self.node, 5, "TASK6_送药区域精确位置"),
                transitions={'succeeded': 'RIGHT_POSITION_LOOP2', 'failed': 'TASK_COMPLETED'}
            )

            # RIGHT_POSITION_LOOP2: 右侧精确定位2
            smach.StateMachine.add(
                'RIGHT_POSITION_LOOP2', 
                PrecisePositionState(self.node, self.lidar_distance[3][2], axis=2, state_name="右侧精确定位2"),
                transitions={'succeeded': 'FRONT_POSITION_LOOP2', 'in_progress': 'RIGHT_POSITION_LOOP2', 'failed': 'TASK_COMPLETED'}
            )
            
            # FRONT_POSITION_LOOP2: 前方精确定位2
            smach.StateMachine.add(
                'FRONT_POSITION_LOOP2', 
                PrecisePositionState(self.node, self.lidar_distance[3][0], axis=0, state_name="前方精确定位2"),
                transitions={'succeeded': 'SEND_3', 'in_progress': 'FRONT_POSITION_LOOP2', 'failed': 'TASK_COMPLETED'}
            )
            
            # SEND_3: 推药3（与原始代码完全一致）
            smach.StateMachine.add(
                'SEND_3', 
                PushMedicineState(self.arm_serial, '3\r', "推药3"),
                transitions={'succeeded': 'RIGHT_POSITION_LOOP4', 'failed': 'TASK_COMPLETED'}
            )

            # RIGHT_POSITION_LOOP4: 右侧精确定位4（准备返回）
            smach.StateMachine.add(
                'RIGHT_POSITION_LOOP4', 
                PrecisePositionState(self.node, self.lidar_distance[2][2], axis=2, state_name="右侧精确定位4"),
                transitions={'succeeded': 'TASK7', 'in_progress': 'RIGHT_POSITION_LOOP4', 'failed': 'TASK_COMPLETED'}
            )
            
            # TASK7: PID导航到 goal_point[6] (返回点X，有特殊处理)
            smach.StateMachine.add(
                'TASK7', 
                TaskNavigationState(self.node, 6, "TASK7_返回点X"),
                transitions={'succeeded': 'TASK8', 'failed': 'TASK_COMPLETED'}
            )
            
            # TASK8: PID导航到 goal_point[7] (返回点Y，有特殊处理)
            smach.StateMachine.add(
                'TASK8', 
                TaskNavigationState(self.node, 7, "TASK8_返回点Y"),
                transitions={'succeeded': 'TASK_COMPLETED', 'failed': 'TASK_COMPLETED'}
            )

    def config_task_sequence_3(self):
        """配置任务序列3 - 完全按照task_new.py的choose_data==3流程"""
        rospy.loginfo("配置任务序列3（与task_new.py完全一致的流程）...")
        
        # 翻转所有目标点的Y坐标（与原始代码完全一致）
        for i in range(self.node.goal_point_num):
            self.node.goal_point[i][1] = -self.node.goal_point[i][1]
            rospy.loginfo(f"翻转goal_point[{i}]的Y坐标: {self.node.goal_point[i][1]}")
        
        with self.sm:
            # TASK2: 三阶段导航到 goal_point[1] (药品区域入口)
            smach.StateMachine.add(
                'TASK2', 
                ThreeStageNavigationState(
                    self.node,
                    self.key_positions['medicine_area_entry'],  # 对应goal_point[1] 
                    self.lidar_distance[2][2],  # 右侧精确定位（与选择3的逻辑一致）
                    2,  # 轴2 = 右侧
                    "TASK2_药品区域入口",
                    parent_task=self
                ),
                transitions={'succeeded': 'TASK3', 'failed': 'TASK_COMPLETED'}
            )
            
            # TASK3: 三阶段导航到 goal_point[2] (药品区域精确位置)
            smach.StateMachine.add(
                'TASK3', 
                ThreeStageNavigationState(
                    self.node,
                    self.key_positions['medicine_area_precise'],  # 对应goal_point[2]
                    self.lidar_distance[3][2],  # 右侧精确定位1
                    2,  # 轴2 = 右侧
                    "TASK3_药品区域精确位置",
                    parent_task=self
                ),
                transitions={'succeeded': 'FRONT_POSITION_LOOP', 'failed': 'TASK_COMPLETED'}
            )

            # FRONT_POSITION_LOOP: 前方精确定位（对应原始代码）
            smach.StateMachine.add(
                'FRONT_POSITION_LOOP', 
                PrecisePositionState(self.node, self.lidar_distance[3][0], axis=0, state_name="前方精确定位"),
                transitions={'succeeded': 'SEND_1', 'in_progress': 'FRONT_POSITION_LOOP', 'failed': 'TASK_COMPLETED'}
            )
            
            # SEND_1: 推药3（注意：任务3中先推的是'3\r'）
            smach.StateMachine.add(
                'SEND_1', 
                PushMedicineState(self.arm_serial, '3\r', "推药3"),
                transitions={'succeeded': 'TASK4', 'failed': 'TASK_COMPLETED'}
            )
            
            # TASK4: PID导航到 goal_point[3] (过渡点)
            smach.StateMachine.add(
                'TASK4', 
                TaskNavigationState(self.node, 3, "TASK4_过渡点"),
                transitions={'succeeded': 'TASK5', 'failed': 'TASK_COMPLETED'}
            )
            
            # TASK5: PID导航到 goal_point[4] (送药区域入口)
            smach.StateMachine.add(
                'TASK5', 
                TaskNavigationState(self.node, 4, "TASK5_送药区域入口"),
                transitions={'succeeded': 'LEFT_POSITION_LOOP', 'failed': 'TASK_COMPLETED'}
            )

            # LEFT_POSITION_LOOP: 左侧精确定位
            smach.StateMachine.add(
                'LEFT_POSITION_LOOP', 
                PrecisePositionState(self.node, self.lidar_distance[0][1], axis=1, state_name="左侧精确定位"),
                transitions={'succeeded': 'TASK6', 'in_progress': 'LEFT_POSITION_LOOP', 'failed': 'TASK_COMPLETED'}
            )
            
            # TASK6: PID导航到 goal_point[5] (送药区域精确位置)
            smach.StateMachine.add(
                'TASK6', 
                TaskNavigationState(self.node, 5, "TASK6_送药区域精确位置"),
                transitions={'succeeded': 'LEFT_POSITION_LOOP2', 'failed': 'TASK_COMPLETED'}
            )

            # LEFT_POSITION_LOOP2: 左侧精确定位2
            smach.StateMachine.add(
                'LEFT_POSITION_LOOP2', 
                PrecisePositionState(self.node, self.lidar_distance[1][1], axis=1, state_name="左侧精确定位2"),
                transitions={'succeeded': 'FRONT_POSITION_LOOP2', 'in_progress': 'LEFT_POSITION_LOOP2', 'failed': 'TASK_COMPLETED'}
            )
            
            # FRONT_POSITION_LOOP2: 前方精确定位2
            smach.StateMachine.add(
                'FRONT_POSITION_LOOP2', 
                PrecisePositionState(self.node, self.lidar_distance[1][0], axis=0, state_name="前方精确定位2"),
                transitions={'succeeded': 'SEND_3', 'in_progress': 'FRONT_POSITION_LOOP2', 'failed': 'TASK_COMPLETED'}
            )
            
            # SEND_3: 推药1（注意：任务3中后推的是'1\r'）
            smach.StateMachine.add(
                'SEND_3', 
                PushMedicineState(self.arm_serial, '1\r', "推药1"),
                transitions={'succeeded': 'LEFT_POSITION_LOOP4', 'failed': 'TASK_COMPLETED'}
            )

            # LEFT_POSITION_LOOP4: 左侧精确定位4（准备返回）
            smach.StateMachine.add(
                'LEFT_POSITION_LOOP4', 
                PrecisePositionState(self.node, self.lidar_distance[0][1], axis=1, state_name="左侧精确定位4"),
                transitions={'succeeded': 'TASK7', 'in_progress': 'LEFT_POSITION_LOOP4', 'failed': 'TASK_COMPLETED'}
            )
            
            # TASK7: PID导航到 goal_point[6] (返回点X，有特殊处理)
            smach.StateMachine.add(
                'TASK7', 
                TaskNavigationState(self.node, 6, "TASK7_返回点X"),
                transitions={'succeeded': 'TASK8', 'failed': 'TASK_COMPLETED'}
            )
            
            # TASK8: PID导航到 goal_point[7] (返回点Y，有特殊处理)
            smach.StateMachine.add(
                'TASK8', 
                TaskNavigationState(self.node, 7, "TASK8_返回点Y"),
                transitions={'succeeded': 'TASK_COMPLETED', 'failed': 'TASK_COMPLETED'}
            )

if __name__ == '__main__':
    """主函数 - 与task_new.py保持一致的启动方式"""
    rospy.init_node('newtask_test_node', anonymous=True)
    rospy.loginfo("启动新任务测试节点（三阶段导航版本）...")
    
    try:
        # 创建任务实例（与原始代码一致）
        task = NewRaceTask()
        task.run()
        # task.test()  # 取消注释可以进行测试
        
    except rospy.ROSInterruptException:
        rospy.loginfo("接收到ROS中断信号，正在退出...")
    except Exception as e:
        rospy.logerr(f"程序异常: {e}")
    finally:
        rospy.loginfo("三阶段导航任务节点结束")