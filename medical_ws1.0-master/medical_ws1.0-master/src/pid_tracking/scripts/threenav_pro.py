#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
简化版三阶段导航系统 - 从threenav_new.py迁移而来
功能：提供最简化的导航调用方式，避免嵌套，完全拆开goto_point_with_arm_task的逻辑

主要特性:
1. 简化设计：
   - 所有三阶段类重新定义，不依赖threestatenav
   - 第三阶段使用pid_node.py中的PID控制
   - 完全避免嵌套调用，使用最基础的函数

2. 清晰的调用方式：
   - maintest()函数：完全拆开的导航逻辑
   - 每个步骤都是独立的函数调用
   - 支持动态启动/停止move_base

3. 独立运行：
   - 不依赖其他导航文件
   - 只导入实际使用的库
   - 可以完全独立运行
"""

# 导入必要的库
import rospy
import actionlib
import subprocess
import os
import signal
import math
import serial
import time
from std_msgs.msg import Int32, String
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import Imu
import tf.transformations

# 从pid_P.py导入PID控制相关
from pid_P import PIDTrackingNode


class AdaptiveNavigationState:
    """自适应导航状态 - 使用move_base导航到大致位置，基于C++控制器的完成信号"""
    def __init__(self, target_pose, state_name="", parent_navigator=None, expected_count=None):
        self.target_pose = target_pose  # [x, y] 只使用位置，不使用yaw
        self.state_name = state_name
        self.parent_navigator = parent_navigator  # 引用到TwoStageNavigator以访问共享资源
        self.expected_count = expected_count  # 外部传入的期待值
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
        """创建move_base目标消息，只设置位置，不设置方向"""
        goal = MoveBaseGoal()
        
        # 设置目标帧
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        
        # 设置位置
        goal.target_pose.pose.position.x = target_pose[0]
        goal.target_pose.pose.position.y = target_pose[1]
        goal.target_pose.pose.position.z = 0.0
        
        # 不设置具体方向，使用默认四元数 (w=1.0，表示无旋转)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0
        
        return goal

    def execute(self):
        """执行自适应导航"""
        rospy.loginfo(f"阶段1 - 自适应导航: {self.state_name}")
        rospy.loginfo(f"目标位置: ({self.target_pose[0]:.3f}, {self.target_pose[1]:.3f})")
        
        if self.move_base_client is None:
            rospy.logerr("move_base客户端未初始化")
            return False

        if self.parent_navigator is None:
            rospy.logerr("未提供parent_navigator引用")
            return False

        # 检查是否提供了期待值
        if self.expected_count is None:
            rospy.logerr("未提供expected_count，无法执行导航")
            return False
        
        expected_count = self.expected_count
        
        # 创建并发送目标
        goal_msg = self.create_goal_message(self.target_pose)
        
        try:
            rospy.loginfo("发送导航目标...")
            self.move_base_client.send_goal(goal_msg)
            
            # 等待C++控制器确认到达
            start_time = rospy.Time.now()
            
            while not rospy.is_shutdown() and self.parent_navigator.completed_targets < expected_count:
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
            
            if self.parent_navigator.completed_targets >= expected_count:
                rospy.loginfo(f"阶段1完成 - 自适应导航成功: {self.state_name} (完成计数: {self.parent_navigator.completed_targets})")
                rospy.loginfo("!!!!!!move_base cancel!!!!!!")
                self.move_base_client.cancel_goal()
                return True
            else:
                rospy.logwarn(f"导航被中断: {self.state_name}")
                return False
                
        except Exception as e:
            rospy.logerr(f"导航执行异常: {e}")
            return False


class ImuAttitudeRecoveryState:
    """IMU姿态恢复状态 - 将机器人yaw角度恢复到初始姿态"""
    def __init__(self, node, target_yaw, state_name="", tolerance=0.05, max_attempts=200):
        self.node = node
        self.target_yaw = target_yaw  # 目标yaw角度（初始姿态）
        self.state_name = state_name
        self.tolerance = tolerance  # 角度容差，默认约3度
        self.max_attempts = max_attempts  # 最大尝试次数
        self.current_yaw = 0.0
        
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
        
    def execute(self):
        """执行IMU姿态恢复"""
        rospy.loginfo(f"阶段2 - IMU姿态恢复: {self.state_name}")
        rospy.loginfo(f"目标yaw: {self.target_yaw:.3f}rad ({math.degrees(self.target_yaw):.1f}°)")
        
        yaw_error = self.calculate_yaw_error()
        error_abs = abs(yaw_error)
        
        rospy.loginfo(f"yaw误差: {yaw_error:.3f}rad ({math.degrees(yaw_error):.1f}°)")
        
        # 检查是否已经在容差范围内
        if error_abs <= self.tolerance:
            rospy.loginfo(f"✓ IMU姿态已在容差范围内 (误差: {math.degrees(error_abs):.1f}°)")
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
        
        return 'in_progress'


class PrecisePositionState:
    """精确定位状态 - 使用PID控制进行精确距离定位（与task_new.py中的PositionLoopState完全一致）"""
    def __init__(self, node, target_distance, axis, state_name="", tolerance_done=0.01):
        self.node = node
        self.target_distance = target_distance
        self.axis = axis  # 0=前方, 1=左侧, 2=右侧
        self.state_name = state_name
        # 与原始task_new.py保持一致的精度设置
        self.tolerance = 0.01  # 固定精度，与原始代码一致
        self.tolerance_done = tolerance_done  # 完成判断的精度，可配置

    def execute(self):
        """执行精确定位（与task_new.py中的PositionLoopState.execute完全一致）"""
        rospy.loginfo(f"阶段3 - 精确定位: {self.state_name}")
        rospy.loginfo(f"轴{self.axis}, 目标距离{self.target_distance:.3f}m, 完成精度{self.tolerance_done:.3f}m")
        
        try:
            # 获取当前距离
            if self.axis >= len(self.node.distance):
                rospy.logerr(f"距离传感器轴{self.axis}不存在")
                return False
                
            current_distance = self.node.distance[self.axis]
            error = abs(current_distance - self.target_distance)
            
            rospy.loginfo(f"当前距离: {current_distance:.3f}m, 目标: {self.target_distance:.3f}m, 误差: {error:.3f}m")
            
            # 使用可配置的完成精度进行判断
            if abs(self.node.distance[self.axis] - self.target_distance) > self.tolerance_done:
                self.node.distance_state = self.axis + 1
                self.node.position_loop(self.target_distance)
                return 'in_progress'  # 继续定位
            
            # 精度满足要求，停止运动
            self.node.publish_vel(0, 0, 0)
            rospy.loginfo(f"阶段3完成 - 精确定位成功: {self.state_name}")
            return 'succeeded'  # 定位完成
            
        except Exception as e:
            rospy.logerr(f"精确定位异常: {e}")
            self.node.publish_vel(0, 0, 0)
            return 'failed'


class GM65Scanner:
    """GM65扫码器 - 获取二维码中的数字（1或3）"""
    
    def __init__(self, scan_timeout=30.0):
        self.scan_timeout = scan_timeout
        self.scanned_data = None
        self.scanning_complete = False
        self.gm65_subscriber = None
        
    def gm65_callback(self, msg):
        """GM65扫码回调函数"""
        try:
            scanned_value = msg.data.strip()
            rospy.loginfo(f"📱 GM65扫码接收到原始数据: '{scanned_value}' (类型: {type(scanned_value)})")
            
            # 验证扫码数据是否为有效值（1或3）
            if scanned_value in ['1', '3']:
                self.scanned_data = int(scanned_value)
                self.scanning_complete = True
                rospy.loginfo(f"✅ 扫码解析成功！")
                rospy.loginfo(f"   原始数据: '{scanned_value}' (str)")
                rospy.loginfo(f"   解析结果: {self.scanned_data} (int)")
                rospy.loginfo(f"   任务类型: {'任务A' if self.scanned_data == 1 else '任务B'}")
            else:
                rospy.logwarn(f"⚠️ 扫码数据无效: '{scanned_value}', 期望值: '1' 或 '3'")
                
        except Exception as e:
            rospy.logerr(f"GM65扫码数据处理异常: {e}")
            rospy.logerr(f"原始数据: {msg.data}, 类型: {type(msg.data)}")
    
    def scan(self):
        """执行GM65扫码，返回1或3"""
        rospy.loginfo("开始GM65扫码...")
        rospy.loginfo("等待扫描二维码...")
        
        # 重置状态
        self.scanned_data = None
        self.scanning_complete = False
        
        # 订阅GM65扫码话题
        self.gm65_subscriber = rospy.Subscriber("/gm65_data", String, self.gm65_callback)
        
        try:
            start_time = rospy.Time.now()
            
            # 等待扫码完成或超时
            while not self.scanning_complete and not rospy.is_shutdown():
                elapsed_time = (rospy.Time.now() - start_time).to_sec()
                
                # 检查超时
                if elapsed_time > self.scan_timeout:
                    rospy.logwarn(f"GM65扫码超时 ({self.scan_timeout}秒)，使用默认值1")
                    self.scanned_data = 1  # 超时默认返回1
                    break
                
                # 每5秒输出一次等待状态
                if int(elapsed_time) % 5 == 0 and elapsed_time > 0:
                    if int(elapsed_time) != getattr(self, '_last_log_time', -1):
                        rospy.loginfo(f"正在等待扫码... 已等待 {elapsed_time:.0f}s")
                        self._last_log_time = int(elapsed_time)
                
                rospy.sleep(0.1)
            
            # 取消订阅
            if self.gm65_subscriber:
                self.gm65_subscriber.unregister()
                self.gm65_subscriber = None
            
            # 返回扫码结果
            rospy.loginfo(f"✅ GM65扫码完成！")
            rospy.loginfo(f"   扫码结果: {self.scanned_data}")
            rospy.loginfo(f"   任务类型: {'任务A' if self.scanned_data == 1 else '任务B'}")
            rospy.loginfo(f"   导航策略: {'2→3→4' if self.scanned_data == 1 else '3→2→4'}")
            
            return self.scanned_data
                
        except Exception as e:
            rospy.logerr(f"GM65扫码执行异常: {e}")
            # 确保取消订阅
            if self.gm65_subscriber:
                self.gm65_subscriber.unregister()
            # 异常时返回默认值
            rospy.logwarn("扫码异常，使用默认值1")
            return 1


class ArmController:
    """机械臂控制器 - 负责与上位机串口通信"""
    
    def __init__(self):
        self.arm_serial = self._init_arm_serial()
        
    def _init_arm_serial(self):
        """初始化机械臂串口连接"""
        port = '/dev/arm'
        baud_rate = 115200
        try:
            ser = serial.Serial(port, baud_rate, timeout=1)
            if ser.is_open:
                rospy.loginfo(f'🤖 串口 {port} 成功打开，波特率: {baud_rate}')
                return ser
            else:
                rospy.logerr(f'❌ 无法打开串口 {port}')
                return None
        except serial.SerialException as e:
            rospy.logerr(f'❌ 打开串口时发生错误: {e}')
            return None
    
    def send_task_command(self, task_id):
        """
        发送任务指令并等待回复
        
        参数:
            task_id (int): 任务编号，1或3
            
        返回:
            tuple: (success, response_data)
                success (bool): 是否成功
                response_data (str): 返回的数据
        """
        if self.arm_serial is None:
            rospy.logerr("🤖 串口未初始化成功")
            return False, "串口错误"
        
        # 根据task_id值确定发送的指令
        if task_id == 1:
            command = '1\r'
            rospy.loginfo("🤖 准备发送指令: '1\\r' (执行任务1)")
        elif task_id == 3:
            command = '3\r'
            rospy.loginfo("🤖 准备发送指令: '3\\r' (执行任务3)")
        else:
            rospy.logerr(f"🤖 无效的任务编号: {task_id}，只支持1或3")
            return False, "无效任务编号"
        
        try:
            # 步骤1: 发送指令给上位机
            rospy.loginfo("🤖 正在发送指令...")
            self.arm_serial.write(command.encode('utf-8'))
            rospy.loginfo(f"🤖 指令已发送: {repr(command)}")
            
            # 步骤2: 等待上位机返回确认数据
            rospy.loginfo("🤖 等待上位机回复...")
            timeout_counter = 0
            max_timeout = 100  # 最大等待次数，避免无限等待
            
            while not self.arm_serial.in_waiting and timeout_counter < max_timeout:
                time.sleep(0.1)  # 短暂等待
                timeout_counter += 1
            
            if timeout_counter >= max_timeout:
                rospy.logerr("🤖 等待回复超时")
                return False, "超时"
            
            # 步骤3: 读取上位机返回的消息
            data = self.arm_serial.readline().decode('utf-8').strip()
            rospy.loginfo(f"🤖 收到回复: '{data}'")
            
            # 步骤4: 处理返回结果
            if data:
                rospy.loginfo(f"🤖 指令执行成功，收到数据: {data}")
                rospy.loginfo("🤖 等待2秒让动作完成...")
                time.sleep(2)  # 等待2秒让动作完成
                return True, data
            else:
                rospy.logerr("🤖 没有收到有效数据")
                return False, "无数据"
                
        except Exception as e:
            rospy.logerr(f"🤖 发送指令或接收数据时发生错误: {e}")
            return False, str(e)
    
    def close(self):
        """关闭串口连接"""
        if self.arm_serial and self.arm_serial.is_open:
            self.arm_serial.close()
            rospy.loginfo("🤖 串口连接已关闭")


class TaskConfigManager:
    """任务配置管理器 - 管理点位配置、精确定位参数等"""
    
    def __init__(self):
        self.navigation_goals = self._define_navigation_goals()
        self.precise_distances = self._define_precise_distances()
        self.default_accuracy_locations = self._define_default_accuracy_locations()
        
        # 加载激光雷达距离参数（与原始代码一致）
        self.lidar_distance = rospy.get_param("lader_point", [])
        if self.lidar_distance:
            rospy.loginfo("从ROS参数加载lader_point距离数据")
            self._preprocess_lidar_distance()
            self._update_distances_from_params()
        else:
            rospy.logwarn("未找到lader_point参数，使用默认硬编码距离")
    
    def _define_navigation_goals(self):
        """定义导航目标点 - 粗略导航的目标位置"""
        return {
            1: {'position': [1.38, 0.05], 'name': '目标点1'},     # 起始区域
            2: {'position': [4.97, 2.19], 'name': '目标点2'},    # 药品区域1  
            3: {'position': [5.03, -1.61], 'name': '目标点3'},   # 药品区域2
            4: {'position': [0.0, 0.0], 'name': '目标点4'}    # 返回点
        }
    
    def _define_precise_distances(self):
        """定义精确距离参数 - 精确定位的目标距离"""
        return {
            1: {'front': 0.20, 'left': 0.30, 'right': 0.30},
            2: {'front': 0.53, 'left': 1.32, 'right': 5.81},
            3: {'front': 0.47, 'left': 5.79, 'right': 1.33},
            4: {'front': 0.40, 'left': 0.35, 'right': 0.35}
        }
    
    def _define_default_accuracy_locations(self):
        """
        定义默认的精确定位控制参数 - acc_loc[需要精确定位, 优先级顺序列表, IMU恢复开关]
        
        格式说明:
        acc_loc[0]: 是否需要精确定位 (True/False)
        acc_loc[1]: 精确定位方向的优先级顺序列表 (e.g., ['front', 'left', 'right'])
        acc_loc[2]: 是否需要IMU姿态恢复 (True/False) - 新增功能
        
        示例:
        acc_loc = [True, ['front'], True]  - 需要精确定位+IMU恢复，只使用前方True
        acc_loc = [True, ['left', 'front'], False]  - 需要精确定位但跳过IMU恢复
        acc_loc = [False, [], True] - 跳过精确定位，只进行IMU恢复True
        acc_loc = [False, [], False] - 跳过精确定位和IMU恢复，只粗导航
        """
        return {
            1: [False, [], True],   # 目标点1: 不需要精确定位，需要IMU恢复
            2: [True, ['left', 'front'], True],   # 目标点2: 需要精确定位，按左侧->前方顺序，需要IMU恢复
            3: [True, ['right', 'front'], True],   # 目标点3: 需要精确定位，按右侧->前方顺序，需要IMU恢复
            4: [False, [], False]  # 目标点4: 不需要精确定位，不需要IMU恢复
        }
    
    def _preprocess_lidar_distance(self):
        """预处理激光雷达距离数据（与原始代码一致）"""
        for i in range(len(self.lidar_distance)):
            for j in range(3):
                self.lidar_distance[i][j] -= 0.2
                if self.lidar_distance[i][j] <= 0:
                    self.lidar_distance[i][j] = 0.1
    
    def _update_distances_from_params(self):
        """从ROS参数lader_point更新精确距离配置（与原始代码一致）"""
        if not self.lidar_distance or len(self.lidar_distance) < 4:
            rospy.logwarn("lader_point参数数据不足，保持默认配置")
            return
        
        try:
            rospy.loginfo("使用lader_point参数更新精确距离配置...")
            
            # 确保lidar_distance有足够的数据
            for i in range(4):
                if i >= len(self.lidar_distance):
                    rospy.logwarn(f"lader_point[{i}]不存在，跳过")
                    continue
                if len(self.lidar_distance[i]) < 3:
                    rospy.logwarn(f"lader_point[{i}]数据不完整，跳过")
                    continue
            
            # 更新目标点距离配置（基于原始代码的使用模式）
            self.precise_distances.update({
                1: {
                    'front': self.lidar_distance[1][0] if len(self.lidar_distance) > 1 else 0.35,
                    'left': self.lidar_distance[0][1] if len(self.lidar_distance) > 0 else 0.30,  
                    'right': self.lidar_distance[2][2] if len(self.lidar_distance) > 2 else 0.30,
                },
                2: {
                    'front': self.lidar_distance[1][0] if len(self.lidar_distance) > 1 else 0.30,
                    'left': self.lidar_distance[1][1] if len(self.lidar_distance) > 1 else 0.25,
                    'right': self.lidar_distance[3][2] if len(self.lidar_distance) > 3 else 0.25,
                },
                3: {
                    'front': self.lidar_distance[3][0] if len(self.lidar_distance) > 3 else 0.28,
                    'left': self.lidar_distance[1][1] if len(self.lidar_distance) > 1 else 0.20,
                    'right': self.lidar_distance[2][2] if len(self.lidar_distance) > 2 else 0.30,
                },
                4: {
                    'front': self.lidar_distance[3][0] if len(self.lidar_distance) > 3 else 0.40,
                    'left': self.lidar_distance[0][1] if len(self.lidar_distance) > 0 else 0.35,
                    'right': self.lidar_distance[3][2] if len(self.lidar_distance) > 3 else 0.35,
                }
            })
            
            rospy.loginfo("成功从lader_point参数更新精确距离配置")
            
            # 输出更新后的配置
            for goal_id, distances in self.precise_distances.items():
                rospy.loginfo(f"目标点{goal_id}: 前方={distances['front']:.3f}m, 左侧={distances['left']:.3f}m, 右侧={distances['right']:.3f}m")
                
        except Exception as e:
            rospy.logerr(f"从lader_point参数更新距离配置失败: {e}")
            rospy.logwarn("保持默认硬编码距离配置")
    
    def get_point_config(self, point_id):
        """获取指定点位的配置信息"""
        if point_id not in self.navigation_goals:
            raise ValueError(f"点位ID {point_id} 不存在")
        
        return PointConfig(
            point_id=point_id,
            position=self.navigation_goals[point_id]['position'],
            name=self.navigation_goals[point_id]['name'],
            precise_distances=self.precise_distances[point_id],
            default_acc_loc=self.default_accuracy_locations[point_id]
        )
    
    def get_task_sequence(self, tasktype):
        """根据任务类型获取导航序列"""
        if tasktype == 1:
            sequence = [1, 2, 3, 4]
            rospy.loginfo("任务序列1: 1 -> 2 -> 3 -> 4")
        elif tasktype == 3:
            sequence = [1, 3, 2, 4]
            rospy.loginfo("任务序列3: 1 -> 3 -> 2 -> 4")
        else:
            rospy.logwarn(f"未知任务类型: {tasktype}，使用默认序列")
            sequence = [1, 2, 3, 4]
        
        return sequence


class PointConfig:
    """单个点位的配置信息"""
    
    def __init__(self, point_id, position, name, precise_distances, default_acc_loc):
        self.point_id = point_id
        self.position = position
        self.name = name
        self.precise_distances = precise_distances
        self.acc_loc = default_acc_loc.copy()  # 使用默认配置
    
    def set_precise_config(self, enable_precise, precise_order=None, enable_imu_recovery=None):
        """设置精确定位和IMU恢复配置"""
        self.acc_loc[0] = enable_precise
        if enable_precise and precise_order:
            self.acc_loc[1] = precise_order
        elif not enable_precise:
            self.acc_loc[1] = []
        
        # 设置IMU恢复配置
        if enable_imu_recovery is not None:
            # 确保acc_loc有3个参数
            if len(self.acc_loc) == 2:
                self.acc_loc.append(enable_imu_recovery)
            else:
                self.acc_loc[2] = enable_imu_recovery
        elif len(self.acc_loc) == 2:
            # 如果没有指定IMU恢复配置，使用默认值True
            self.acc_loc.append(True)
        
        rospy.loginfo(f"更新点位{self.point_id}配置: 精确定位={self.acc_loc[0]}, 方向={self.acc_loc[1]}, IMU恢复={self.acc_loc[2]}")
    
    def get_target_distance(self, direction):
        """获取指定方向的目标距离"""
        if direction not in self.precise_distances:
            raise ValueError(f"方向 {direction} 不存在")
        return self.precise_distances[direction]


class BaseNavigationService:
    """基础导航服务 - 提供基础的导航能力和状态管理"""
    
    def __init__(self):
        # 初始化PID跟踪节点 (它会自动初始化ROS节点)
        self.node = PIDTrackingNode()
        rospy.loginfo("初始化基础导航服务...")
        
        # 初始化target_done支持
        self.completed_targets = 0
        self._init_target_done_subscriber()
        
        # 初始化IMU支持
        self.current_yaw = 0.0
        self.initial_yaw = None
        self.imu_initialized = False
        self._init_imu_subscriber()
        
        # 初始化arm_done发布器
        self.arm_done_pub = rospy.Publisher('/arm_done', Int32, queue_size=1)
        self.arm_done_count = 0
        
        rospy.loginfo("基础导航服务初始化完成")
    
    def _init_target_done_subscriber(self):
        """订阅target_done话题，监听C++控制器的目标完成计数"""
        rospy.loginfo("订阅target_done话题，监听目标完成信号...")
        rospy.Subscriber('/target_done', Int32, self._target_done_callback)
    
    def _target_done_callback(self, msg):
        """target_done回调函数，检查C++控制器的目标完成计数"""
        if msg.data > self.completed_targets:
            self.completed_targets = msg.data
            rospy.loginfo("!!!!target_done renew!!!!!")
            rospy.loginfo(f"收到目标完成信号，总完成数: {msg.data}")
    
    def _init_imu_subscriber(self):
        """订阅IMU话题，监听机器人姿态"""
        rospy.loginfo("订阅/handsfree/imu话题，监听机器人姿态...")
        rospy.Subscriber('/handsfree/imu', Imu, self._imu_callback)
        
        # 等待IMU数据初始化
        rospy.loginfo("等待IMU数据初始化...")
        init_timeout = 10.0
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
    
    def _imu_callback(self, msg):
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
    
    def increment_arm_done(self):
        """机械臂任务完成后增加arm_done计数"""
        self.arm_done_count += 1
        self.arm_done_pub.publish(self.arm_done_count)
        rospy.loginfo(f"发布arm_done: {self.arm_done_count}")
    
    def emergency_stop(self):
        """紧急停止"""
        rospy.logwarn("执行紧急停止")
        self.node.publish_vel(0, 0, 0)


class SinglePointNavigator:
    """单点导航器 - 处理单个点的三阶段导航"""
    
    def __init__(self, nav_service):
        self.nav_service = nav_service
        rospy.loginfo("初始化单点导航器")
    
    def navigate_to_point(self, point_config):
        """导航到指定点位，执行三阶段导航"""
        # 确保acc_loc有3个参数（向后兼容）
        if len(point_config.acc_loc) == 2:
            point_config.acc_loc.append(True)  # 默认启用IMU恢复
            rospy.logwarn(f"点位{point_config.point_id}的acc_loc只有2个参数，自动启用IMU恢复")
        
        rospy.loginfo(f"\n{'='*60}")
        rospy.loginfo(f"开始导航到 {point_config.name} (ID: {point_config.point_id})")
        rospy.loginfo(f"粗略位置: ({point_config.position[0]:.2f}, {point_config.position[1]:.2f})")
        
        # 检查各阶段配置
        need_precise = point_config.acc_loc[0]
        need_imu_recovery = point_config.acc_loc[2]
        
        precise_icon = "🎯" if need_precise else "🚀"
        imu_icon = "🧭" if need_imu_recovery else "⏭️"
        
        rospy.loginfo(f"{precise_icon} 精确定位需求: {'是' if need_precise else '否'}")
        rospy.loginfo(f"{imu_icon} IMU恢复需求: {'是' if need_imu_recovery else '否'}")
        rospy.loginfo(f"📋 完整acc_loc配置: {point_config.acc_loc}")
        
        # 构建执行计划
        execution_plan = ["阶段1=粗导航"]
        if need_imu_recovery:
            execution_plan.append("阶段2=IMU恢复")
        else:
            execution_plan.append("阶段2=跳过")
            
        if need_precise:
            directions = []
            priority_order = point_config.acc_loc[1]
            for direction in priority_order:
                distance = point_config.get_target_distance(direction)
                directions.append(f"{direction}({distance:.3f}m)")
            rospy.loginfo(f"💡 精确定位顺序: {', '.join(directions) if directions else '无'}")
            execution_plan.append("阶段3=精确定位")
        else:
            execution_plan.append("阶段3=跳过")
        
        rospy.loginfo(f"📋 执行计划: {' → '.join(execution_plan)}")
        rospy.loginfo(f"{'='*60}")
        
        # 执行三阶段导航
        success = self._execute_three_stages(point_config)
        
        if success:
            rospy.loginfo(f"✓ 三阶段导航完成: {point_config.name}")
        else:
            rospy.logerr(f"✗ 三阶段导航失败: {point_config.name}")
        
        return success
    
    def _execute_three_stages(self, point_config):
        """执行三阶段导航流程"""
        # 阶段1: 自适应导航
        if not self._execute_adaptive_navigation(point_config):
            return False
        
        # 阶段2: IMU姿态恢复（根据acc_loc[2]配置决定）
        need_imu_recovery = point_config.acc_loc[2]
        if need_imu_recovery:
            if not self._execute_imu_recovery(point_config):
                return False
        else:
            rospy.loginfo("⏭️ 跳过阶段2 - IMU姿态恢复 (acc_loc[2]=False)")
        
        # 阶段3: 精确定位（如果需要）
        if point_config.acc_loc[0]:  # 需要精确定位
            if not self._execute_precise_positioning(point_config):
                return False
        
        return True
    
    def _execute_adaptive_navigation(self, point_config):
        """执行阶段1: 自适应导航"""
        adaptive_nav = AdaptiveNavigationState(
            point_config.position, 
            f"{point_config.name}_粗略导航",
            parent_navigator=self.nav_service
        )
        
        success = adaptive_nav.execute()
        if not success:
            rospy.logerr(f"阶段1失败: {point_config.name}")
            return False
        
        rospy.loginfo("阶段1完成，准备进入阶段2...")
        rospy.sleep(0.3)
        return True
    
    def _execute_imu_recovery(self, point_config):
        """执行阶段2: IMU姿态恢复"""
        rospy.loginfo("🧭 开始执行阶段2 - IMU姿态恢复")
        
        imu_recovery = ImuAttitudeRecoveryState(
            self.nav_service.node,
            self.nav_service.initial_yaw,
            f"{point_config.name}_姿态恢复"
        )
        
        # 执行IMU姿态恢复
        max_imu_attempts = 1000
        imu_attempts = 0
        
        while imu_attempts < max_imu_attempts and not rospy.is_shutdown():
            imu_recovery.set_current_yaw(self.nav_service.current_yaw)
            result = imu_recovery.execute()
            
            if result == 'succeeded':
                rospy.loginfo("✓ 阶段2完成 - IMU姿态恢复成功")
                rospy.loginfo("阶段2完成，准备进入阶段3...")
                rospy.sleep(0.3)
                return True
            elif result == 'failed':
                rospy.logerr(f"阶段2失败: {point_config.name} - IMU姿态恢复")
                return False
            elif result == 'in_progress':
                imu_attempts += 1
            else:
                rospy.logerr(f"未知的IMU姿态恢复结果: {result}")
                return False
        
        rospy.logerr(f"阶段2超时: {point_config.name} - IMU姿态恢复")
        return False
    
    def _execute_precise_positioning(self, point_config):
        """执行阶段3: 精确定位"""
        priority_order = point_config.acc_loc[1]
        direction_to_axis = {'front': 0, 'left': 1, 'right': 2}
        
        for direction in priority_order:
            if direction not in direction_to_axis:
                rospy.logerr(f"无效的方向: {direction}")
                return False
            
            axis = direction_to_axis[direction]
            target_distance = point_config.get_target_distance(direction)
            
            rospy.loginfo(f"执行{direction}精确定位，目标距离: {target_distance:.3f}m")
            
            precise_pos = PrecisePositionState(
                self.nav_service.node,
                target_distance,
                axis,
                f"{point_config.name}_{direction}精确定位"
            )
            
            # 使用与原始代码一致的循环控制逻辑
            max_position_attempts = 10000
            position_attempts = 0
            
            while position_attempts < max_position_attempts and not rospy.is_shutdown():
                result = precise_pos.execute()
                
                if result == 'succeeded':
                    rospy.loginfo(f"✓ {direction}精确定位完成")
                    break
                elif result == 'failed':
                    rospy.logerr(f"阶段3失败: {point_config.name} - {direction}精确定位")
                    return False
                elif result == 'in_progress':
                    position_attempts += 1
                else:
                    rospy.logerr(f"未知的精确定位结果: {result}")
                    return False
            
            if position_attempts >= max_position_attempts:
                rospy.logerr(f"精确定位超时: {point_config.name} - {direction}精确定位")
                return False
            
            rospy.loginfo(f"阶段3完成 - {direction}精确定位成功")
            rospy.sleep(0.3)
        
        rospy.loginfo("✓ 阶段3完成 - 所有方向精确定位成功")
        return True


class MainController:
    """主控制器 - 实现灵活的点位导航调用"""
    
    def __init__(self, tasktype=1):
        self.tasktype = tasktype
        
        # 初始化各个组件
        self.config_manager = TaskConfigManager()
        self.nav_service = BaseNavigationService()
        self.point_navigator = SinglePointNavigator(self.nav_service)
        self.gm65_scanner = GM65Scanner()  # 添加GM65扫码器
        self.arm_controller = ArmController()  # 添加机械臂控制器
        
        # 初始化roslaunch控制变量
        self.move_base_process = None
        self.nav_is_running = False
        
        # 获取任务序列
        self.task_sequence = self.config_manager.get_task_sequence(tasktype)
        
        rospy.loginfo("主控制器初始化完成")
    
    def start_nav(self):
        """启动move_base导航节点"""
        try:
            if self.nav_is_running:
                rospy.logwarn("🔄 move_base已经在运行中，跳过启动")
                return True
            
            rospy.loginfo("🚀 开始启动move_base导航节点...")
            rospy.loginfo(f"📋 启动命令: roslaunch auto_nav test_bspline_follow_old.launch")
            
            # 使用更简单的方式启动roslaunch
            import subprocess
            import os
            
            # 设置环境变量
            env = os.environ.copy()
            rospy.loginfo("🔧 设置环境变量...")
            rospy.loginfo(f"   ROS_MASTER_URI: {env.get('ROS_MASTER_URI', '未设置')}")
            rospy.loginfo(f"   ROS_PACKAGE_PATH: {env.get('ROS_PACKAGE_PATH', '未设置')[:100]}...")
            
            # 启动roslaunch进程
            launch_cmd = ['roslaunch', 'auto_nav', 'test_bspline_follow_old.launch']
            rospy.loginfo(f"⚡ 执行启动命令: {' '.join(launch_cmd)}")
            
            self.move_base_process = subprocess.Popen(
                launch_cmd,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid
            )
            
            # 检查进程是否成功启动
            if self.move_base_process.poll() is None:
                rospy.loginfo(f"✅ 进程启动成功，PID: {self.move_base_process.pid}")
                self.nav_is_running = True
                rospy.loginfo("⏳ 等待节点完全启动...")
                rospy.sleep(3.0)  # 等待节点完全启动
                rospy.loginfo("🎉 move_base导航节点启动完成！")
                return True
            else:
                # 进程立即退出，说明启动失败
                stdout, stderr = self.move_base_process.communicate()
                rospy.logerr(f"❌ 进程启动失败，立即退出")
                rospy.logerr(f"   返回码: {self.move_base_process.returncode}")
                if stdout:
                    rospy.logerr(f"   标准输出: {stdout.decode('utf-8', errors='ignore')}")
                if stderr:
                    rospy.logerr(f"   错误输出: {stderr.decode('utf-8', errors='ignore')}")
                self.nav_is_running = False
                return False
            
        except FileNotFoundError as e:
            rospy.logerr(f"❌ 启动失败: 找不到roslaunch命令")
            rospy.logerr(f"   错误详情: {e}")
            rospy.logerr(f"   请确保ROS环境已正确设置")
            self.nav_is_running = False
            return False
        except PermissionError as e:
            rospy.logerr(f"❌ 启动失败: 权限不足")
            rospy.logerr(f"   错误详情: {e}")
            self.nav_is_running = False
            return False
        except Exception as e:
            rospy.logerr(f"❌ 启动move_base失败: {e}")
            rospy.logerr(f"   错误类型: {type(e).__name__}")
            self.nav_is_running = False
            return False
    
    def stop_nav(self):
        """停止move_base导航节点"""
        try:
            if not self.nav_is_running:
                rospy.logwarn("🔄 move_base未在运行，跳过停止")
                return True
            
            rospy.loginfo("🛑 开始停止move_base导航节点...")
            
            if hasattr(self, 'move_base_process') and self.move_base_process is not None:
                import os
                import signal
                
                rospy.loginfo(f"📋 进程信息: PID={self.move_base_process.pid}")
                
                # 检查进程是否还在运行
                if self.move_base_process.poll() is None:
                    rospy.loginfo("⚡ 进程仍在运行，开始终止...")
                    
                    try:
                        # 获取进程组ID
                        pgid = os.getpgid(self.move_base_process.pid)
                        rospy.loginfo(f"🔧 进程组ID: {pgid}")
                        
                        # 发送SIGTERM信号终止进程组
                        rospy.loginfo("📤 发送SIGTERM信号...")
                        os.killpg(pgid, signal.SIGTERM)
                        
                        # 等待进程结束
                        rospy.loginfo("⏳ 等待进程正常退出...")
                        try:
                            return_code = self.move_base_process.wait(timeout=5)
                            rospy.loginfo(f"✅ 进程正常退出，返回码: {return_code}")
                        except subprocess.TimeoutExpired:
                            rospy.logwarn("⚠️ 进程未在5秒内退出，发送SIGKILL强制终止...")
                            try:
                                os.killpg(pgid, signal.SIGKILL)
                                return_code = self.move_base_process.wait(timeout=2)
                                rospy.loginfo(f"✅ 进程强制终止，返回码: {return_code}")
                            except subprocess.TimeoutExpired:
                                rospy.logerr("❌ 进程无法终止，可能存在僵尸进程")
                                return False
                            except ProcessLookupError:
                                rospy.loginfo("✅ 进程已被终止")
                        
                    except ProcessLookupError as e:
                        rospy.logwarn(f"⚠️ 进程已不存在: {e}")
                    except PermissionError as e:
                        rospy.logerr(f"❌ 权限不足，无法终止进程: {e}")
                        return False
                else:
                    # 进程已经结束
                    return_code = self.move_base_process.returncode
                    rospy.loginfo(f"ℹ️ 进程已结束，返回码: {return_code}")
                
                # 清理进程对象
                self.move_base_process = None
                rospy.loginfo("🧹 进程对象已清理")
            else:
                rospy.logwarn("⚠️ 进程对象不存在或为None")
            
            self.nav_is_running = False
            rospy.loginfo("🎉 move_base导航节点停止完成！")
            rospy.sleep(1.0)  # 等待节点完全停止
            return True
            
        except Exception as e:
            rospy.logerr(f"❌ 停止move_base失败: {e}")
            rospy.logerr(f"   错误类型: {type(e).__name__}")
            self.nav_is_running = False
            return False
    
    def scan_qr_code(self):
        """执行GM65扫码"""
        return self.gm65_scanner.scan()
    
    def publish_move_base_goal(self, point_id, target_expect=None):
        """最简单的发布move_base goal函数"""
        rospy.loginfo(f"🚀 发布move_base指令到目标点{point_id}")
        
        # 获取目标位置
        position = self.config_manager.navigation_goals[point_id]['position']
        name = self.config_manager.navigation_goals[point_id]['name']
        
        # 确定expected_count
        if target_expect is not None:
            expected_count = target_expect
        else:
            # 使用默认逻辑（initial_count + 1）
            initial_count = self.nav_service.completed_targets
            expected_count = initial_count + 1
        
        rospy.loginfo(f"🎯 期待target_done值: {expected_count}")
        
        # 创建AdaptiveNavigationState并执行
        adaptive_nav = AdaptiveNavigationState(
            position, 
            f"{name}_粗略导航",
            parent_navigator=self.nav_service,
            expected_count=expected_count
        )
        
        success = adaptive_nav.execute()
        if success:
            rospy.loginfo(f"✅ 成功到达目标点{point_id}")
        else:
            rospy.logerr(f"❌ 导航到目标点{point_id}失败")
        
        return success
    
    def wait_for_target_done(self, expected_value):
        """等待target_done达到期望值"""
        rospy.loginfo(f"⏳ 等待target_done信号({expected_value})...")
        
        timeout_counter = 0
        max_timeout = 50  # 5秒超时 (50 * 0.1s)
        
        while self.nav_service.completed_targets < expected_value and timeout_counter < max_timeout:
            rospy.sleep(0.1)
            timeout_counter += 1
        
        if timeout_counter >= max_timeout:
            rospy.logwarn(f"⚠️ 等待target_done超时，期望值: {expected_value}")
            return False
        else:
            rospy.loginfo(f"✅ 收到target_done信号({expected_value})")
            return True
    
    def cancel_move_base_goal(self):
        """取消当前的move_base目标"""
        rospy.loginfo("🛑 取消当前move_base目标...")
        
        # 创建一个临时的AdaptiveNavigationState来获取move_base客户端
        temp_nav = AdaptiveNavigationState([0, 0], "临时导航", self.nav_service)
        move_base_client = temp_nav.move_base_client
        
        if move_base_client is None:
            rospy.logwarn("⚠️ move_base客户端未初始化")
            return False
        
        try:
            # 取消当前目标
            move_base_client.cancel_goal()
            rospy.loginfo("✅ 已取消move_base目标")
            return True
        except Exception as e:
            rospy.logerr(f"❌ 取消move_base目标失败: {e}")
            return False
    
    def execute_imu_recovery(self, point_id):
        """执行IMU恢复"""
        rospy.loginfo(f"🧭 开始执行目标点{point_id}的IMU恢复...")
        
        point_config = self.config_manager.get_point_config(point_id)
        success = self.point_navigator._execute_imu_recovery(point_config)
        
        if success:
            rospy.loginfo(f"✅ 目标点{point_id}的IMU恢复完成")
        else:
            rospy.logerr(f"❌ 目标点{point_id}的IMU恢复失败")
        
        return success
    
    def execute_precise_positioning(self, point_id, directions):
        """执行精确定位"""
        rospy.loginfo(f"🎯 开始执行目标点{point_id}的精确定位，方向: {directions}")
        
        point_config = self.config_manager.get_point_config(point_id)
        point_config.acc_loc[1] = directions  # 设置精确定位方向
        
        success = self.point_navigator._execute_precise_positioning(point_config)
        
        if success:
            rospy.loginfo(f"✅ 目标点{point_id}的精确定位完成")
        else:
            rospy.logerr(f"❌ 目标点{point_id}的精确定位失败")
        
        return success
    
    def cleanup(self):
        """清理资源"""
        rospy.loginfo("🧹 开始清理资源...")
        
        # 停止move_base
        self.stop_nav()
        
        # 关闭机械臂串口
        if hasattr(self, 'arm_controller'):
            self.arm_controller.close()
        
        rospy.loginfo("✅ 资源清理完成")


def maintest():
    """最简化的main函数，避免嵌套，完全拆开goto_point_with_arm_task的逻辑"""
    rospy.loginfo(f"\n{'='*80}")
    rospy.loginfo("🚀 开始执行简化版导航任务")
    rospy.loginfo(f"{'='*80}")
    
    try:
        # 创建主控制器
        controller = MainController(tasktype=1)
        
        # 1. 启动move_base
        rospy.loginfo("🚀 启动move_base导航节点...")
        if not controller.start_nav():
            rospy.logerr("❌ 启动move_base失败，终止程序")
            return
        
        # 2. 发布move_base指令到地点1
        rospy.loginfo("🎯 步骤1: 导航到目标点1")
        success = controller.publish_move_base_goal(1, target_expect=1)
        if not success:
            rospy.logerr("❌ 导航到点1失败，终止任务")
            return
        
        # 3. 获得扫码结果
        rospy.loginfo("📱 步骤2: 开始GM65扫码...")
        scan_result = controller.scan_qr_code()
        
        if scan_result == 1:
            rospy.loginfo("🎯 扫码结果=1，执行序列：2→3→4")
            
            # 4. 发布move_base指令到地点2
            rospy.loginfo("🎯 步骤3: 导航到目标点2")
            success = controller.publish_move_base_goal(2, target_expect=2)
            if not success:
                rospy.logerr("❌ 导航到点2失败，终止任务")
                return
            
            # 5. 等待target_done结果为2
            rospy.loginfo("⏳ 步骤4: 等待target_done信号(2)")
            if not controller.wait_for_target_done(2):
                rospy.logwarn("⚠️ 等待target_done超时，继续执行")
            
            # 6. 关闭move_base
            rospy.loginfo("🛑 步骤5: 停止move_base")
            controller.stop_nav()
            
            # 7. 确认关闭
            rospy.loginfo("⏳ 步骤6: 确认move_base完全停止")
            rospy.sleep(2.0)
            
            # 8. 进行第二阶段（IMU恢复）
            rospy.loginfo("🧭 步骤7: 执行IMU恢复")
            if not controller.execute_imu_recovery(2):
                rospy.logwarn("⚠️ IMU恢复失败，继续执行")
            
            # 9. 进行第三阶段（精确定位）
            rospy.loginfo("🎯 步骤8: 执行精确定位")
            if not controller.execute_precise_positioning(2, ['left', 'front']):
                rospy.logwarn("⚠️ 精确定位失败，继续执行")
            
            # 10. 发布机械臂指令
            rospy.loginfo("🤖 步骤9: 发送机械臂任务1")
            arm_success, response = controller.arm_controller.send_task_command(1)
            if not arm_success:
                rospy.logerr(f"❌ 机械臂任务1执行失败: {response}，终止任务")
                return
            rospy.loginfo(f"✅ 机械臂任务1执行成功: {response}")
            # 发布arm_done信号
            controller.nav_service.increment_arm_done()
            # 发布arm_done信号
            controller.nav_service.increment_arm_done()
            
            # 11. 确认机械臂任务完成后启动move_base
            rospy.loginfo("🚀 步骤10: 重新启动move_base")
            if not controller.start_nav():
                rospy.logerr("❌ 启动move_base失败，终止任务")
                return
            rospy.loginfo("✅ move_base启动成功")
            
            # 重置target_done计数，因为重新启动了move_base
            rospy.loginfo("🔄 重置target_done计数")
            controller.nav_service.completed_targets = 0
            
            # 12. 发布move_base命令至地点3
            rospy.loginfo("🎯 步骤11: 导航到目标点3")
            success = controller.publish_move_base_goal(3, target_expect=1)
            if not success:
                rospy.logerr("❌ 导航到点3失败，终止任务")
                return
            rospy.loginfo("✅ 点3导航目标发布成功")
            
            # 13. 等待target_done结果为1
            rospy.loginfo("⏳ 步骤12: 等待target_done信号(1)")
            if not controller.wait_for_target_done(1):
                rospy.logerr("❌ 等待target_done超时，终止任务")
                return
            rospy.loginfo("✅ 收到target_done信号(1)")
            
            # 14. 关闭move_base
            rospy.loginfo("🛑 步骤13: 停止move_base")
            controller.stop_nav()
            
            # 15. 确认关闭
            rospy.loginfo("⏳ 步骤14: 确认move_base完全停止")
            rospy.sleep(2.0)
            
            # 16. 进行第二阶段（IMU恢复）
            rospy.loginfo("🧭 步骤15: 执行IMU恢复")
            if not controller.execute_imu_recovery(3):
                rospy.logwarn("⚠️ IMU恢复失败，继续执行")
            
            # 17. 进行第三阶段（精确定位）
            rospy.loginfo("🎯 步骤16: 执行精确定位")
            if not controller.execute_precise_positioning(3, ['right', 'front']):
                rospy.logwarn("⚠️ 精确定位失败，继续执行")
            
            # 18. 发布机械臂指令
            rospy.loginfo("🤖 步骤17: 发送机械臂任务3")
            arm_success, response = controller.arm_controller.send_task_command(3)
            if arm_success:
                rospy.loginfo(f"✅ 机械臂任务3执行成功: {response}")
            else:
                rospy.logwarn(f"⚠️ 机械臂任务3执行失败: {response}")
            
            # 19. 收到回调后启动move_base
            rospy.loginfo("🚀 步骤18: 重新启动move_base")
            controller.start_nav()
            
            # 20. 导航至4
            rospy.loginfo("🎯 步骤19: 导航到目标点4")
            success = controller.publish_move_base_goal(4, target_expect=1)
            if success:
                rospy.loginfo("✅ 成功到达目标点4")
            else:
                rospy.logwarn("⚠️ 导航到点4失败")
                
        elif scan_result == 3:
            rospy.loginfo("🎯 扫码结果=3，执行序列：3→2→4")
            
            # 序列：3→2→4 (类似上面的逻辑，但是先到3，再到2，最后到4)
            # 为了简化，这里先实现基本框架
            
            # 导航到点3
            rospy.loginfo("🎯 步骤3: 导航到目标点3")
            success = controller.publish_move_base_goal(3, target_expect=2)
            if not success:
                rospy.logerr("❌ 导航到点3失败，终止任务")
                return
            
            # 等待target_done结果为2
            rospy.loginfo("⏳ 步骤4: 等待target_done信号(2)")
            if not controller.wait_for_target_done(2):
                rospy.logwarn("⚠️ 等待target_done超时，继续执行")
            
            # 停止move_base并执行精确定位
            controller.stop_nav()
            rospy.sleep(2.0)
            controller.execute_imu_recovery(3)
            controller.execute_precise_positioning(3, ['right', 'front'])
            
            # 机械臂任务3
            rospy.loginfo("🤖 发送机械臂任务3")
            arm_success, response = controller.arm_controller.send_task_command(3)
            if not arm_success:
                rospy.logerr(f"❌ 机械臂任务3执行失败: {response}，终止任务")
                return
            rospy.loginfo(f"✅ 机械臂任务3执行成功: {response}")
            # 发布arm_done信号
            controller.nav_service.increment_arm_done()
            # 发布arm_done信号
            controller.nav_service.increment_arm_done()
            
            # 确认机械臂任务完成后启动move_base
            rospy.loginfo("🚀 重新启动move_base")
            if not controller.start_nav():
                rospy.logerr("❌ 启动move_base失败，终止任务")
                return
            rospy.loginfo("✅ move_base启动成功")
            
            # 重置target_done计数，因为重新启动了move_base
            rospy.loginfo("🔄 重置target_done计数")
            controller.nav_service.completed_targets = 0
            
            # 导航到点2
            rospy.loginfo("🎯 导航到目标点2")
            success = controller.publish_move_base_goal(2, target_expect=1)
            if not success:
                rospy.logerr("❌ 导航到点2失败，终止任务")
                return
            rospy.loginfo("✅ 点2导航目标发布成功")
            
            # 等待target_done结果为1
            rospy.loginfo("⏳ 等待target_done信号(1)")
            if not controller.wait_for_target_done(1):
                rospy.logerr("❌ 等待target_done超时，终止任务")
                return
            rospy.loginfo("✅ 收到target_done信号(1)")
            
            # 停止move_base并执行精确定位
            controller.stop_nav()
            rospy.sleep(2.0)
            controller.execute_imu_recovery(2)
            controller.execute_precise_positioning(2, ['left', 'front'])
            
            # 机械臂任务1
            arm_success, response = controller.arm_controller.send_task_command(1)
            if arm_success:
                rospy.loginfo(f"✅ 机械臂任务1执行成功: {response}")
            
            # 重新启动并导航到点4
            controller.start_nav()
            rospy.loginfo("🎯 导航到目标点4")
            success = controller.publish_move_base_goal(4, target_expect=1)
            if success:
                rospy.loginfo("✅ 成功到达目标点4")
            else:
                rospy.logwarn("⚠️ 导航到点4失败")
        
        rospy.loginfo("🎉 简化版导航任务完成！")
        
    except rospy.ROSInterruptException:
        rospy.loginfo("接收到中断信号，正在退出...")
    except KeyboardInterrupt:
        rospy.loginfo("用户中断，正在退出...")
    except Exception as e:
        rospy.logerr(f"程序异常: {e}")
    finally:
        if 'controller' in locals():
            # 确保停止move_base
            rospy.loginfo("🛑 程序结束，停止move_base导航节点...")
            controller.stop_nav()
            controller.cleanup()
        rospy.loginfo("再见!")

def main():
    """最简化的main函数，避免嵌套，完全拆开goto_point_with_arm_task的逻辑"""
    rospy.loginfo(f"\n{'='*80}")
    rospy.loginfo("🚀 开始执行简化版导航任务")
    rospy.loginfo(f"{'='*80}")
    
    try:
        # 创建主控制器
        controller = MainController(tasktype=1)
        
        # 1. 发布move_base指令到地点1
        rospy.loginfo("🎯 步骤1: 导航到目标点1")
        success = controller.publish_move_base_goal(1, target_expect=1)
        if not success:
            rospy.logerr("❌ 导航到点1失败，终止任务")
            return
        
        # 2. 获得扫码结果
        rospy.loginfo("📱 步骤2: 开始GM65扫码...")
        scan_result = controller.scan_qr_code()
        
        if scan_result == 1:
            rospy.loginfo("🎯 扫码结果=1，执行序列：2→3→4")
            
            # 3. 发布move_base指令到地点2
            rospy.loginfo("🎯 步骤3: 导航到目标点2")
            success = controller.publish_move_base_goal(2, target_expect=2)
            if not success:
                rospy.logerr("❌ 导航到点2失败，终止任务")
                return
            
            # 4. 等待target_done结果为2
            rospy.loginfo("⏳ 步骤4: 等待target_done信号(2)")
            if not controller.wait_for_target_done(2):
                rospy.logwarn("⚠️ 等待target_done超时，继续执行")
            
            # 5. 取消move_base目标
            rospy.loginfo("🛑 步骤5: 取消move_base目标")
            controller.cancel_move_base_goal()
            
            # 6. 进行第二阶段（IMU恢复）
            rospy.loginfo("🧭 步骤6: 执行IMU恢复")
            if not controller.execute_imu_recovery(2):
                rospy.logwarn("⚠️ IMU恢复失败，继续执行")
            
            # 7. 进行第三阶段（精确定位）
            rospy.loginfo("🎯 步骤7: 执行精确定位")
            if not controller.execute_precise_positioning(2, ['left', 'front']):
                rospy.logwarn("⚠️ 精确定位失败，继续执行")
            
            # 8. 发布机械臂指令
            rospy.loginfo("🤖 步骤8: 发送机械臂任务1")
            arm_success, response = controller.arm_controller.send_task_command(1)
            if arm_success:
                rospy.loginfo(f"✅ 机械臂任务1执行成功: {response}")
                # 增加arm_done计数
                controller.nav_service.increment_arm_done()
            else:
                rospy.logwarn(f"⚠️ 机械臂任务1执行失败: {response}")
            
            # 9. 发布move_base命令至地点3
            rospy.loginfo("🎯 步骤9: 导航到目标点3")
            success = controller.publish_move_base_goal(3, target_expect=3)
            if not success:
                rospy.logerr("❌ 导航到点3失败，终止任务")
                return
            
            # 10. 等待点3的第一阶段导航完成
            rospy.loginfo("⏳ 步骤10: 等待点3第一阶段导航完成(target_done=3)")
            if not controller.wait_for_target_done(3):
                rospy.logwarn("⚠️ 等待target_done超时，继续执行")
            
            # 11. 取消move_base目标
            rospy.loginfo("🛑 步骤11: 取消move_base目标")
            controller.cancel_move_base_goal()
            
            # 12. 进行第二阶段（IMU恢复）
            rospy.loginfo("🧭 步骤12: 执行IMU恢复")
            if not controller.execute_imu_recovery(3):
                rospy.logwarn("⚠️ IMU恢复失败，继续执行")
            
            # 13. 进行第三阶段（精确定位）
            rospy.loginfo("🎯 步骤13: 执行精确定位")
            if not controller.execute_precise_positioning(3, ['right', 'front']):
                rospy.logwarn("⚠️ 精确定位失败，继续执行")
            
            # 14. 发布机械臂指令
            rospy.loginfo("🤖 步骤14: 发送机械臂任务3")
            arm_success, response = controller.arm_controller.send_task_command(3)
            if arm_success:
                rospy.loginfo(f"✅ 机械臂任务3执行成功: {response}")
                # 增加arm_done计数
                controller.nav_service.increment_arm_done()
            else:
                rospy.logwarn(f"⚠️ 机械臂任务3执行失败: {response}")
            
            # 15. 导航至4
            rospy.loginfo("🎯 步骤15: 导航到目标点4")
            success = controller.publish_move_base_goal(4, target_expect=4)
            if success:
                rospy.loginfo("✅ 成功到达目标点4")
            else:
                rospy.logwarn("⚠️ 导航到点4失败")
                
        elif scan_result == 3:
            rospy.loginfo("🎯 扫码结果=3，执行序列：3→2→4")
            
            # 3. 导航到点3
            rospy.loginfo("🎯 步骤3: 导航到目标点3")
            success = controller.publish_move_base_goal(3, target_expect=2)
            if not success:
                rospy.logerr("❌ 导航到点3失败，终止任务")
                return
            
            # 4. 等待target_done结果为2
            rospy.loginfo("⏳ 步骤4: 等待target_done信号(2)")
            if not controller.wait_for_target_done(2):
                rospy.logwarn("⚠️ 等待target_done超时，继续执行")
            
            # 5. 取消move_base目标
            rospy.loginfo("🛑 步骤5: 取消move_base目标")
            controller.cancel_move_base_goal()
            
            # 6. 进行第二阶段（IMU恢复）
            rospy.loginfo("🧭 步骤6: 执行IMU恢复")
            if not controller.execute_imu_recovery(3):
                rospy.logwarn("⚠️ IMU恢复失败，继续执行")
            
            # 7. 进行第三阶段（精确定位）
            rospy.loginfo("🎯 步骤7: 执行精确定位")
            if not controller.execute_precise_positioning(3, ['right', 'front']):
                rospy.logwarn("⚠️ 精确定位失败，继续执行")
            
            # 8. 发布机械臂指令
            rospy.loginfo("🤖 步骤8: 发送机械臂任务3")
            arm_success, response = controller.arm_controller.send_task_command(3)
            if arm_success:
                rospy.loginfo(f"✅ 机械臂任务3执行成功: {response}")
                # 增加arm_done计数
                controller.nav_service.increment_arm_done()
            else:
                rospy.logwarn(f"⚠️ 机械臂任务3执行失败: {response}")
            
            # 9. 导航到点2
            rospy.loginfo("🎯 步骤9: 导航到目标点2")
            success = controller.publish_move_base_goal(2, target_expect=3)
            if not success:
                rospy.logerr("❌ 导航到点2失败，终止任务")
                return
            
            # 10. 等待点2的第一阶段导航完成
            rospy.loginfo("⏳ 步骤10: 等待点2第一阶段导航完成(target_done=3)")
            if not controller.wait_for_target_done(3):
                rospy.logwarn("⚠️ 等待target_done超时，继续执行")
            
            # 11. 取消move_base目标
            rospy.loginfo("🛑 步骤11: 取消move_base目标")
            controller.cancel_move_base_goal()
            
            # 12. 进行第二阶段（IMU恢复）
            rospy.loginfo("🧭 步骤12: 执行IMU恢复")
            if not controller.execute_imu_recovery(2):
                rospy.logwarn("⚠️ IMU恢复失败，继续执行")
            
            # 13. 进行第三阶段（精确定位）
            rospy.loginfo("🎯 步骤13: 执行精确定位")
            if not controller.execute_precise_positioning(2, ['left', 'front']):
                rospy.logwarn("⚠️ 精确定位失败，继续执行")
            
            # 14. 发布机械臂指令
            rospy.loginfo("🤖 步骤14: 发送机械臂任务1")
            arm_success, response = controller.arm_controller.send_task_command(1)
            if arm_success:
                rospy.loginfo(f"✅ 机械臂任务1执行成功: {response}")
                # 增加arm_done计数
                controller.nav_service.increment_arm_done()
            else:
                rospy.logwarn(f"⚠️ 机械臂任务1执行失败: {response}")
            
            # 15. 导航到点4
            rospy.loginfo("🎯 步骤15: 导航到目标点4")
            success = controller.publish_move_base_goal(4, target_expect=4)
            if success:
                rospy.loginfo("✅ 成功到达目标点4")
            else:
                rospy.logwarn("⚠️ 导航到点4失败")
        
        rospy.loginfo("🎉 简化版导航任务完成！")
        
    except rospy.ROSInterruptException:
        rospy.loginfo("接收到中断信号，正在退出...")
    except KeyboardInterrupt:
        rospy.loginfo("用户中断，正在退出...")
    except Exception as e:
        rospy.logerr(f"程序异常: {e}")
    finally:
        if 'controller' in locals():
            rospy.loginfo("🛑 程序结束，清理资源...")
            controller.cleanup()
        rospy.loginfo("再见!")

if __name__ == '__main__':
    main()
