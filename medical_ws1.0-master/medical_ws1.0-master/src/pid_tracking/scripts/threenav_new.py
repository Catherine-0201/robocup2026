#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
模块化三阶段导航系统 - 重构自threestatenav.py
功能：提供灵活的点位导航调用方式，支持动态配置精确定位参数

主要特性:
1. 模块化设计：
   - BaseNavigationService: 基础导航服务
   - TaskConfigManager: 配置管理器
   - SinglePointNavigator: 单点导航器
   - MainController: 主控制器

2. 灵活的调用方式：
   - goto_point(point_id, enable_precise=True, precise_order=['left', 'front'])
   - 支持运行时动态配置每个点的精确定位策略
   - 保持与threestatenav.py完全一致的导航逻辑

3. 易于集成：
   - 清晰的模块边界，便于集成其他功能
   - 支持自定义导航策略
   - 保持原有的参数支持和错误处理
"""

from pid_node import PIDTrackingNode
import rospy
import actionlib
import math
import serial
import time
import roslaunch
import roslaunch.rlutil
import roslaunch.parent
import subprocess
from std_msgs.msg import Int32, String
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import Imu
import tf.transformations

# 从threestatenav.py导入三个状态类
from threestatenav import AdaptiveNavigationState, ImuAttitudeRecoveryState, PrecisePositionState


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
            
            # 如果还有其他方向需要定位，短暂停顿
            if priority_order.index(direction) < len(priority_order) - 1:
                rospy.sleep(0.5)
        
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
    
    def goto_point(self, point_id, enable_precise=None, precise_order=None, enable_imu_recovery=None):
        """前往指定点位 - 提供灵活的导航配置"""
        # 获取点位配置
        point_config = self.config_manager.get_point_config(point_id)
        
        # # 如果提供了自定义配置，则使用自定义配置
        # if enable_precise is not None or enable_imu_recovery is not None:
        #     point_config.set_precise_config(enable_precise, precise_order, enable_imu_recovery)
        
        # 执行导航
        success = self.point_navigator.navigate_to_point(point_config)
        
        if success:
            rospy.loginfo(f"✓ 成功到达 {point_config.name}")
        else:
            rospy.logwarn(f"✗ 导航失败 {point_config.name}")
        
        return success
    
    def _execute_lateral_precise_positioning(self, axis, target_distance, state_name):
        """
        执行横向精确定位（直接调用底层方法，避免配置污染）
        
        Args:
            axis (int): 轴方向 0=前方, 1=左侧, 2=右侧
            target_distance (float): 目标距离
            state_name (str): 状态名称
        
        Returns:
            bool: 是否成功
        """
        rospy.loginfo(f"🎯 开始横向精确定位: {state_name}")
        rospy.loginfo(f"轴{axis}, 目标距离{target_distance:.3f}m")
        
        try:
            # 直接创建PrecisePositionState对象，不通过PointConfig
            # 横向平移使用更宽松的精度要求（0.05m = 5cm）
            precise_pos = PrecisePositionState(
                self.nav_service.node,
                target_distance,
                axis,
                state_name,
                tolerance_done=0.05  # 横向平移使用5cm精度
            )
            
            # 使用与原始代码一致的循环控制逻辑
            max_position_attempts = 10000  # 使用您修改后的值
            position_attempts = 0
            
            while position_attempts < max_position_attempts and not rospy.is_shutdown():
                result = precise_pos.execute()
                
                if result == 'succeeded':
                    rospy.loginfo(f"✅ {state_name}完成")
                    return True
                elif result == 'failed':
                    rospy.logerr(f"横向精确定位失败: {state_name}")
                    return False
                elif result == 'in_progress':
                    position_attempts += 1
                else:
                    rospy.logerr(f"未知的横向精确定位结果: {result}")
                    return False
            
            rospy.logerr(f"横向精确定位超时: {state_name}")
            return False
            
        except Exception as e:
            rospy.logerr(f"横向精确定位异常: {e}")
            return False

    def publish_move_base_goal(self, point_id):
        """最简单的发布move_base goal函数"""
        rospy.loginfo(f"🚀 发布move_base指令到目标点{point_id}")
        
        # 获取目标位置
        position = self.config_manager.navigation_goals[point_id]['position']
        name = self.config_manager.navigation_goals[point_id]['name']
        
        # 创建AdaptiveNavigationState并执行
        adaptive_nav = AdaptiveNavigationState(
            position, 
            f"{name}_粗略导航",
            parent_navigator=self.nav_service
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

    def goto_point_with_arm_task(self, point_id, task_id=None, enable_precise=None, precise_order=None, enable_imu_recovery=None, scan_result=None):
        """前往指定点位，完成后执行机械臂任务"""
        rospy.loginfo(f"\n{'='*70}")
        rospy.loginfo(f"🎯 导航到目标点{point_id}")
        if task_id is not None:
            rospy.loginfo(f"🤖 完成后将执行机械臂任务: {task_id}")
        rospy.loginfo(f"{'='*70}")
        
        # 1. 执行导航
        success = self.goto_point(point_id, enable_precise, precise_order, enable_imu_recovery)
        
        # 针对点2和点3，在粗导航完成后停止move_base，避免与精确定位冲突
        if success and point_id in [2, 3] and enable_precise:
            rospy.loginfo(f"🎯 目标点{point_id}粗导航完成，等待target_done信号...")
            
            # 根据任务序列确定期望的target_done值
            if scan_result == 1:  # 序列：1→2→3→4
                if point_id == 2:
                    expected_target_done = 2  # 第一个送药点
                elif point_id == 3:
                    expected_target_done = 1  # 第二个送药点
            elif scan_result == 3:  # 序列：1→3→2→4
                if point_id == 3:
                    expected_target_done = 2  # 第一个送药点
                elif point_id == 2:
                    expected_target_done = 1  # 第二个送药点
            else:
                rospy.logwarn(f"⚠️ 未知的扫码结果: {scan_result}，跳过target_done等待")
                expected_target_done = None
            
            if expected_target_done is not None:
                # 等待target_done信号
                timeout_counter = 0
                max_timeout = 50  # 5秒超时 (50 * 0.1s)
                
                while self.nav_service.completed_targets < expected_target_done and timeout_counter < max_timeout:
                    rospy.sleep(0.1)
                    timeout_counter += 1
                
                if timeout_counter >= max_timeout:
                    rospy.logwarn(f"⚠️ 等待target_done超时，继续执行后续任务")
                else:
                    rospy.loginfo(f"✅ 收到target_done信号({expected_target_done})，停止move_base避免与精确定位冲突")
                    stop_success = self.stop_nav()
                    if stop_success:
                        rospy.loginfo("✅ move_base已完全停止，可以安全进行精确定位")
                        # 额外等待确保所有相关节点都已停止
                        rospy.sleep(2.0)
                    else:
                        rospy.logwarn("⚠️ move_base停止可能不完整，但继续执行后续任务")
        
        if success and task_id is not None:
            # 2. 发送机械臂指令
            rospy.loginfo(f"\n🤖 开始执行机械臂任务 {task_id}...")
            arm_success, response = self.arm_controller.send_task_command(task_id)
            
            if arm_success:
                rospy.loginfo(f"✅ 机械臂任务{task_id}执行成功!")
                rospy.loginfo(f"📄 机械臂回复: {response}")
                
                # 3. 机械臂任务完成后，执行横向精确定位
                # rospy.loginfo(f"\n🔄 机械臂任务完成，开始横向精确定位...")
                
                # 根据点位确定横向移动方向和距离
                # if point_id == 2:
                #     # 2点送药完成后向左移动1m
                #     original_distance = self.config_manager.get_point_config(2).get_target_distance('left')
                #     target_distance = original_distance + 0.5
                #     rospy.loginfo(f"📍 2点横向移动：左向 {original_distance:.3f}m → {target_distance:.3f}m (+1.0m)")
                #     
                #     # 直接调用底层精确定位方法，避免配置污染
                #     rospy.loginfo(f"🔄 开始执行2点左向横向移动...")
                #     lateral_success = self._execute_lateral_precise_positioning(1, target_distance, "2点左向横向移动")
                #     if lateral_success:
                #         rospy.loginfo(f"✅ 2点横向移动完成：左移1m")
                #     else:
                #         rospy.logwarn(f"⚠️ 2点横向移动失败，但不影响后续任务")
                #         
                # elif point_id == 3:
                #     # 3点送药完成后向右移动1m
                #     original_distance = self.config_manager.get_point_config(3).get_target_distance('right')
                #     target_distance = original_distance + 0.5
                #     rospy.loginfo(f"📍 3点横向移动：右向 {original_distance:.3f}m → {target_distance:.3f}m (+1.0m)")
                #     
                #     # 直接调用底层精确定位方法，避免配置污染
                #     rospy.loginfo(f"🔄 开始执行3点右向横向移动...")
                #     lateral_success = self._execute_lateral_precise_positioning(2, target_distance, "3点右向横向移动")
                #     if lateral_success:
                #         rospy.loginfo(f"✅ 3点横向移动完成：右移1m")
                #     else:
                #         rospy.logwarn(f"⚠️ 3点横向移动失败，但不影响后续任务")
                #         
                # else:
                #     rospy.loginfo(f"ℹ️ 点位{point_id}无需横向移动")
                
                # 注释掉横向移动，收到机械臂回复后直接发布去下一个目标点
                rospy.loginfo(f"ℹ️ 机械臂任务{task_id}完成，跳过横向移动，准备前往下一个目标点")
                
                # 重新启动move_base，为下一个点的导航做准备
                if point_id in [2, 3]:  # 只有点2和点3需要重新启动
                    rospy.loginfo(f"🔄 机械臂任务完成，重新启动move_base为下一个点做准备...")
                    self.start_nav()
                
            else:
                rospy.logwarn(f"❌ 机械臂任务{task_id}执行失败")
                rospy.logwarn(f"📄 错误信息: {response}")
                # 注意：机械臂任务失败不影响导航继续进行
        elif success:
            rospy.loginfo(f"✅ 导航到目标点{point_id}完成（无机械臂任务）")
                
        return success
    
    def scan_qr_code(self):
        """执行GM65扫码"""
        rospy.loginfo(f"\n{'='*60}")
        rospy.loginfo("📱 开始GM65扫码...")
        rospy.loginfo(f"{'='*60}")
        
        scan_result = self.gm65_scanner.scan()
        
        rospy.loginfo(f"\n📊 扫码结果:")
        rospy.loginfo(f"   扫码值: {scan_result}")
        rospy.loginfo(f"   任务类型: {'任务A' if scan_result == 1 else '任务B'}")
        rospy.loginfo(f"   后续路径: {'2→3→4' if scan_result == 1 else '3→2→4'}")
        
        return scan_result
    
    def run_navigation_with_qr_scan(self):
        """带GM65扫码的导航流程"""
        rospy.loginfo(f"\n{'='*80}")
        rospy.loginfo("开始执行带GM65扫码的导航任务")
        rospy.loginfo(f"{'='*80}")
        
        successful_goals = 0
        
        # 阶段1：导航到点1
        rospy.loginfo("🎯 阶段1：导航到目标点1")
        if not self.goto_point(1, enable_precise=False, enable_imu_recovery=True):
            rospy.logerr("导航到点1失败，终止任务")
            return False
        
        successful_goals += 1
        rospy.loginfo("✓ 成功到达目标点1")
        self._wait_at_target(2.0)  # 在目标点停留
        
        # 阶段2：GM65扫码
        rospy.loginfo("📱 阶段2：开始GM65扫码...")
        scan_result = self.scan_qr_code()
        
        # 阶段3：根据扫码结果决定后续导航
        if scan_result == 1:
            rospy.loginfo("🎯 扫码结果=1，执行序列：2→3→4")
            # 依次导航到2, 3, 4
            if self.goto_point(2, enable_precise=True, precise_order=['left', 'front'], enable_imu_recovery=True):
                successful_goals += 1
                self._wait_at_target(2.0)
            
            if self.goto_point(3, enable_precise=True, precise_order=['right', 'front'], enable_imu_recovery=True):
                successful_goals += 1
                self._wait_at_target(2.0)
            
            if self.goto_point(4, enable_precise=False, enable_imu_recovery=False):
                successful_goals += 1
                
        elif scan_result == 3:
            rospy.loginfo("🎯 扫码结果=3，执行序列：3→2→4")
            # 依次导航到3, 2, 4
            if self.goto_point(3, enable_precise=True, precise_order=['right', 'front'], enable_imu_recovery=True):
                successful_goals += 1
                self._wait_at_target(2.0)
            
            if self.goto_point(2, enable_precise=True, precise_order=['left', 'front'], enable_imu_recovery=True):
                successful_goals += 1
                self._wait_at_target(2.0)
            
            if self.goto_point(4, enable_precise=False, enable_imu_recovery=False):
                successful_goals += 1
        
        # 任务总结
        total_goals = 4
        self._print_task_summary(successful_goals, total_goals, scan_result)
        
        rospy.loginfo("🎉 带GM65扫码的导航任务完成！")
        return successful_goals == total_goals
    
    def run_predefined_sequence(self):
        """运行预定义的任务序列"""
        rospy.loginfo(f"\n{'='*80}")
        rospy.loginfo(f"开始执行预定义任务序列 (tasktype={self.tasktype})")
        
        goal_names = [self.config_manager.navigation_goals[gid]['name'] for gid in self.task_sequence]
        rospy.loginfo(f"导航序列: {' -> '.join(goal_names)}")
        rospy.loginfo(f"{'='*80}")
        
        successful_goals = 0
        total_goals = len(self.task_sequence)
        
        # 根据tasktype执行不同的序列
        if self.tasktype == 1:
            successful_goals += self._execute_tasktype_1()
        elif self.tasktype == 3:
            successful_goals += self._execute_tasktype_3()
        else:
            rospy.logwarn(f"未知任务类型: {self.tasktype}，执行默认序列")
            successful_goals += self._execute_tasktype_1()
        
        # 任务总结
        self._print_task_summary(successful_goals, total_goals)
        
        return successful_goals == total_goals
    
    def _execute_tasktype_1(self):
        """执行任务类型1: 1 -> 2 -> 3 -> 4"""
        successful_goals = 0
        
        # 前往点1 - 不需要精确定位，需要IMU恢复
        if self.goto_point(1, enable_precise=False, enable_imu_recovery=True):
            successful_goals += 1
            self._wait_at_target(2.0)
        
        # 前往点2 - 需要精确定位，按左侧->前方顺序，需要IMU恢复
        if self.goto_point(2, enable_precise=True, precise_order=['left', 'front'], enable_imu_recovery=True):
            successful_goals += 1
            self._wait_at_target(2.0)
        
        # 前往点3 - 需要精确定位，按右侧->前方顺序，需要IMU恢复
        if self.goto_point(3, enable_precise=True, precise_order=['right', 'front'], enable_imu_recovery=True):
            successful_goals += 1
            self._wait_at_target(2.0)
        
        # 前往点4 - 不需要精确定位，不需要IMU恢复
        if self.goto_point(4, enable_precise=False, enable_imu_recovery=False):
            successful_goals += 1
        
        return successful_goals
    
    def _execute_tasktype_3(self):
        """执行任务类型3: 1 -> 3 -> 2 -> 4"""
        successful_goals = 0
        
        # 前往点1 - 不需要精确定位，需要IMU恢复
        if self.goto_point(1, enable_precise=False, enable_imu_recovery=True):
            successful_goals += 1
            self._wait_at_target(2.0)
        
        # 前往点3 - 需要精确定位，按右侧->前方顺序，需要IMU恢复
        if self.goto_point(3, enable_precise=True, precise_order=['right', 'front'], enable_imu_recovery=True):
            successful_goals += 1
            self._wait_at_target(2.0)
        
        # 前往点2 - 需要精确定位，按左侧->前方顺序，需要IMU恢复
        if self.goto_point(2, enable_precise=True, precise_order=['left', 'front'], enable_imu_recovery=True):
            successful_goals += 1
            self._wait_at_target(2.0)
        
        # 前往点4 - 不需要精确定位，不需要IMU恢复
        if self.goto_point(4, enable_precise=False, enable_imu_recovery=False):
            successful_goals += 1
        
        return successful_goals
    
    def _wait_at_target(self, duration):
        """在目标点停留指定时间"""
        if duration > 0:
            rospy.loginfo(f"在目标点停留{duration}秒...")
            rospy.sleep(duration)
    
    def _print_task_summary(self, successful_goals, total_goals, scan_result=None):
        """打印任务总结"""
        rospy.loginfo(f"\n{'='*80}")
        rospy.loginfo("模块化三阶段导航任务完成总结")
        rospy.loginfo("="*80)
        rospy.loginfo(f"任务类型: {self.tasktype}")
        
        if scan_result is not None:
            rospy.loginfo(f"📱 GM65扫码结果: {scan_result}")
            rospy.loginfo(f"🎯 实际执行任务类型: {'任务A' if scan_result == 1 else '任务B'}")
            rospy.loginfo(f"📋 执行路径: {'1→2→3→4' if scan_result == 1 else '1→3→2→4'}")
        else:
            goal_names = [self.config_manager.navigation_goals[gid]['name'] for gid in self.task_sequence]
            rospy.loginfo(f"导航序列: {' -> '.join(goal_names)}")
        
        rospy.loginfo(f"总目标点数: {total_goals}")
        rospy.loginfo(f"成功完成: {successful_goals}")
        rospy.loginfo(f"失败次数: {total_goals - successful_goals}")
        rospy.loginfo(f"成功率: {(successful_goals/total_goals*100):.1f}%")
        
        if successful_goals == total_goals:
            rospy.loginfo("🎉 恭喜！所有目标点都成功完成三阶段导航!")
            if scan_result is not None:
                rospy.loginfo("📱 GM65扫码功能正常工作!")
        else:
            rospy.logwarn(f"⚠️  有 {total_goals - successful_goals} 个目标点未能完成")
        
        rospy.loginfo("="*80)
    
    def print_system_info(self):
        """打印系统信息"""
        rospy.loginfo("\n" + "="*80)
        rospy.loginfo("模块化三阶段导航系统准备就绪")
        rospy.loginfo("="*80)
        rospy.loginfo(f"任务类型: {self.tasktype}")
        
        goal_names = [self.config_manager.navigation_goals[gid]['name'] for gid in self.task_sequence]
        rospy.loginfo(f"导航序列: {' -> '.join(goal_names)}")
        rospy.loginfo("")
        rospy.loginfo("系统架构:")
        rospy.loginfo("  📦 TaskConfigManager: 配置管理器")
        rospy.loginfo("  🛠️  BaseNavigationService: 基础导航服务")
        rospy.loginfo("  🎯 SinglePointNavigator: 单点导航器")
        rospy.loginfo("  📱 GM65Scanner: 二维码扫描器")
        rospy.loginfo("  🤖 ArmController: 机械臂控制器")
        rospy.loginfo("  🎮 MainController: 主控制器")
        rospy.loginfo("")
        rospy.loginfo("导航策略:")
        rospy.loginfo("  阶段1: 使用move_base进行自适应导航到大致位置")
        rospy.loginfo("  阶段2: 使用IMU进行姿态恢复到初始yaw角度")
        rospy.loginfo("  阶段3: 使用PID控制进行精确距离定位")
        rospy.loginfo("")
        rospy.loginfo("灵活调用方式:")
        rospy.loginfo("  goto_point(1, enable_precise=False, enable_imu_recovery=True)")
        rospy.loginfo("  goto_point(2, enable_precise=True, precise_order=['left', 'front'], enable_imu_recovery=True)")
        rospy.loginfo("  goto_point(3, enable_precise=True, precise_order=['right'], enable_imu_recovery=False)")
        rospy.loginfo("  goto_point(4, enable_precise=False, enable_imu_recovery=False)")
        rospy.loginfo("")
        rospy.loginfo("🆕 新增功能:")
        rospy.loginfo("  📱 GM65扫码: 目标点1后自动扫码，30秒超时保护")
        rospy.loginfo("  🔄 动态路径: 扫码结果1→2,3,4 | 扫码结果3→3,2,4")
        rospy.loginfo("  🤖 机械臂通信: 目标点2发送任务1，目标点3发送任务3")
        rospy.loginfo("  📡 串口通信: /dev/arm端口，115200波特率，超时保护")
        rospy.loginfo("  🧭 IMU恢复控制:")
        rospy.loginfo("    enable_imu_recovery=True:  执行阶段2 IMU姿态恢复")
        rospy.loginfo("    enable_imu_recovery=False: 跳过阶段2，直接进入精确定位或完成")
        rospy.loginfo("")
        rospy.loginfo("扫码导航模式:")
        rospy.loginfo("  controller.run_navigation_with_qr_scan()  # 带扫码的完整导航")
        rospy.loginfo("  controller.run_predefined_sequence()     # 原有的预定义序列")
        rospy.loginfo("="*80)
    
    def emergency_stop(self):
        """紧急停止"""
        self.nav_service.emergency_stop()
    
    def cleanup(self):
        """清理资源"""
        rospy.loginfo("🧹 清理系统资源...")
        
        # 停止move_base导航节点
        try:
            self.stop_nav()
        except Exception as e:
            rospy.logerr(f"停止move_base时发生异常: {e}")
        
        # 清理机械臂控制器
        try:
            self.arm_controller.close()
        except Exception as e:
            rospy.logerr(f"清理机械臂控制器时发生异常: {e}")
        
        # 停止导航服务
        try:
            self.nav_service.emergency_stop()
        except Exception as e:
            rospy.logerr(f"停止导航服务时发生异常: {e}")
        
        rospy.loginfo("✅ 资源清理完成")


def main():
    """主函数 - 展示灵活的导航调用方式"""
    try:
        # 获取任务类型参数
        tasktype = rospy.get_param('~tasktype', 1)
        rospy.loginfo(f"启动模块化三阶段导航系统，任务类型: {tasktype}")
        
        # 创建主控制器
        controller = MainController(tasktype=tasktype)
        
        # 显示系统信息
        controller.print_system_info()
        
        # 启动move_base导航节点
        rospy.loginfo("🚀 启动move_base导航节点...")
        if not controller.start_nav():
            rospy.logerr("❌ 启动move_base失败，终止程序")
            return
        
        # 等待启动
        rospy.loginfo("3秒后开始导航...")
        for i in range(3, 0, -1):
            rospy.loginfo(f"{i}...")
            rospy.sleep(1.0)
        
        # 主导航流程：先到点1，扫码，然后根据结果分支导航
        
        # 阶段1：导航到点1
        rospy.loginfo("🎯 阶段1：导航到目标点1")
        if not controller.goto_point(1, enable_precise=False, enable_imu_recovery=True):
            rospy.logerr("导航到点1失败，终止任务")
            return
        
        rospy.loginfo("✓ 成功到达目标点1")
        # rospy.sleep(2.0)  # 在目标点停留
        
        # 阶段2：GM65扫码
        rospy.loginfo("📱 阶段2：开始GM65扫码...")
        scan_result = controller.scan_qr_code()
        
        # 阶段3：根据扫码结果决定后续导航
        if scan_result == 1:
            rospy.loginfo("🎯 扫码结果=1，执行序列：2→3→4")
            # 到达点2，发送机械臂任务1
            controller.goto_point_with_arm_task(2, task_id=1, 
                                              enable_precise=True, 
                                              precise_order=['left', 'front'], 
                                              enable_imu_recovery=True,
                                              scan_result=scan_result)
            rospy.sleep(2.0)
            
            # 到达点3，发送机械臂任务3
            controller.goto_point_with_arm_task(3, task_id=3, 
                                              enable_precise=True, 
                                              precise_order=['right', 'front'], 
                                              enable_imu_recovery=True,
                                              scan_result=scan_result)
            rospy.sleep(2.0)
            
            # 到达点4，无机械臂任务
            controller.goto_point_with_arm_task(4, task_id=None, 
                                              enable_precise=False, 
                                              enable_imu_recovery=False)
            
        elif scan_result == 3:
            rospy.loginfo("🎯 扫码结果=3，执行序列：3→2→4")
            # 到达点3，发送机械臂任务3
            controller.goto_point_with_arm_task(3, task_id=3, 
                                              enable_precise=True, 
                                              precise_order=['right', 'front'], 
                                              enable_imu_recovery=True,
                                              scan_result=scan_result)
            rospy.sleep(5.0)
            
            # 到达点2，发送机械臂任务1
            controller.goto_point_with_arm_task(2, task_id=1, 
                                              enable_precise=True, 
                                              precise_order=['left', 'front'], 
                                              enable_imu_recovery=True,
                                              scan_result=scan_result)
            rospy.sleep(5.0)
            
            # 到达点4，无机械臂任务
            controller.goto_point_with_arm_task(4, task_id=None, 
                                              enable_precise=False, 
                                              enable_imu_recovery=False)
        
        rospy.loginfo("🎉 带GM65扫码的导航任务完成！")
        rospy.loginfo("模块化三阶段导航程序结束")
        
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
            controller.cleanup()  # 使用新的清理方法
        rospy.loginfo("再见!")
def testlaunch():
    """测试launch启动和停止功能"""
    rospy.loginfo("🧪 开始测试launch启动和停止功能...")
    
    # 创建主控制器
    controller = MainController(tasktype=1)
    
    # 测试启动launch
    rospy.loginfo("🚀 测试1: 启动launch...")
    start_success = controller.start_nav()
    rospy.loginfo(f"📊 启动结果: {'✅ 成功' if start_success else '❌ 失败'}")
    
    if start_success:
        # 保持5秒
        rospy.loginfo("⏳ 保持运行5秒...")
        for i in range(5, 0, -1):
            rospy.loginfo(f"   剩余 {i} 秒...")
            rospy.sleep(1.0)
        
        # 测试停止launch
        rospy.loginfo("🛑 测试2: 停止launch...")
        stop_success = controller.stop_nav()
        rospy.loginfo(f"📊 停止结果: {'✅ 成功' if stop_success else '❌ 失败'}")
        
        # 总结测试结果
        rospy.loginfo("📋 测试总结:")
        rospy.loginfo(f"   启动launch: {'✅ 成功' if start_success else '❌ 失败'}")
        rospy.loginfo(f"   停止launch: {'✅ 成功' if stop_success else '❌ 失败'}")
        
        if start_success and stop_success:
            rospy.loginfo("🎉 所有测试通过！launch功能正常")
        else:
            rospy.logwarn("⚠️ 部分测试失败，请检查launch功能")
    else:
        rospy.logerr("❌ 启动失败，无法进行停止测试")
    
    # 清理
    controller.cleanup()
    rospy.loginfo("🧹 测试完成，已清理资源")

def test():
    # 获取任务类型参数
    tasktype = rospy.get_param('~tasktype', 1)
    rospy.loginfo(f"启动模块化三阶段导航系统，任务类型: {tasktype}")
    
    # 创建主控制器
    controller = MainController(tasktype=tasktype)
    point_config = controller.config_manager.get_point_config(3)
    controller.point_navigator.navigate_to_point(point_config)
    # controller.goto_point(3, enable_precise=True,precise_order=['right', 'front'],enable_imu_recovery=True)
    controller.goto_point_with_arm_task(2, task_id=1, 
                                              enable_precise=True, 
                                              precise_order=['left', 'front'], 
                                              enable_imu_recovery=True,
                                              scan_result=1)

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
        success = controller.publish_move_base_goal(1)
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
            success = controller.publish_move_base_goal(2)
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
            if arm_success:
                rospy.loginfo(f"✅ 机械臂任务1执行成功: {response}")
            else:
                rospy.logwarn(f"⚠️ 机械臂任务1执行失败: {response}")
            
            # 11. 收到回调后启动move_base
            rospy.loginfo("🚀 步骤10: 重新启动move_base")
            controller.start_nav()
            
            # 12. 发布move_base命令至地点3
            rospy.loginfo("🎯 步骤11: 导航到目标点3")
            success = controller.publish_move_base_goal(3)
            if not success:
                rospy.logerr("❌ 导航到点3失败，终止任务")
                return
            
            # 13. 等待target_done结果为1
            rospy.loginfo("⏳ 步骤12: 等待target_done信号(1)")
            if not controller.wait_for_target_done(1):
                rospy.logwarn("⚠️ 等待target_done超时，继续执行")
            
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
            success = controller.publish_move_base_goal(4)
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
            success = controller.publish_move_base_goal(3)
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
            arm_success, response = controller.arm_controller.send_task_command(3)
            if arm_success:
                rospy.loginfo(f"✅ 机械臂任务3执行成功: {response}")
            
            # 重新启动并导航到点2
            controller.start_nav()
            rospy.loginfo("🎯 导航到目标点2")
            success = controller.publish_move_base_goal(2)
            if not success:
                rospy.logerr("❌ 导航到点2失败，终止任务")
                return
            
            # 等待target_done结果为1
            rospy.loginfo("⏳ 等待target_done信号(1)")
            if not controller.wait_for_target_done(1):
                rospy.logwarn("⚠️ 等待target_done超时，继续执行")
            
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
            success = controller.publish_move_base_goal(4)
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

if __name__ == '__main__':
    # main()
    # testlaunch()
    maintest()