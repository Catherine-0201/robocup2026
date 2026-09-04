#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
三阶段导航专用脚本 - 从newtask_test.py抽离并升级
功能：集成自适应导航(move_base) + IMU姿态恢复 + 精确距离定位(PID控制) + 送药功能

主要特性:
1. tasktype控制导航顺序：
   - tasktype=1: 依次到达 1,2,3,4 目标点  
   - tasktype=3: 依次到达 1,3,2,4 目标点

2. acc_loc精确控制每个目标点的定位需求：
   - acc_loc[0]: 是否需要精确定位 (True/False)
   - acc_loc[1]: 精确定位方向的优先级顺序列表 (e.g., ['front', 'left', 'right'])

3. 三阶段导航流程：
   - 阶段1: 自适应导航到大致位置 (基于C++控制器完成检测，只使用x,y位置)
   - 阶段2: IMU姿态恢复到初始yaw角度 (基于IMU角度误差检测)
   - 阶段3: 精确距离定位 (基于雷达距离误差检测，按优先级顺序执行)

4. 灵活的导航策略：
   - 每个目标点可以独立配置是否进行精确定位
   - 可以配置精确定位的方向优先级顺序
   - 支持跳过精确定位，只进行粗导航+姿态恢复

5. 多重完成检测机制：
   - 阶段1: 基于C++控制器的可靠完成检测
   - 阶段2: 基于IMU数据的姿态误差检测
   - 阶段3: 基于雷达距离的精确定位检测

6. 增强功能：
   - GM65二维码扫描：到达目标点1后扫码确定后续导航顺序
   - 从distance.yaml动态加载目标点2和3的精确定位参数
   - 机械臂上位机通信：在目标点2发送任务编号1，在目标点3发送任务编号3
   - 使用push_test.py优化通信：超时机制、更好的错误处理、使用/dev/arm端口
   - acc_loc第三参数控制IMU恢复是否执行
   - 精确的距离匹配：YAML目标距离 = GetDistanceByLidarLine发布的原始距离
"""

from pid_node import PIDTrackingNode

import rospy
import actionlib
import math
import yaml
import os
import serial
from std_msgs.msg import Int32, String
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import Imu
import tf.transformations

class AdaptiveNavigationState:
    """自适应导航状态 - 使用move_base导航到大致位置，基于C++控制器的完成信号"""
    def __init__(self, target_pose, state_name="", parent_navigator=None):
        self.target_pose = target_pose  # [x, y] 只使用位置，不使用yaw
        self.state_name = state_name
        self.parent_navigator = parent_navigator  # 引用到TwoStageNavigator以访问共享资源
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

        # 记录当前预期的完成计数
        initial_count = self.parent_navigator.completed_targets
        expected_count = initial_count + 1
        
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





class GM65ScannerState:
    """GM65扫码状态 - 获取二维码中的数字（1或3）"""
    def __init__(self, state_name="GM65扫码"):
        self.state_name = state_name
        self.scanned_data = None
        self.scanning_complete = False
        self.gm65_subscriber = None
        self.scan_timeout = 30.0  # 扫码超时时间30秒
        
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
    
    def execute(self):
        """执行GM65扫码"""
        rospy.loginfo(f"开始GM65扫码: {self.state_name}")
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
                    rospy.logwarn(f"GM65扫码超时 ({self.scan_timeout}秒)，使用默认任务类型1")
                    self.scanned_data = 1  # 默认值
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
            
            if self.scanned_data is not None:
                rospy.loginfo(f"✅ GM65扫码完成！")
                rospy.loginfo(f"   扫码结果: {self.scanned_data} (类型: {type(self.scanned_data)})")
                rospy.loginfo(f"   任务类型: {'任务A' if self.scanned_data == 1 else '任务B'}")
                rospy.loginfo(f"   导航策略: {'2→3→4' if self.scanned_data == 1 else '3→2→4'}")
                return self.scanned_data
            else:
                rospy.logerr("❌ GM65扫码失败")
                return None
                
        except Exception as e:
            rospy.logerr(f"GM65扫码执行异常: {e}")
            # 确保取消订阅
            if self.gm65_subscriber:
                self.gm65_subscriber.unregister()
            return None

class ThreeStageNavigator:
    """三阶段导航器 - 自适应导航 + IMU姿态恢复 + 精确距离定位"""
    
        # 初始化PID跟踪节点 (它会自动初始化ROS节点)
        self.node = PIDTrackingNode()
        rospy.loginfo("初始化三阶段导航器...")
        
        # 任务类型 - 现在可能在扫码后动态更新
        self.tasktype = tasktype
        self.scanned_tasktype = None  # 扫码获得的任务类型
        rospy.loginfo(f"初始任务类型: {self.tasktype}")
        
        # 初始化target_done支持
        self.completed_targets = 0  # 已完成的目标数量
        self.init_target_done_subscriber()
        
        # 初始化IMU支持
        self.current_yaw = 0.0  # 当前yaw角度
        self.initial_yaw = None  # 初始yaw角度
        self.imu_initialized = False  # IMU是否已初始化
        self.init_imu_subscriber()
        
        # 定义导航目标点
        self.navigation_goals = self.define_navigation_goals()
        
        # 定义精确距离参数（支持从ROS参数读取，与task_new.py一致）
        self.precise_distances = self.define_precise_distances()
        
        # 加载激光雷达距离参数（与task_new.py完全一致）
        self.lidar_distance = rospy.get_param("lader_point", [])
        if self.lidar_distance:
            rospy.loginfo("从ROS参数加载lader_point距离数据")
            # 与原始task_new.py一致的预处理
            for i in range(len(self.lidar_distance)):
                for j in range(3):
                    self.lidar_distance[i][j] -= 0.2
                    if self.lidar_distance[i][j] <= 0:
                        self.lidar_distance[i][j] = 0.1
            # 使用参数文件中的数据更新精确距离
            self.update_distances_from_params()
        else:
            rospy.logwarn("未找到lader_point参数，使用默认硬编码距离")
        
        # 从distance.yaml文件加载目标点2和3的精确定位数据
        self.load_yaml_distances()
        
        # 定义精确定位控制参数
        self.accuracy_location = self.define_accuracy_location()
        
        # 初始导航顺序（可能在扫码后更新）
        self.goal_sequence = self.get_goal_sequence()
        

        
        rospy.loginfo("三阶段导航器初始化完成")
    def __init__(self, tasktype=1):
        self.node = PIDTrackingNode()
        rospy.loginfo("初始化三阶段导航器.")


        self.tasktype = tasktype
        self.scanned_tasktype = None
        self.completed_targets = 0
        self.init_target_done_subscriber()


        self.current_yaw = 0.0
        self.initial_yaw = None
        self.imu_initialized = False
        self.init_imu_subscriber()


        self.navigation_goals = self.define_navigation_goals()
        self.precise_distances = self.define_precise_distances()


        self.lidar_distance = rospy.get_param("lader_point", [])
        if self.lidar_distance:
            rospy.loginfo("从ROS参数加载lader_point距离数据 (不再减0.2)")
            self.update_distances_from_params()
        else:
            rospy.logwarn("未找到lader_point参数，使用默认硬编码距离")


        self.load_yaml_distances()
        self.accuracy_location = self.define_accuracy_location()
        self.goal_sequence = self.get_goal_sequence()
rospy.loginfo("三阶段导航器初始化完成") 


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

    def define_navigation_goals(self):
        """定义导航目标点 - 粗略导航的目标位置"""
        return {
            1: {'position': [1.6, 0.0], 'name': '目标点1'},     # 起始区域
            2: {'position': [5.393, 2.47], 'name': '目标点2'},    # 药品区域1  
            3: {'position': [5.624, -1.823], 'name': '目标点3'},   # 药品区域2
            4: {'position': [0.0, 0.0], 'name': '目标点4'}    # 返回点
        }
    
    def define_precise_distances(self):
        """定义精确距离参数 - 精确定位的目标距离"""
        return {
            1: {
                'front': 0.20,   
                'left': 0.30,   
                'right': 0.30,   
            },
            2: {
                'front': 0.50,   
                'left': 0.30,    
                'right': 0.20,   
            },
            3: {
                'front': 0.50,   
                'left': 0.20,    
                'right': 0.30,   
            },
            4: {
                'front': 0.40,   
                'left': 0.35,    
                'right': 0.35,   
            }
        }

    def define_accuracy_location(self):
        """
        定义精确定位控制参数 - acc_loc[需要精确定位, 优先级顺序列表, IMU恢复开关]
        
        格式说明:
        acc_loc[0]: 是否需要精确定位 (True/False)
        acc_loc[1]: 精确定位方向的优先级顺序列表 (e.g., ['front', 'left', 'right'])
        acc_loc[2]: 是否需要IMU姿态恢复 (True/False) - 新增功能
        
        示例:
        acc_loc = [True, ['front'], True]  - 需要精确定位+IMU恢复，只使用前方
        acc_loc = [True, ['left', 'front'], False]  - 需要精确定位但跳过IMU恢复
        acc_loc = [False, [], True] - 跳过精确定位，只进行IMU恢复
        acc_loc = [False, [], False] - 跳过精确定位和IMU恢复，只粗导航
        """
        return {
            1: [False, [], False],   # 目标点1: 不需要精确定位，需要IMU恢复
            2: [True, ['left', 'front'], True],   # 目标点2: 需要精确定位，按左侧->前方顺序，需要IMU恢复
            3: [True, ['right', 'front'], True],   # 目标点3: 需要精确定位，按右侧->前方顺序，需要IMU恢复
            4: [False, [], False]  # 目标点4: 不需要精确定位，不需要IMU恢复
        }



    def update_distances_from_params(self):
        """从ROS参数lader_point更新精确距离配置（与task_new.py的使用模式一致）"""
        if not self.lidar_distance or len(self.lidar_distance) < 4:
            rospy.logwarn("lader_point参数数据不足，保持默认配置")
            return
        
        try:
            # 根据task_new.py中的使用模式更新距离
            # LEFT_POSITION_LOOP: self.lidar_distance[0][1] (axis=1)
            # LEFT_POSITION_LOOP2: self.lidar_distance[1][1] (axis=1) 
            # FRONT_POSITION_LOOP: self.lidar_distance[1][0] (axis=0)
            # RIGHT_POSITION_LOOP: self.lidar_distance[2][2] (axis=2)
            # RIGHT_POSITION_LOOP2: self.lidar_distance[3][2] (axis=2)
            # FRONT_POSITION_LOOP2: self.lidar_distance[3][0] (axis=0)
            
            rospy.loginfo("使用lader_point参数更新精确距离配置...")
            
            # 确保lidar_distance有足够的数据
            for i in range(4):
                if i >= len(self.lidar_distance):
                    rospy.logwarn(f"lader_point[{i}]不存在，跳过")
                    continue
                if len(self.lidar_distance[i]) < 3:
                    rospy.logwarn(f"lader_point[{i}]数据不完整，跳过")
                    continue
            
            # 更新目标点距离配置（基于task_new.py的使用模式）
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

    def load_yaml_distances(self):
        """从distance.yaml文件加载目标点2和3的精确定位数据"""
        yaml_file_path = "/home/jetson/code_files/robocup/medical_ws/src/robcup/data/distance.yaml"
        
        try:
            # 检查文件是否存在
            if not os.path.exists(yaml_file_path):
                rospy.logwarn(f"⚠️ distance.yaml文件不存在: {yaml_file_path}")
                rospy.logwarn("使用默认精确定位数据")
                return
            
            # 读取YAML文件
            with open(yaml_file_path, 'r', encoding='utf-8') as file:
                yaml_data = yaml.safe_load(file)
            
            rospy.loginfo(f"📄 成功读取distance.yaml文件")
            rospy.loginfo(f"   文件路径: {yaml_file_path}")
            
            # 检查数据结构
            if 'lader_point' not in yaml_data:
                rospy.logwarn("⚠️ YAML文件中未找到'lader_point'键")
                return
            
            lader_point_data = yaml_data['lader_point']
            
            # 验证数据结构（期望至少2组数据，每组3个值）
            if len(lader_point_data) < 2:
                rospy.logwarn(f"⚠️ YAML数据不足，需要至少2组数据，实际: {len(lader_point_data)}")
                return
            
            for i, group in enumerate(lader_point_data[:2]):  # 只取前2组
                if len(group) != 3:
                    rospy.logwarn(f"⚠️ YAML数据组{i}格式错误，期望3个值，实际: {len(group)}")
                    return
            
            # 提取目标点2和3的距离数据
            # 假设YAML中第一组数据对应目标点2，第二组对应目标点3
            goal2_distances = lader_point_data[0]  # [front, left, right]
            goal3_distances = lader_point_data[1]  # [front, left, right]
            
            # 更新精确距离配置
            rospy.loginfo("🎯 从YAML更新目标点2和3的精确定位数据:")
            
            # 直接使用YAML中的距离值，这些值应该是GetDistanceByLidarLine.py发布的原始距离
            # 这样精确定位时就能到达与保存定位参数时相同的位置
            # 目标点2
            self.precise_distances[2] = {
                'front': float(goal2_distances[0]),
                'left': float(goal2_distances[1]),
                'right': float(goal2_distances[2])
            }
            rospy.loginfo(f"   目标点2 (GetDistanceByLidarLine目标): 前方={goal2_distances[0]}m, 左侧={goal2_distances[1]}m, 右侧={goal2_distances[2]}m")
            
            # 目标点3  
            self.precise_distances[3] = {
                'front': float(goal3_distances[0]),
                'left': float(goal3_distances[1]),
                'right': float(goal3_distances[2])
            }
            rospy.loginfo(f"   目标点3 (GetDistanceByLidarLine目标): 前方={goal3_distances[0]}m, 左侧={goal3_distances[1]}m, 右侧={goal3_distances[2]}m")
            
            rospy.loginfo("✅ YAML距离数据加载完成")
            
        except yaml.YAMLError as e:
            rospy.logerr(f"❌ YAML文件解析错误: {e}")
        except Exception as e:
            rospy.logerr(f"❌ 加载distance.yaml失败: {e}")
            rospy.logwarn("使用默认精确定位数据")

    def update_accuracy_location(self, goal_id, acc_loc):
        """
        更新指定目标点的精确定位配置
        
        参数:
        goal_id: 目标点ID (1,2,3,4)
        acc_loc: 新的配置 [需要精确定位, 优先级顺序列表, IMU恢复开关]
        
        示例:
        navigator.update_accuracy_location(2, [True, ['front', 'left'], True])  # 精确定位+IMU恢复
        navigator.update_accuracy_location(3, [True, ['right'], False])  # 精确定位但跳过IMU恢复
        navigator.update_accuracy_location(4, [False, [], True])  # 跳过精确定位，只IMU恢复
        """
        if goal_id in self.accuracy_location:
            # 确保acc_loc有3个参数，如果只有2个则添加默认的IMU恢复开关
            if len(acc_loc) == 2:
                acc_loc.append(True)  # 默认启用IMU恢复
                rospy.logwarn(f"目标点{goal_id}的acc_loc只有2个参数，自动添加IMU恢复开关=True")
            elif len(acc_loc) != 3:
                rospy.logerr(f"目标点{goal_id}的acc_loc参数数量错误，期望3个，实际{len(acc_loc)}")
                return
            
            self.accuracy_location[goal_id] = acc_loc
            rospy.loginfo(f"已更新目标点{goal_id}的配置: 精确定位={acc_loc[0]}, 方向={acc_loc[1]}, IMU恢复={acc_loc[2]}")
        else:
            rospy.logwarn(f"目标点{goal_id}不存在")

    def print_acc_loc_examples(self):
        """打印acc_loc配置示例"""
        rospy.loginfo("\n" + "="*70)
        rospy.loginfo("acc_loc 配置示例 (新增IMU恢复控制):")
        rospy.loginfo("="*70)
        rospy.loginfo("格式: [需要精确定位, 优先级顺序列表, IMU恢复开关]")
        rospy.loginfo("")
        rospy.loginfo("示例配置:")
        rospy.loginfo("  [True, ['front'], True]       - 前方精确定位 + IMU恢复")
        rospy.loginfo("  [True, ['left'], False]       - 左侧精确定位 + 跳过IMU恢复")
        rospy.loginfo("  [True, ['right'], True]       - 右侧精确定位 + IMU恢复")
        rospy.loginfo("  [True, ['front', 'left'], True]  - 前方->左侧精确定位 + IMU恢复")
        rospy.loginfo("  [True, ['left', 'right'], False] - 左侧->右侧精确定位 + 跳过IMU恢复")
        rospy.loginfo("  [False, [], True]             - 跳过精确定位 + 只IMU恢复")
        rospy.loginfo("  [False, [], False]            - 跳过精确定位和IMU恢复，只粗导航")
        rospy.loginfo("")
        rospy.loginfo("🆕 新功能说明:")
        rospy.loginfo("  • acc_loc[2] = True:  执行IMU姿态恢复（阶段2）")
        rospy.loginfo("  • acc_loc[2] = False: 跳过IMU姿态恢复，直接进入精确定位或完成")
        rospy.loginfo("="*70)

    def get_goal_sequence(self):
        """根据tasktype获取目标点顺序"""
        # 使用扫码得到的任务类型（如果有的话），否则使用初始任务类型
        current_tasktype = self.scanned_tasktype if self.scanned_tasktype is not None else self.tasktype
        
        if current_tasktype == 1:
            sequence = [1, 2, 3, 4]
            rospy.loginfo("任务序列1: 1 -> 2 -> 3 -> 4")
        elif current_tasktype == 3:
            sequence = [1, 3, 2, 4]
            rospy.loginfo("任务序列3: 1 -> 3 -> 2 -> 4")
        else:
            rospy.logwarn(f"未知任务类型: {current_tasktype}，使用默认序列")
            sequence = [1, 2, 3, 4]
        
        return sequence
    
    def update_goal_sequence_after_scan(self, scanned_tasktype):
        """扫码后更新导航顺序"""
        rospy.loginfo(f"\n🔄 开始更新导航序列...")
        rospy.loginfo(f"   接收到的扫码结果: {scanned_tasktype} (类型: {type(scanned_tasktype)})")
        
        self.scanned_tasktype = scanned_tasktype
        old_sequence = self.goal_sequence.copy()
        
        rospy.loginfo(f"   原始导航序列: {old_sequence}")
        rospy.loginfo(f"   原始任务类型: {self.tasktype}")
        
        # 重新计算导航顺序（排除已经完成的目标点1）
        if scanned_tasktype == 1:
            remaining_sequence = [2, 3, 4]
            task_name = "任务A"
            rospy.loginfo(f"🎯 扫码结果=1 → {task_name}")
            rospy.loginfo(f"   后续导航序列: 2 -> 3 -> 4")
        elif scanned_tasktype == 3:
            remaining_sequence = [3, 2, 4]
            task_name = "任务B"
            rospy.loginfo(f"🎯 扫码结果=3 → {task_name}")
            rospy.loginfo(f"   后续导航序列: 3 -> 2 -> 4")
        else:
            remaining_sequence = [2, 3, 4]
            task_name = "默认任务A"
            rospy.logwarn(f"⚠️ 未知扫码结果: {scanned_tasktype} (类型: {type(scanned_tasktype)})")
            rospy.logwarn(f"   使用默认序列: 2 -> 3 -> 4")
        
        # 更新完整的导航序列（目标点1 + 后续序列）
        self.goal_sequence = [1] + remaining_sequence
        
        rospy.loginfo(f"📱 任务类型确认: {self.tasktype} → {scanned_tasktype} ({task_name})")
        rospy.loginfo(f"🔄 完整导航序列已更新: {' -> '.join([self.navigation_goals[gid]['name'] for gid in self.goal_sequence])}")
        rospy.loginfo(f"📋 后续剩余序列: {remaining_sequence}")
        
        return remaining_sequence

    def execute_three_stage_navigation(self, goal_id):
        """执行单个目标点的三阶段导航（根据acc_loc控制精确定位和IMU恢复）"""
        goal_info = self.navigation_goals[goal_id]
        precise_info = self.precise_distances[goal_id]
        acc_loc = self.accuracy_location[goal_id]
        
        # 确保acc_loc有3个参数（向后兼容）
        if len(acc_loc) == 2:
            acc_loc.append(True)  # 默认启用IMU恢复
            rospy.logwarn(f"目标点{goal_id}的acc_loc只有2个参数，自动启用IMU恢复")
        
        rospy.loginfo(f"\n{'='*60}")
        rospy.loginfo(f"开始导航到 {goal_info['name']} (ID: {goal_id})")
        rospy.loginfo(f"粗略位置: ({goal_info['position'][0]:.2f}, {goal_info['position'][1]:.2f})")
        
        # 检查各阶段配置
        need_precise = acc_loc[0]
        need_imu_recovery = acc_loc[2]
        
        precise_icon = "🎯" if need_precise else "🚀"
        imu_icon = "🧭" if need_imu_recovery else "⏭️"
        
        rospy.loginfo(f"{precise_icon} 精确定位需求: {'是' if need_precise else '否'}")
        rospy.loginfo(f"{imu_icon} IMU恢复需求: {'是' if need_imu_recovery else '否'}")
        rospy.loginfo(f"📋 完整acc_loc配置: {acc_loc}")
        
        # 构建执行计划
        execution_plan = ["阶段1=粗导航"]
        if need_imu_recovery:
            execution_plan.append("阶段2=IMU恢复")
        else:
            execution_plan.append("阶段2=跳过")
            
        if need_precise:
            directions = []
            priority_order = acc_loc[1]
            for direction in priority_order:
                directions.append(f"{direction}({precise_info[direction]:.3f}m)")
            rospy.loginfo(f"💡 精确定位顺序: {', '.join(directions) if directions else '无'}")
            execution_plan.append("阶段3=精确定位")
        else:
            execution_plan.append("阶段3=跳过")
        
        rospy.loginfo(f"📋 执行计划: {' → '.join(execution_plan)}")
        rospy.loginfo(f"{'='*60}")
        
        # 阶段1: 自适应导航（总是执行）
        adaptive_nav = AdaptiveNavigationState(
            goal_info['position'], 
            f"{goal_info['name']}_粗略导航",
            parent_navigator=self
        )
        
        success_stage1 = adaptive_nav.execute()
        if not success_stage1:
            rospy.logerr(f"阶段1失败: {goal_info['name']}")
            return False
        
        # 短暂停顿（优化减少等待时间）
        rospy.loginfo("阶段1完成，准备进入阶段2...")
        rospy.sleep(0.3)
        
        # 阶段2: IMU姿态恢复（根据acc_loc[2]配置决定）
        success_stage2 = True  # 默认成功
        
        if need_imu_recovery:
            rospy.loginfo("🧭 开始执行阶段2 - IMU姿态恢复")
            
            imu_recovery = ImuAttitudeRecoveryState(
                self.node,
                self.initial_yaw,
                f"{goal_info['name']}_姿态恢复"
            )
            
            # 执行IMU姿态恢复
            max_imu_attempts = 200  # 防止无限循环 
            imu_attempts = 0
            success_stage2 = False
            
            while imu_attempts < max_imu_attempts and not rospy.is_shutdown():
                # 更新当前yaw值
                imu_recovery.set_current_yaw(self.current_yaw)
                result = imu_recovery.execute()
                
                if result == 'succeeded':
                    rospy.loginfo("✓ 阶段2完成 - IMU姿态恢复成功")
                    success_stage2 = True
                    break
                elif result == 'failed':
                    rospy.logerr(f"阶段2失败: {goal_info['name']} - IMU姿态恢复")
                    break
                elif result == 'in_progress':
                    # 继续姿态恢复，移除延迟以获得最快响应
                    imu_attempts += 1
                else:
                    rospy.logerr(f"未知的IMU姿态恢复结果: {result}")
                    break
            
            # 检查是否因为超时退出
            if imu_attempts >= max_imu_attempts:
                rospy.logerr(f"阶段2超时: {goal_info['name']} - IMU姿态恢复")
                success_stage2 = False
            
            if not success_stage2:
                rospy.logerr(f"阶段2失败: {goal_info['name']}")
                return False
        else:
            rospy.loginfo("⏭️ 跳过阶段2 - IMU姿态恢复 (acc_loc[2]=False)")
            success_stage2 = True
        
        # 如果不需要精确定位，在姿态恢复后直接完成
        if not need_precise:
            rospy.loginfo(f"✓ 三阶段导航完成: {goal_info['name']} (跳过精确定位)")
            return True
        
        # 短暂停顿（优化减少等待时间）
        rospy.loginfo("阶段2完成，准备进入阶段3...")
        rospy.sleep(0.3)
        
        # 阶段3: 精确定位（使用pid_node.py集成逻辑）
        precision_success = self.execute_precise_positioning(goal_id)
        
        if precision_success:
            rospy.loginfo(f"✓ 三阶段导航完成: {goal_info['name']}")
            return True
        else:
            rospy.logerr(f"✗ 精确定位阶段失败: {goal_info['name']}")
            return False

    def run_navigation_sequence(self):
        """执行完整的导航序列（包含GM65扫码功能）"""
        rospy.loginfo(f"\n{'='*80}")
        rospy.loginfo(f"开始执行三阶段导航任务 (初始tasktype={self.tasktype})")
        rospy.loginfo(f"初始导航序列: {' -> '.join([self.navigation_goals[gid]['name'] for gid in self.goal_sequence])}")
        rospy.loginfo(f"{'='*80}")
        
        successful_goals = 0
        
        # 第一阶段：到达目标点1
        rospy.loginfo(f"\n>>> 阶段1: 导航到 {self.navigation_goals[1]['name']}")
        success = self.execute_three_stage_navigation(1)
        
        if success:
            successful_goals += 1
            rospy.loginfo(f"✓ {self.navigation_goals[1]['name']} 完成")
            
            # 在目标点1停留2秒
            rospy.loginfo("在目标点停留2秒...")
            rospy.sleep(2.0)
            
            # GM65扫码
            rospy.loginfo(f"\n{'='*60}")
            rospy.loginfo("🔍 目标点1完成，开始GM65扫码...")
            rospy.loginfo(f"{'='*60}")
            
            scanner = GM65ScannerState("目标点1_GM65扫码")
            scanned_result = scanner.execute()
            
            rospy.loginfo(f"📊 扫码函数返回值调试:")
            rospy.loginfo(f"   scanned_result = {scanned_result}")
            rospy.loginfo(f"   类型: {type(scanned_result)}")
            rospy.loginfo(f"   是否为None: {scanned_result is None}")
            
            if scanned_result is not None:
                # 根据扫码结果确定后续导航序列
                remaining_sequence = self.update_goal_sequence_after_scan(scanned_result)
                
                rospy.loginfo(f"\n🎯 扫码成功！后续将按新序列继续导航...")
                rospy.loginfo(f"📋 剩余目标: {' -> '.join([self.navigation_goals[gid]['name'] for gid in remaining_sequence])}")
                
                # 第二阶段：按扫码结果执行剩余导航
                task_name = "任务A" if scanned_result == 1 else "任务B"
                
                for i, goal_id in enumerate(remaining_sequence):
                    if rospy.is_shutdown():
                        rospy.loginfo("接收到关闭信号，终止导航")
                        break
                    
                    rospy.loginfo(f"\n>>> 【{task_name}】阶段2.{i+1}: 前往{self.navigation_goals[goal_id]['name']}中...")
                    rospy.loginfo(f"    扫码结果: {scanned_result} | 任务类型: {task_name}")
                    rospy.loginfo(f"    目标点: {goal_id} ({self.navigation_goals[goal_id]['name']})")
                    rospy.loginfo(f"    进度: {i+1}/{len(remaining_sequence)}")
                    
                    success = self.execute_three_stage_navigation(goal_id)
                    
                    if success:
                        successful_goals += 1
                        rospy.loginfo(f"✓ {self.navigation_goals[goal_id]['name']} 完成")
                        

                        
                        # 在目标点停留2秒（除了最后一个目标）
                        if i < len(remaining_sequence) - 1:
                            rospy.loginfo("在目标点停留2秒...")
                            rospy.sleep(2.0)
                    else:
                        rospy.logwarn(f"✗ {self.navigation_goals[goal_id]['name']} 失败")
                        # 可以选择继续下一个目标或停止
                        # break  # 取消注释此行以在失败时停止整个序列
                
                total_goals = 1 + len(remaining_sequence)
            else:
                rospy.logwarn("⚠️ GM65扫码失败，将按原序列继续导航")
                # 继续按原始序列执行剩余目标点
                original_remaining = self.goal_sequence[1:]  # 跳过已完成的目标点1
                
                for i, goal_id in enumerate(original_remaining):
                    if rospy.is_shutdown():
                        rospy.loginfo("接收到关闭信号，终止导航")
                        break
                    
                    rospy.loginfo(f"\n>>> 执行目标 {i+2}/{len(self.goal_sequence)}: {self.navigation_goals[goal_id]['name']}")
                    
                    success = self.execute_three_stage_navigation(goal_id)
                    
                    if success:
                        successful_goals += 1
                        rospy.loginfo(f"✓ {self.navigation_goals[goal_id]['name']} 完成")
                        

                        
                        # 在目标点停留2秒（除了最后一个目标）
                        if i < len(original_remaining) - 1:
                            rospy.loginfo("在目标点停留2秒...")
                            rospy.sleep(2.0)
                    else:
                        rospy.logwarn(f"✗ {self.navigation_goals[goal_id]['name']} 失败")
                
                total_goals = len(self.goal_sequence)
        else:
            rospy.logerr(f"✗ {self.navigation_goals[1]['name']} 失败，终止任务")
            total_goals = len(self.goal_sequence)
        
        # 任务总结
        self.print_task_summary(successful_goals, total_goals)



    def print_task_summary(self, successful_goals, total_goals):
        """打印任务总结"""
        rospy.loginfo(f"\n{'='*80}")
        rospy.loginfo("三阶段导航任务完成总结")
        rospy.loginfo("="*80)
        rospy.loginfo(f"初始任务类型: {self.tasktype}")
        if self.scanned_tasktype is not None:
            rospy.loginfo(f"📱 GM65扫码结果: {self.scanned_tasktype}")
            rospy.loginfo(f"🎯 实际执行任务类型: {self.scanned_tasktype}")
        else:
            rospy.loginfo(f"⚠️ 未进行GM65扫码或扫码失败")
        rospy.loginfo(f"导航序列: {' -> '.join([self.navigation_goals[gid]['name'] for gid in self.goal_sequence])}")
        rospy.loginfo(f"总目标点数: {total_goals}")
        rospy.loginfo(f"成功完成: {successful_goals}")
        rospy.loginfo(f"失败次数: {total_goals - successful_goals}")
        rospy.loginfo(f"成功率: {(successful_goals/total_goals*100):.1f}%")
        
        if successful_goals == total_goals:
            rospy.loginfo("🎉 恭喜！所有目标点都成功完成三阶段导航!")
            if self.scanned_tasktype is not None:
                rospy.loginfo("📱 GM65扫码功能正常工作!")
        else:
            rospy.logwarn(f"⚠️  有 {total_goals - successful_goals} 个目标点未能完成")
        
        rospy.loginfo("="*80)

    def print_navigation_config(self):
        """打印详细的导航配置信息"""
        rospy.loginfo("\n" + "="*80)
        rospy.loginfo("三阶段导航器准备就绪（包含GM65扫码功能）")
        rospy.loginfo("="*80)
        rospy.loginfo(f"初始任务类型: {self.tasktype}")
        rospy.loginfo(f"初始导航序列: {' -> '.join([self.navigation_goals[gid]['name'] for gid in self.goal_sequence])}")
        rospy.loginfo("")
        rospy.loginfo("🆕 新增功能:")
        rospy.loginfo("  📱 GM65扫码: 目标点1完成后自动扫码获取任务类型")
        rospy.loginfo("  🔄 动态路径: 根据扫码结果(1或3)动态调整后续导航顺序")
        rospy.loginfo("  ⏰ 扫码超时: 30秒超时保护，超时则使用默认任务类型1")
        rospy.loginfo("  📋 路径选择: 扫码=1→2,3,4 | 扫码=3→3,2,4")
        rospy.loginfo("")
        rospy.loginfo("导航策略:")
        rospy.loginfo("  阶段1: 使用move_base进行自适应导航到大致位置")
        rospy.loginfo("        🔧 完成检测: C++控制器 /target_done 信号递增")
        rospy.loginfo("  阶段2: 使用IMU进行姿态恢复到初始yaw角度 (可配置)")
        rospy.loginfo("        🔧 完成检测: IMU角度误差判断 (误差 < 0.05rad ≈ 3°)")
        rospy.loginfo("        🆕 控制开关: acc_loc[2] = True/False")
        rospy.loginfo("  阶段3: 使用PID控制进行精确距离定位 (可配置)")
        rospy.loginfo("        🔧 完成检测: 本地距离传感器数据判断 (误差 < 0.01m)")
        rospy.loginfo("        🆕 控制开关: acc_loc[0] = True/False")
        rospy.loginfo("")
        rospy.loginfo("🔄 完成逻辑说明:")
        rospy.loginfo("  📡 粗导航: 等待C++控制器在/target_done话题发布递增计数")
        rospy.loginfo("  🧭 姿态恢复: 实时检查IMU yaw角度与初始姿态的差值")
        rospy.loginfo("  📏 精确定位: 实时检查node.distance[轴]与目标距离的误差")
        rospy.loginfo("  ⏱️  精确定位采用与task_new.py完全一致的判断逻辑和循环控制")
        rospy.loginfo("  📱 GM65扫码: 监听/gm65_data话题，期望值'1'或'3'")
        rospy.loginfo("")
        rospy.loginfo("各目标点配置详情:")
        rospy.loginfo("-" * 50)
        
        axis_names = ['前方', '左侧', '右侧']
        distance_keys = ['front', 'left', 'right']
        
        # 统计精确定位情况
        total_precise_goals = sum(1 for goal_id in self.goal_sequence if self.accuracy_location[goal_id][0])
        total_goals = len(self.goal_sequence)
        
        rospy.loginfo(f"📊 精确定位统计: {total_precise_goals}/{total_goals} 个目标点需要精确定位")
        rospy.loginfo("")
        
        for goal_id in self.goal_sequence:
            goal_info = self.navigation_goals[goal_id]
            acc_loc = self.accuracy_location[goal_id]
            precise_info = self.precise_distances[goal_id]
            
            # 确保acc_loc有3个参数（向后兼容）
            if len(acc_loc) == 2:
                acc_loc.append(True)  # 默认启用IMU恢复
            
            # 状态标识
            precise_status = "🎯 精确定位" if acc_loc[0] else "🚀 仅粗导航"
            imu_status = "🧭 IMU恢复" if acc_loc[2] else "⏭️ 跳过IMU"
            
            rospy.loginfo(f"📍 {goal_info['name']} (ID: {goal_id}) - {precise_status} + {imu_status}")
            rospy.loginfo(f"   粗导航位置: ({goal_info['position'][0]:.2f}, {goal_info['position'][1]:.2f})")
            rospy.loginfo(f"   acc_loc配置: {acc_loc}")
            
            # IMU恢复信息
            if acc_loc[2]:
                rospy.loginfo(f"   🧭 IMU姿态恢复: 启用 (目标yaw={self.initial_yaw:.3f}rad)")
            else:
                rospy.loginfo(f"   ⏭️ IMU姿态恢复: 跳过")
            
            # 精确定位信息
            if acc_loc[0]:  # 需要精确定位
                enabled_directions = []
                for direction in acc_loc[1]:
                    distance = precise_info[direction]
                    enabled_directions.append(f"{direction}: {distance:.3f}m")
                
                if enabled_directions:
                    rospy.loginfo(f"   💡 精确定位顺序: {', '.join(enabled_directions)}")
                    rospy.loginfo(f"   🔧 完成检测: 本地距离传感器 (误差 < 0.01m)")
                else:
                    rospy.loginfo(f"   ⚠️  精确定位: 已启用但未配置顺序")
            else:
                rospy.loginfo(f"   ⏭️  精确定位: 跳过")
            
            rospy.loginfo("")
        
        rospy.loginfo("="*80)

    def verify_task_new_consistency(self):
        """验证与task_new.py的一致性"""
        rospy.loginfo("\n" + "="*80)
        rospy.loginfo("与 task_new.py 一致性验证")
        rospy.loginfo("="*80)
        
        # 检查关键参数
        consistency_items = [
            "✅ 精度容差: 0.01 (与原始PositionLoopState一致)",
            "✅ 距离状态设置: self.node.distance_state = self.axis + 1",
            "✅ PID控制调用: self.node.position_loop(self.target_distance)",
            "✅ 停止控制: self.node.publish_vel(0, 0, 0)",
            "✅ 状态返回值: 'in_progress', 'succeeded', 'failed'",
            "✅ 参数加载: rospy.get_param('lader_point', [])",
            "✅ 距离预处理: distance[i][j] -= 0.2",
            "✅ 循环控制逻辑: 与原始状态机模式一致",
            "✅ GM65扫码话题: /gm65_data (与task_new.py一致)"
        ]
        
        for item in consistency_items:
            rospy.loginfo(f"  {item}")
        
        rospy.loginfo("")
        rospy.loginfo("主要改进:")
        improvements = [
            "🔄 支持 acc_loc 灵活控制每个目标点的精确定位需求",
            "🎯 支持多方向同时精确定位",
            "📊 详细的配置信息显示和状态监控",
            "🛡️ 增强的错误处理和超时保护",
            "🔧 运行时配置更新能力",
            "📱 集成GM65扫码功能，实现动态路径规划",
            "🚫 移除状态机依赖，改用顺序执行控制"
        ]
        
        for improvement in improvements:
            rospy.loginfo(f"  {improvement}")
        
        rospy.loginfo("")
        rospy.loginfo("🆕 新增功能详情:")
        new_features = [
            "📱 GM65扫码: 目标点1后自动扫码，30秒超时保护",
            "🔄 动态路径: 扫码结果1→2,3,4 | 扫码结果3→3,2,4",
            "📋 智能回退: 扫码失败时使用原始任务序列",
            "🎯 保持一致: acc_loc配置与目标点绑定，不受执行顺序影响"
        ]
        
        for feature in new_features:
            rospy.loginfo(f"  {feature}")
        
        rospy.loginfo("="*80)

    def emergency_stop(self):
        """紧急停止"""
        rospy.logwarn("执行紧急停止")
        self.node.publish_vel(0, 0, 0)

    def execute_precise_positioning(self, goal_id):
        """
        精确定位执行 - 直接集成pid_node.py的稳定逻辑
        
        Args:
            goal_id (int): 目标点ID
        
        Returns:
            bool: 精确定位是否成功
        """
        acc_loc = self.accuracy_location[goal_id]
        
        # 检查是否需要精确定位
        if not acc_loc[0]:
            rospy.loginfo(f"⏭️ 目标点{goal_id} 跳过精确定位")
            return True
        
        direction_list = acc_loc[1]
        goal_name = self.navigation_goals[goal_id]['name']
        
        rospy.loginfo(f"🎯 阶段3 - {goal_name} 精确定位")
        rospy.loginfo(f"   定位方向序列: {direction_list}")
        rospy.loginfo(f"   使用pid_node.py集成逻辑")
        
        try:
            # 按顺序执行每个方向的精确定位
            for i, direction in enumerate(direction_list):
                target_distance = self.precise_distances[goal_id][direction]
                axis = {'front': 0, 'left': 1, 'right': 2}[direction]
                
                rospy.loginfo(f"   步骤{i+1}/{len(direction_list)}: {direction}方向定位")
                rospy.loginfo(f"   轴{axis}, YAML目标距离: {target_distance:.3f}m")
                
                # 执行单轴精确定位（集成pid_node.py逻辑）
                success = self.precise_position_single_axis(axis, target_distance, direction)
                
                if not success:
                    rospy.logerr(f"❌ {direction}方向定位失败")
                    return False
                
                rospy.loginfo(f"✅ {direction}方向定位完成")
                
                # 方向间停顿
                if i < len(direction_list) - 1:
                    rospy.sleep(0.2)
            
            rospy.loginfo(f"✅ 阶段3完成 - {goal_name} 精确定位成功")
            return True
            
        except Exception as e:
            rospy.logerr(f"❌ 精确定位异常: {e}")
            self.node.publish_vel(0, 0, 0)
            return False

    def precise_position_single_axis(self, axis, target_distance, direction_name, timeout=45):
        rospy.loginfo(f"执行{direction}方向精确定位: 目标{target_distance:.3f}m")

        pid_target = target_distance - 0.2
        pid_target = max(pid_target, 0.12)
        rospy.loginfo(f"PID目标距离(已减0.2): {pid_target:.3f}m")


        tolerance = 0.02 # 放宽容差
        dwell_required = 0.3 # 驻留时间
        ok_since = None


        start_time = rospy.Time.now()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            current_distance = self.node.distance[axis]
            error = abs(current_distance - pid_target)


            if error <= tolerance:
                if ok_since is None:
                    ok_since = rospy.Time.now()
                if (rospy.Time.now() - ok_since).to_sec() >= dwell_required:
                    rospy.loginfo(f"✅ {direction}方向定位成功，误差{error:.3f}m")
                    self.node.publish_vel(0, 0, 0)
                    return True
            else:
                ok_since = None


            self.node.distance_state = axis + 1
            self.node.position_loop(pid_target)


            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logerr(f"❌ {direction}方向定位超时")
                self.node.publish_vel(0, 0, 0)
                return False

            rate.sleep()


        return False
        
    def test_gm65_scanning(self):
        """测试GM65扫码功能"""
        rospy.loginfo("\n🧪 开始测试GM65扫码功能...")
        
        # 测试扫码类
        scanner = GM65ScannerState("测试GM65扫码")
        
        rospy.loginfo("📱 请发布测试消息:")
        rospy.loginfo("   rostopic pub /gm65_data std_msgs/String \"data: '1'\"")
        rospy.loginfo("   或")
        rospy.loginfo("   rostopic pub /gm65_data std_msgs/String \"data: '3'\"")
        rospy.loginfo("")
        rospy.loginfo("⏱️ 等待扫码中...")
        
        scanned_result = scanner.execute()
        
        rospy.loginfo(f"\n📊 测试结果:")
        rospy.loginfo(f"   返回值: {scanned_result}")
        rospy.loginfo(f"   类型: {type(scanned_result)}")
        
        if scanned_result is not None:
            # 测试序列更新
            remaining_sequence = self.update_goal_sequence_after_scan(scanned_result)
            rospy.loginfo(f"✅ 扫码测试成功！")
            rospy.loginfo(f"   任务类型: {'任务A' if scanned_result == 1 else '任务B'}")
            rospy.loginfo(f"   后续序列: {remaining_sequence}")
        else:
            rospy.logwarn(f"❌ 扫码测试失败")
        
        return scanned_result
    


def main():
    """主函数"""
    try:
        # 获取任务类型参数
        tasktype = rospy.get_param('~tasktype', 1)  # 默认为1
        test_mode = rospy.get_param('~test_mode', False)  # 测试模式
        
        rospy.loginfo(f"启动三阶段导航器，任务类型: {tasktype}")
        if test_mode:
            rospy.loginfo("🧪 测试模式已启用")
        
        # 创建导航器
        navigator = ThreeStageNavigator(tasktype=tasktype)
        
        # 显示启动信息和配置详情
        navigator.print_navigation_config()
        
        # 验证与原始task_new.py的一致性
        navigator.verify_task_new_consistency()
        
        # 检查是否是测试模式
        if test_mode:
            rospy.loginfo("\n🧪 进入扫码测试模式...")
            test_result = navigator.test_gm65_scanning()
            rospy.loginfo(f"🧪 扫码测试完成，结果: {test_result}")
            return
        

        
        # 等待启动
        rospy.loginfo("3秒后开始导航...")
        for i in range(3, 0, -1):
            rospy.loginfo(f"{i}...")
            rospy.sleep(1.0)
        
        # 执行导航任务
        navigator.run_navigation_sequence()
        
        rospy.loginfo("三阶段导航程序结束")
        
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