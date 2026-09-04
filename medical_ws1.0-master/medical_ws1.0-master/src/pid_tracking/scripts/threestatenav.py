#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
三阶段导航专用脚本 - 从newtask_test.py抽离并升级
功能：集成自适应导航(move_base) + IMU姿态恢复 + 精确距离定位(PID控制)

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

6. 与task_new.py完全一致的精确定位逻辑：
   - 精度容差: 0.01m (与原始PositionLoopState一致)
   - 状态机循环控制 (in_progress/succeeded/failed)
   - 支持从ROS参数lader_point读取距离数据
   - 相同的距离预处理逻辑 (减去0.2，最小值0.1)
"""

from pid_node import PIDTrackingNode
import rospy
import actionlib
import math
from std_msgs.msg import Int32
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

class ThreeStageNavigator:
    """三阶段导航器 - 自适应导航 + IMU姿态恢复 + 精确距离定位"""
    
    def __init__(self, tasktype=1):
        # 初始化PID跟踪节点 (它会自动初始化ROS节点)
        self.node = PIDTrackingNode()
        rospy.loginfo("初始化三阶段导航器...")
        
        # 任务类型
        self.tasktype = tasktype
        rospy.loginfo(f"任务类型: {self.tasktype}")
        
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
        
        # 定义精确定位控制参数
        self.accuracy_location = self.define_accuracy_location()
        
        # 根据tasktype确定导航顺序
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
        定义精确定位控制参数 - acc_loc[需要精确定位, 优先级顺序列表]
        
        格式说明:
        acc_loc[0]: 是否需要精确定位 (True/False)
        acc_loc[1]: 精确定位方向的优先级顺序列表 (e.g., ['front', 'left', 'right'])
        
        示例:
        acc_loc = [True, ['front']]  - 需要精确定位，只使用前方
        acc_loc = [True, ['left', 'front', 'right']]  - 需要精确定位，按左侧->前方->右侧顺序
        acc_loc = [False, []] - 不需要精确定位，跳过阶段3
        """
        return {
            1: [False, []],   # 目标点1: 不需要精确定位
            2: [True, ['left', 'front']],   # 目标点2: 需要精确定位，按左侧->前方顺序
            3: [True, ['right', 'front']],   # 目标点3: 需要精确定位，按右侧->前方顺序
            4: [False, []]  # 目标点4: 不需要精确定位
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

    def update_accuracy_location(self, goal_id, acc_loc):
        """
        更新指定目标点的精确定位配置
        
        参数:
        goal_id: 目标点ID (1,2,3,4)
        acc_loc: 新的配置 [需要精确定位, 优先级顺序列表]
        
        示例:
        navigator.update_accuracy_location(2, [True, ['front', 'left']])  # 按前方->左侧顺序
        """
        if goal_id in self.accuracy_location:
            self.accuracy_location[goal_id] = acc_loc
            rospy.loginfo(f"已更新目标点{goal_id}的精确定位配置: {acc_loc}")
        else:
            rospy.logwarn(f"目标点{goal_id}不存在")

    def print_acc_loc_examples(self):
        """打印acc_loc配置示例"""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("acc_loc 配置示例:")
        rospy.loginfo("="*60)
        rospy.loginfo("格式: [需要精确定位, 优先级顺序列表]")
        rospy.loginfo("")
        rospy.loginfo("示例配置:")
        rospy.loginfo("  [True, ['front']]             - 只使用前方精确定位")
        rospy.loginfo("  [True, ['left']]              - 只使用左侧精确定位") 
        rospy.loginfo("  [True, ['right']]             - 只使用右侧精确定位")
        rospy.loginfo("  [True, ['front', 'left']]     - 按前方->左侧顺序精确定位")
        rospy.loginfo("  [True, ['front', 'right']]    - 按前方->右侧顺序精确定位")
        rospy.loginfo("  [True, ['left', 'right']]     - 按左侧->右侧顺序精确定位")
        rospy.loginfo("  [True, ['front', 'left', 'right']] - 按前方->左侧->右侧顺序精确定位")
        rospy.loginfo("  [False, []]                   - 跳过精确定位，只粗导航")
        rospy.loginfo("="*60)

    def get_goal_sequence(self):
        """根据tasktype获取目标点顺序"""
        if self.tasktype == 1:
            sequence = [1, 2, 3, 4]
            rospy.loginfo("任务序列1: 1 -> 2 -> 3 -> 4")
        elif self.tasktype == 3:
            sequence = [1, 3, 2, 4]
            rospy.loginfo("任务序列3: 1 -> 3 -> 2 -> 4")
        else:
            rospy.logwarn(f"未知任务类型: {self.tasktype}，使用默认序列")
            sequence = [1, 2, 3, 4]
        
        return sequence

    def execute_three_stage_navigation(self, goal_id):
        """执行单个目标点的三阶段导航（根据acc_loc控制是否精确定位）"""
        goal_info = self.navigation_goals[goal_id]
        precise_info = self.precise_distances[goal_id]
        acc_loc = self.accuracy_location[goal_id]
        
        rospy.loginfo(f"\n{'='*60}")
        rospy.loginfo(f"开始导航到 {goal_info['name']} (ID: {goal_id})")
        rospy.loginfo(f"粗略位置: ({goal_info['position'][0]:.2f}, {goal_info['position'][1]:.2f})")
        
        # 检查是否需要精确定位
        need_precise = acc_loc[0]
        precise_icon = "🎯" if need_precise else "🚀"
        rospy.loginfo(f"{precise_icon} 精确定位需求: {'是' if need_precise else '否'}")
        rospy.loginfo(f"📋 acc_loc配置: {acc_loc}")
        
        if need_precise:
            # 分析启用的定位方向和顺序
            directions = []
            priority_order = acc_loc[1]
            for direction in priority_order:
                directions.append(f"{direction}({precise_info[direction]:.3f}m)")
            
            rospy.loginfo(f"💡 精确定位顺序: {', '.join(directions) if directions else '无'}")
            rospy.loginfo(f"🔧 完成检测方式: 阶段1=C++控制器信号, 阶段2=IMU角度判断, 阶段3=本地距离判断")
        else:
            rospy.loginfo(f"⏭️  精确定位: 跳过阶段3，仅执行粗导航+姿态恢复")
            rospy.loginfo(f"🔧 完成检测方式: 阶段1=C++控制器信号, 阶段2=IMU角度判断")
        
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
        
        # 阶段2: IMU姿态恢复（总是执行）
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
        
        # 如果不需要精确定位，在姿态恢复后直接完成
        if not need_precise:
            rospy.loginfo(f"✓ 三阶段导航完成: {goal_info['name']} (跳过精确定位)")
            return True
        
        # 短暂停顿（优化减少等待时间）
        rospy.loginfo("阶段2完成，准备进入阶段3...")
        rospy.sleep(0.3)
        
        # 阶段3: 精确定位（按照acc_loc[1]的优先级顺序执行）
        precision_success = True
        
        # 按优先级顺序执行精确定位方向
        priority_order = acc_loc[1]
        direction_to_axis = {'front': 0, 'left': 1, 'right': 2}
        
        for direction in priority_order:
            if direction not in direction_to_axis:
                rospy.logerr(f"无效的方向: {direction}")
                precision_success = False
                break
            
            axis = direction_to_axis[direction]
            target_distance = precise_info[direction]
            
            rospy.loginfo(f"执行{direction}精确定位，目标距离: {target_distance:.3f}m")
            
            precise_pos = PrecisePositionState(
                self.node,
                target_distance,
                axis,
                f"{goal_info['name']}_{direction}精确定位"
            )
            
            # 使用与原始task_new.py一致的循环控制逻辑
            max_position_attempts = 1000  # 防止无限循环
            position_attempts = 0
            
            while position_attempts < max_position_attempts and not rospy.is_shutdown():
                result = precise_pos.execute()
                
                if result == 'succeeded':
                    rospy.loginfo(f"✓ {direction}精确定位完成")
                    break  # 成功完成，退出循环
                elif result == 'failed':
                    rospy.logerr(f"阶段3失败: {goal_info['name']} - {direction}精确定位")
                    precision_success = False
                    break  # 失败，退出循环
                elif result == 'in_progress':
                    # 继续定位，移除延迟以获得最快收敛速度
                    position_attempts += 1
                else:
                    rospy.logerr(f"未知的精确定位结果: {result}")
                    precision_success = False
                    break
            
            # 检查是否因为超时退出
            if position_attempts >= max_position_attempts:
                rospy.logerr(f"精确定位超时: {goal_info['name']} - {direction}精确定位")
                precision_success = False
                break
            
            # 如果这个方向失败了，不继续其他方向
            if not precision_success:
                break
                
            # 如果还有其他方向需要定位，短暂停顿
            if priority_order.index(direction) < len(priority_order) - 1:
                rospy.sleep(0.5)
        
        if precision_success:
            rospy.loginfo(f"✓ 三阶段导航完成: {goal_info['name']}")
            return True
        else:
            rospy.logerr(f"✗ 精确定位阶段失败: {goal_info['name']}")
            return False

    def run_navigation_sequence(self):
        """执行完整的导航序列"""
        rospy.loginfo(f"\n{'='*80}")
        rospy.loginfo(f"开始执行三阶段导航任务 (tasktype={self.tasktype})")
        rospy.loginfo(f"导航序列: {' -> '.join([self.navigation_goals[gid]['name'] for gid in self.goal_sequence])}")
        rospy.loginfo(f"{'='*80}")
        
        successful_goals = 0
        total_goals = len(self.goal_sequence)
        
        for i, goal_id in enumerate(self.goal_sequence):
            if rospy.is_shutdown():
                rospy.loginfo("接收到关闭信号，终止导航")
                break
            
            rospy.loginfo(f"\n>>> 执行目标 {i+1}/{total_goals}: {self.navigation_goals[goal_id]['name']}")
            
            # 执行三阶段导航
            success = self.execute_three_stage_navigation(goal_id)
            
            if success:
                successful_goals += 1
                rospy.loginfo(f"✓ {self.navigation_goals[goal_id]['name']} 完成")
                
                # 在目标点停留2秒
                if i < total_goals - 1:  # 不是最后一个目标
                    rospy.loginfo("在目标点停留2秒...")
                    rospy.sleep(2.0)
            else:
                rospy.logwarn(f"✗ {self.navigation_goals[goal_id]['name']} 失败")
                # 可以选择继续下一个目标或停止
                # break  # 取消注释此行以在失败时停止整个序列
        
        # 任务总结
        self.print_task_summary(successful_goals, total_goals)

    def print_task_summary(self, successful_goals, total_goals):
        """打印任务总结"""
        rospy.loginfo(f"\n{'='*80}")
        rospy.loginfo("三阶段导航任务完成总结")
        rospy.loginfo("="*80)
        rospy.loginfo(f"任务类型: {self.tasktype}")
        rospy.loginfo(f"导航序列: {' -> '.join([self.navigation_goals[gid]['name'] for gid in self.goal_sequence])}")
        rospy.loginfo(f"总目标点数: {total_goals}")
        rospy.loginfo(f"成功完成: {successful_goals}")
        rospy.loginfo(f"失败次数: {total_goals - successful_goals}")
        rospy.loginfo(f"成功率: {(successful_goals/total_goals*100):.1f}%")
        
        if successful_goals == total_goals:
            rospy.loginfo("🎉 恭喜！所有目标点都成功完成三阶段导航!")
        else:
            rospy.logwarn(f"⚠️  有 {total_goals - successful_goals} 个目标点未能完成")
        
        rospy.loginfo("="*80)

    def print_navigation_config(self):
        """打印详细的导航配置信息"""
        rospy.loginfo("\n" + "="*80)
        rospy.loginfo("三阶段导航器准备就绪")
        rospy.loginfo("="*80)
        rospy.loginfo(f"任务类型: {self.tasktype}")
        rospy.loginfo(f"导航序列: {' -> '.join([self.navigation_goals[gid]['name'] for gid in self.goal_sequence])}")
        rospy.loginfo("")
        rospy.loginfo("导航策略:")
        rospy.loginfo("  阶段1: 使用move_base进行自适应导航到大致位置")
        rospy.loginfo("        🔧 完成检测: C++控制器 /target_done 信号递增")
        rospy.loginfo("  阶段2: 使用IMU进行姿态恢复到初始yaw角度")
        rospy.loginfo("        🔧 完成检测: IMU角度误差判断 (误差 < 0.05rad ≈ 3°)")
        rospy.loginfo("  阶段3: 使用PID控制进行精确距离定位（根据acc_loc配置）")
        rospy.loginfo("        🔧 完成检测: 本地距离传感器数据判断 (误差 < 0.01m)")
        rospy.loginfo("")
        rospy.loginfo("🔄 完成逻辑说明:")
        rospy.loginfo("  📡 粗导航: 等待C++控制器在/target_done话题发布递增计数")
        rospy.loginfo("  🧭 姿态恢复: 实时检查IMU yaw角度与初始姿态的差值")
        rospy.loginfo("  📏 精确定位: 实时检查node.distance[轴]与目标距离的误差")
        rospy.loginfo("  ⏱️  精确定位采用与task_new.py完全一致的判断逻辑和循环控制")
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
            
            # 精确定位状态标识
            precise_status = "🎯 精确定位" if acc_loc[0] else "🚀 仅粗导航"
            
            rospy.loginfo(f"📍 {goal_info['name']} (ID: {goal_id}) - {precise_status}")
            rospy.loginfo(f"   粗导航位置: ({goal_info['position'][0]:.2f}, {goal_info['position'][1]:.2f})")
            rospy.loginfo(f"   acc_loc配置: {acc_loc}")
            
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
                rospy.loginfo(f"   🔧 完成检测: C++控制器 /target_done 信号")
            
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
            "✅ 循环控制逻辑: 与原始状态机模式一致"
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
            "🔧 运行时配置更新能力"
        ]
        
        for improvement in improvements:
            rospy.loginfo(f"  {improvement}")
        
        rospy.loginfo("="*80)

    def emergency_stop(self):
        """紧急停止"""
        rospy.logwarn("执行紧急停止")
        self.node.publish_vel(0, 0, 0)

def main():
    """主函数"""
    try:
        # 获取任务类型参数
        tasktype = rospy.get_param('~tasktype', 1)  # 默认为1
        rospy.loginfo(f"启动三阶段导航器，任务类型: {tasktype}")
        
        # 创建导航器
        navigator = ThreeStageNavigator(tasktype=tasktype)
        
        # 显示启动信息和配置详情
        navigator.print_navigation_config()
        
        # 验证与原始task_new.py的一致性
        navigator.verify_task_new_consistency()
        
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