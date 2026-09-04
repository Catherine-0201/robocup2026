import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
import time
import serial

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


# def send_move_base_goal(x, y):
#     """
#     发布move_base目标点
    
#     参数:
#         x (float): 目标点的X坐标
#         y (float): 目标点的Y坐标
    
#     返回:
#         bool: 是否成功发布目标点
#     """
#     rospy.init_node('move_base_goal_publisher', anonymous=True)
    
#     # 创建发布者，发布到 /move_base_simple/goal 话题
#     goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

#     # 创建PoseStamped消息
#     goal_msg = PoseStamped()
#     goal_msg.header.stamp = rospy.Time.now()
#     goal_msg.header.frame_id = "map"  # 假设我们使用map坐标系

#     # 设置目标位置
#     goal_msg.pose.position.x = x
#     goal_msg.pose.position.y = y
#     goal_msg.pose.position.z = 0.0  # 假设目标平面上，Z为0

#     # 设置目标姿态（朝向可以根据需要调整，这里假设没有特定朝向）
#     goal_msg.pose.orientation.x = 0.0
#     goal_msg.pose.orientation.y = 0.0
#     goal_msg.pose.orientation.z = 0.0
#     goal_msg.pose.orientation.w = 1.0  # 无特定旋转时，w设置为1

#     # 发布目标点
#     try:
#         goal_pub.publish(goal_msg)
#         rospy.loginfo(f"目标点已发布: x={x}, y={y}")
#         return True
#     except Exception as e:
#         rospy.logerr(f"发布目标点失败: {e}")
#         return False
def send_move_base_goal(x, y):
    """
    发布move_base目标点
    
    参数:
        x (float): 目标点的X坐标
        y (float): 目标点的Y坐标
    
    返回:
        bool: 是否成功发布目标点
    """
    # rospy.init_node('move_base_goal_publisher', anonymous=True)  # 这里不需要再次初始化节点
    
    # 创建发布者，发布到 /move_base_simple/goal 话题
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    # 创建PoseStamped消息
    goal_msg = PoseStamped()
    goal_msg.header.stamp = rospy.Time.now()
    goal_msg.header.frame_id = "map"  # 假设我们使用map坐标系

    # 设置目标位置
    goal_msg.pose.position.x = x
    goal_msg.pose.position.y = y
    goal_msg.pose.position.z = 0.0  # 假设目标平面上，Z为0

    # 设置目标姿态（朝向可以根据需要调整，这里假设没有特定朝向）
    goal_msg.pose.orientation.x = 0.0
    goal_msg.pose.orientation.y = 0.0
    goal_msg.pose.orientation.z = 0.0
    goal_msg.pose.orientation.w = 1.0  # 无特定旋转时，w设置为1

    # 发布目标点
    try:
        goal_pub.publish(goal_msg)
        rospy.loginfo(f"目标点已发布: x={x}, y={y}")
        return True
    except Exception as e:
        rospy.logerr(f"发布目标点失败: {e}")
        return False


def main():
    """主函数 - 控制导航和机械臂"""
    rospy.init_node('main_control_node', anonymous=True)

    # 初始化扫码器和机械臂控制器
    scanner = GM65Scanner()
    arm_controller = ArmController()

    # 步骤1: 发布第一个目标点 (2, 0)
    send_move_base_goal(1.38, 0.05)

    # 步骤2: 执行GM65扫码
    scan_result = scanner.scan()

    # 步骤3: 根据扫码结果决定下一个目标点和机械臂任务
    if scan_result == 1:
        # 发布目标点 (5, 2)
        send_move_base_goal(4.97, 2.19)
        # 发送机械臂任务 1
        arm_controller.send_task_command(1)
    elif scan_result == 3:
        # 发布目标点 (5, -2)
        send_move_base_goal(5.0, -2.0)
        # 发送机械臂任务 3
        arm_controller.send_task_command(3)

    # 完成任务后关闭机械臂控制器
    arm_controller.close()


if __name__ == '__main__':
    main()
