#!/usr/bin/env python
from pid_node import PIDTrackingNode
import smach
import rospy
import serial
import smach_ros
import threading
from std_msgs.msg import String

class TaskState(smach.State):
    def __init__(self, node, goal_idx):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.node = node
        self.goal_idx = goal_idx

    def execute(self, userdata):
        # rospy.loginfo(f"Executing task to goal {self.goal_idx}")
        if(self.goal_idx == 6):
            self.node.goal_point[self.goal_idx][0]=-self.node.currant_distance_state[0]
        elif self.goal_idx == 7:
            self.node.goal_point[self.goal_idx][1]=-self.node.currant_distance_state[1]
        self.node.task(self.node.goal_point[self.goal_idx][0], self.node.goal_point[self.goal_idx][1])
        return 'succeeded'

class PositionLoopState(smach.State):
    def __init__(self, node, distance, axis):
        smach.State.__init__(self, outcomes=['succeeded', 'in_progress'])
        self.node = node
        self.distance = distance #目标距离
        self.axis = axis  # 用来选择距离检查的维度
       
        # print(self.node.distance_state)

    def execute(self, userdata):
        # rospy.loginfo(f"Executing position loop on axis {self.axis}")
        if abs(self.node.distance[self.axis] - self.distance) > 0.01:
            self.node.distance_state = self.axis+1
            # print(self.node.distance_state,self.axis)
            self.node.position_loop(self.distance)
            return 'in_progress'
        self.node.publish_vel(0, 0, 0)
        return 'succeeded'
class Push_medicines_state(smach.State):
    def __init__(self,arm_serial,msg):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.arm_serial = arm_serial
        self.msg = msg
    def execute(self, userdata):
        rospy.loginfo("Executing SerialCommunicationState")
        self.arm_serial.write(self.msg.encode('utf-8'))

        # 等待下位机返回数据
        while not self.arm_serial.in_waiting:
            pass
        data = data = self.arm_serial.readline().decode('utf-8').strip()
        if data:
            rospy.loginfo(f"Received from serial: {data}")
            rospy.sleep(2)
            return 'succeeded'
        else:
            rospy.loginfo("No data received from serial.")
            return 'failed'
        
    def read_from_serial(ser):
        try:
            if ser.in_waiting > 0:
                data = ser.readline().decode('utf-8').strip()
                return data
        except Exception as e:
            rospy.logerr(f'Error reading from serial: {e}')
        return None
class Delay_state(smach.State):
    def __init__(self, time):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.time = time

       
        # print(self.node.distance_state)

    def execute(self, userdata):
        rospy.sleep(self.time)
        return 'succeeded'
class Race_task():
    def __init__(self):
        # 初始化PIDTrackingNode实例
        self.node = PIDTrackingNode()
        self._arm_serial = self.arm_serial()
        self.choose_data = 0
        rospy.Subscriber("/gm65_data",String,self.choose_callback)#节点
        self.lidar_distance = rospy.get_param("lader_point",[])
        for i in range(len(self.lidar_distance)):
            for j in range(3):
                self.lidar_distance[i][j] -= 0.2
                if self.lidar_distance[i][j] <= 0:
                    self.lidar_distance[i][j] = 0.1
        # self.test_distance=[0.3,0.2,0.15]
        # 创建状态机
        self.sm = smach.StateMachine(outcomes=['TASK_COMPLETED'])

    def choose_callback(self,msg):
        if msg.data == '1':
            self.choose_data = 1
        elif msg.data == '3':
            self.choose_data = 3
        print(self.choose_data,msg.data)



    def arm_serial(self):
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
    def config(self):
        if not hasattr(self, 'sm'): 
            self.sm = smach.StateMachine(outcomes=['TASK_COMPLETED'])  # 初始化 self.sm
        if self.choose_data == 1:
            with self.sm:
                # 添加任务状态到状态机中
                smach.StateMachine.add('TASK2', TaskState(self.node, 1), transitions={'succeeded': 'LEFT_POSITION_LOOP', 'failed': 'TASK_COMPLETED'})

                # 添加左定位的循环状态
                smach.StateMachine.add('LEFT_POSITION_LOOP', PositionLoopState(self.node, self.lidar_distance[0][1], axis=1), transitions={'succeeded': 'TASK3', 'in_progress': 'LEFT_POSITION_LOOP'})
                smach.StateMachine.add('TASK3', TaskState(self.node, 2), transitions={'succeeded': 'LEFT_POSITION_LOOP2', 'failed': 'TASK_COMPLETED'})

                smach.StateMachine.add('LEFT_POSITION_LOOP2', PositionLoopState(self.node, self.lidar_distance[1][1], axis=1), transitions={'succeeded': 'FRONT_POSITION_LOOP', 'in_progress': 'LEFT_POSITION_LOOP2'})
                smach.StateMachine.add('FRONT_POSITION_LOOP', PositionLoopState(self.node, self.lidar_distance[1][0], axis=0), transitions={'succeeded': 'DELAY_1', 'in_progress': 'FRONT_POSITION_LOOP'})
                smach.StateMachine.add('DELAY_1', Delay_state(2), transitions={'succeeded': 'LEFT_POSITION_LOOP3', 'failed': 'TASK_COMPLETED'})
                smach.StateMachine.add('LEFT_POSITION_LOOP3', PositionLoopState(self.node, self.lidar_distance[2][1], axis=1), transitions={'succeeded': 'FRONT_POSITION_LOOP1', 'in_progress': 'LEFT_POSITION_LOOP3'})
                smach.StateMachine.add('FRONT_POSITION_LOOP1', PositionLoopState(self.node, self.lidar_distance[2][0], axis=0), transitions={'succeeded': 'SEND_1', 'in_progress': 'FRONT_POSITION_LOOP1'})
                smach.StateMachine.add('SEND_1', Push_medicines_state(self._arm_serial, '1\r'),transitions={'succeeded': 'TASK4', 'failed': 'TASK_COMPLETED'})
                
                smach.StateMachine.add('TASK4', TaskState(self.node, 3), transitions={'succeeded': 'TASK5', 'failed': 'TASK_COMPLETED'})
                smach.StateMachine.add('TASK5', TaskState(self.node, 4), transitions={'succeeded': 'RIGHT_POSITION_LOOP', 'failed': 'TASK_COMPLETED'})

                # 添加右定位的循环状态
                smach.StateMachine.add('RIGHT_POSITION_LOOP', PositionLoopState(self.node, self.lidar_distance[3][2], axis=2), transitions={'succeeded': 'TASK6', 'in_progress': 'RIGHT_POSITION_LOOP'})
                smach.StateMachine.add('TASK6', TaskState(self.node, 5), transitions={'succeeded': 'RIGHT_POSITION_LOOP2', 'failed': 'TASK_COMPLETED'})

                smach.StateMachine.add('RIGHT_POSITION_LOOP2', PositionLoopState(self.node, self.lidar_distance[4][2], axis=2), transitions={'succeeded': 'FRONT_POSITION_LOOP2', 'in_progress': 'RIGHT_POSITION_LOOP2'})
                smach.StateMachine.add('FRONT_POSITION_LOOP2', PositionLoopState(self.node, self.lidar_distance[4][0], axis=0), transitions={'succeeded': 'DELAY_2', 'in_progress': 'FRONT_POSITION_LOOP2'})
                smach.StateMachine.add('DELAY_2', Delay_state(2), transitions={'succeeded': 'RIGHT_POSITION_LOOP3', 'failed': 'TASK_COMPLETED'})
                smach.StateMachine.add('RIGHT_POSITION_LOOP3', PositionLoopState(self.node, self.lidar_distance[5][2], axis=2), transitions={'succeeded': 'FRONT_POSITION_LOOP3', 'in_progress': 'RIGHT_POSITION_LOOP3'})
                smach.StateMachine.add('FRONT_POSITION_LOOP3', PositionLoopState(self.node, self.lidar_distance[5][0], axis=0), transitions={'succeeded': 'SEND_3', 'in_progress': 'FRONT_POSITION_LOOP3'})

                smach.StateMachine.add('SEND_3', Push_medicines_state(self._arm_serial, '3\r'),transitions={'succeeded': 'RIGHT_POSITION_LOOP4', 'failed': 'TASK_COMPLETED'})

                smach.StateMachine.add('RIGHT_POSITION_LOOP4', PositionLoopState(self.node, self.lidar_distance[3][2], axis=2), transitions={'succeeded': 'TASK7', 'in_progress': 'RIGHT_POSITION_LOOP4'})
                smach.StateMachine.add('TASK7', TaskState(self.node, 6), transitions={'succeeded': 'TASK8', 'failed': 'TASK_COMPLETED'})
                smach.StateMachine.add('TASK8', TaskState(self.node, 7), transitions={'succeeded': 'TASK_COMPLETED', 'failed': 'TASK_COMPLETED'})

            # 创建并启动 introspection 服务来可视化状态机
            self.sis = smach_ros.IntrospectionServer('server_name', self.sm, '/SM_ROOT')
        elif self.choose_data == 3: #先走3
            with self.sm:
                # 添加任务状态到状态机中
                for i in range(self.node.goal_point_num):
                    self.node.goal_point[i][1] = -self.node.goal_point[i][1]
                # smach.StateMachine.add('TASK1', TaskState(node, 0), transitions={'succeeded': 'TASK2', 'failed': 'TASK_COMPLETED'})
                smach.StateMachine.add('TASK2', TaskState(self.node, 1), transitions={'succeeded': 'RIGHT_POSITION_LOOP', 'failed': 'TASK_COMPLETED'})

                # 添加右定位的循环状态
                smach.StateMachine.add('RIGHT_POSITION_LOOP', PositionLoopState(self.node, self.lidar_distance[3][2], axis=2), transitions={'succeeded': 'TASK3', 'in_progress': 'RIGHT_POSITION_LOOP'})
                smach.StateMachine.add('TASK3', TaskState(self.node, 2), transitions={'succeeded': 'RIGHT_POSITION_LOOP1', 'failed': 'TASK_COMPLETED'})

                smach.StateMachine.add('RIGHT_POSITION_LOOP1', PositionLoopState(self.node, self.lidar_distance[4][2], axis=2), transitions={'succeeded': 'FRONT_POSITION_LOOP', 'in_progress': 'RIGHT_POSITION_LOOP1'})
                smach.StateMachine.add('FRONT_POSITION_LOOP', PositionLoopState(self.node, self.lidar_distance[4][0], axis=0), transitions={'succeeded': 'DELAY_1', 'in_progress': 'FRONT_POSITION_LOOP'})
                smach.StateMachine.add('DELAY_1', Delay_state(2), transitions={'succeeded': 'RIGHT_POSITION_LOOP2', 'failed': 'TASK_COMPLETED'})
                smach.StateMachine.add('RIGHT_POSITION_LOOP2', PositionLoopState(self.node, self.lidar_distance[5][2], axis=2), transitions={'succeeded': 'FRONT_POSITION_LOOP1', 'in_progress': 'RIGHT_POSITION_LOOP2'})
                smach.StateMachine.add('FRONT_POSITION_LOOP1', PositionLoopState(self.node, self.lidar_distance[5][0], axis=0), transitions={'succeeded': 'SEND_1', 'in_progress': 'FRONT_POSITION_LOOP1'})
                smach.StateMachine.add('SEND_1', Push_medicines_state(self._arm_serial, '3\r'),transitions={'succeeded': 'TASK4', 'failed': 'TASK_COMPLETED'})
                
                smach.StateMachine.add('TASK4', TaskState(self.node, 3), transitions={'succeeded': 'TASK5', 'failed': 'TASK_COMPLETED'})
                smach.StateMachine.add('TASK5', TaskState(self.node, 4), transitions={'succeeded': 'LEFT_POSITION_LOOP', 'failed': 'TASK_COMPLETED'})

                # 添加左定位的循环状态
                smach.StateMachine.add('LEFT_POSITION_LOOP', PositionLoopState(self.node, self.lidar_distance[0][1], axis=1), transitions={'succeeded': 'TASK6', 'in_progress': 'LEFT_POSITION_LOOP'})
                smach.StateMachine.add('TASK6', TaskState(self.node, 5), transitions={'succeeded': 'LEFT_POSITION_LOOP2', 'failed': 'TASK_COMPLETED'})

                smach.StateMachine.add('LEFT_POSITION_LOOP2', PositionLoopState(self.node, self.lidar_distance[1][1], axis=1), transitions={'succeeded': 'FRONT_POSITION_LOOP2', 'in_progress': 'LEFT_POSITION_LOOP2'})
                smach.StateMachine.add('FRONT_POSITION_LOOP2', PositionLoopState(self.node, self.lidar_distance[1][0], axis=0), transitions={'succeeded': 'DELAY_2', 'in_progress': 'FRONT_POSITION_LOOP2'})
                smach.StateMachine.add('DELAY_2', Delay_state(2), transitions={'succeeded': 'LEFT_POSITION_LOOP3', 'failed': 'TASK_COMPLETED'})
                smach.StateMachine.add('LEFT_POSITION_LOOP3', PositionLoopState(self.node, self.lidar_distance[2][1], axis=1), transitions={'succeeded': 'FRONT_POSITION_LOOP3', 'in_progress': 'LEFT_POSITION_LOOP3'})
                smach.StateMachine.add('FRONT_POSITION_LOOP3', PositionLoopState(self.node, self.lidar_distance[2][0], axis=0), transitions={'succeeded': 'SEND_3', 'in_progress': 'FRONT_POSITION_LOOP3'})
                smach.StateMachine.add('SEND_3', Push_medicines_state(self._arm_serial, '1\r'),transitions={'succeeded': 'LEFT_POSITION_LOOP4', 'failed': 'TASK_COMPLETED'})

                smach.StateMachine.add('LEFT_POSITION_LOOP4', PositionLoopState(self.node, self.lidar_distance[0][1], axis=1), transitions={'succeeded': 'TASK7', 'in_progress': 'LEFT_POSITION_LOOP4'})
                smach.StateMachine.add('TASK7', TaskState(self.node, 6), transitions={'succeeded': 'TASK8', 'failed': 'TASK_COMPLETED'})
                smach.StateMachine.add('TASK8', TaskState(self.node, 7), transitions={'succeeded': 'TASK_COMPLETED', 'failed': 'TASK_COMPLETED'})

            # 创建并启动 introspection 服务来可视化状态机
            self.sis = smach_ros.IntrospectionServer('server_name', self.sm, '/SM_ROOT')
    def run(self):
        #巡诊台
        
        self.node.task(self.node.goal_point[0][0], self.node.goal_point[0][1])
        
        self.choose_data = rospy.get_param("gm65_data",0)
        while self.choose_data == 0:
            self.choose_data = rospy.get_param("gm65_data",0)
            pass
        if self.choose_data == 1 or self.choose_data == 3:
            self.config()
            self.sis.start()
                # 执行状态机
            outcome = self.sm.execute()
                # 关闭 introspection 服务
            self.sis.stop()
    def test(self):
        self.choose_data = 1
        if not hasattr(self, 'sm'):
            self.sm = smach.StateMachine(outcomes=['TASK_COMPLETED'])  # 初始化 self.sm
        try:
            with self.sm:
                    smach.StateMachine.add('SEND_1', Push_medicines_state(self._arm_serial, '1\r'),transitions={'succeeded': 'TASK_COMPLETED', 'failed': 'TASK_COMPLETED'})
                    self.sis = smach_ros.IntrospectionServer('server_name', self.sm, '/SM_ROOT')
            self.sis.start()
                    # 执行状态机
            outcome = self.sm.execute()
        except NameError:
            self.sm = self.sm = smach.StateMachine(outcomes=['TASK_COMPLETED'])
if __name__ == '__main__':
    task = Race_task()
    task.run()
    # task.test()
    # print("hello")
