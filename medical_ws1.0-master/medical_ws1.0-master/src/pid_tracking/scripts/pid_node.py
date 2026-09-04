#!/usr/bin/env python
import rospy
import math
import threading
import tf
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_msgs.msg import Float32MultiArray 
from pid import PID
import tf.transformations

class PIDTrackingNode:
    def __init__(self):
        rospy.init_node('pid_tracking_node')

        self.angle_k = [1.5, 0.1, 0]
        self.x_k = [0.5, 0, 0] #TODO,参数待调
        self.y_k = [0.5, 0, 0] #TODO,参数待调
        self.max_vel_x = rospy.get_param("x_max_vel", 0.5)
        self.max_vel_y = rospy.get_param("y_max_vel", 0.5)
        self.max_vel_w = rospy.get_param("w_max_vel", 0.5)
        self.acc = rospy.get_param("acc", 0.02)
        self.acc_time = rospy.get_param("time", 100)
        self.target_distance = [0.3,0.2,0.2]#前，左，右
        # self.right_distance = 0.2
        # self.front_distance = 0.3

        self.currant_distance_state = [0, 0, 0]
        self.init_angle = 0
        self.init_flag = False
        self.distance = [0.0,0.0,0.0]

        self.distance_state = 0 #1往左，定左边位置，2为右，定右边位置

        self.twist_msg = Twist()
        self.goal_point_num = rospy.get_param("goal_point_num", 1)
        self.goal_point = rospy.get_param("goal_point",[])
        # for i in range(self.goal_point_num):
        #     self.goal_point.append((list)[rospy.get_param(f"goal_point{i}_x", -1.0),
        #                    rospy.get_param(f"goal_point{i}_y", -1.0)])
        # self.goal_point = [[1.8,0.0],[0.0,1.9],[3.65,0.0],[-1.6,0.0],[0.0,-3.9],[1.6,0.0],[-5.7,0.0],[0.0,1.8]]

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.angle_pid = PID("PID_POSITION", self.angle_k, self.max_vel_w, 0.01 * self.max_vel_w)
        self.x_pid = PID("PID_POSITION", self.x_k, 0.4, 0.05)
        self.y_pid = PID("PID_POSITION", self.y_k, 0.4, 0.05)

        # Initialize PID controllers
        # self.angle_pid.init("PID_POSITION", self.angle_k, self.max_vel_w, 0.1 * self.max_vel_w)
        # self.x_pid.init("PID_POSITION", self.x_k, self.max_vel_x, 0.01 * self.max_vel_x) 
        # self.y_pid.init("PID_POSITION", self.y_k, self.max_vel_y, 0.01 * self.max_vel_y)

        rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.odom_callback)
        rospy.Subscriber("/lidar_distances",Float32MultiArray,self.distance_callback)#节点

        rospy.on_shutdown(self.MySigintHandler)
    def distance_callback(self,msg):
        for i in range(3):
            if msg.data[i] != float("inf"):
                self.distance[i] = msg.data[i]
            else:
                self.distance[i] = 10.0
    def odom_callback(self, msg):
        if not self.init_flag:
            self.init_angle = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, 
                                                                          msg.pose.pose.orientation.y, 
                                                                          msg.pose.pose.orientation.z, 
                                                                          msg.pose.pose.orientation.w])[2]
            # print("init_angle: ", self.init_angle)
            self.init_flag = True

        self.currant_distance_state[0] = msg.pose.pose.position.x
        self.currant_distance_state[1] = msg.pose.pose.position.y
        self.currant_distance_state[2] = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, 
                                                                          msg.pose.pose.orientation.y, 
                                                                          msg.pose.pose.orientation.z, 
                                                                          msg.pose.pose.orientation.w])[2]
        # rospy.loginfo("x:%f, y:%f, z:%f", self.currant_distance_state[0], self.currant_distance_state[1], self.currant_distance_state[2])
    
    def publish_vel(self, vx, vy, vw):
        self.twist_msg.linear.x = vx
        self.twist_msg.linear.y = vy
        self.twist_msg.angular.z = vw
        self.cmd_vel_pub.publish(self.twist_msg)
        # print(vx, vy, vw)

    def MySigintHandler(self):
        """自定义的关闭节点处理函数，确保在关闭时停止机器人"""
        print("Shutting down PID tracking node...")
        self.publish_vel(0, 0, 0)  # 停止所有运动
        rospy.signal_shutdown("Shutting down PID tracking node")

    def angle_loop(self, target_angle, vx, vy):
        if not self.init_flag:
            return
        if abs(self.currant_distance_state[2] - target_angle) < 0.0001:
            result = 0
        else:
            angle_pid_value = self.angle_pid.calc(target_angle, self.currant_distance_state[2])
            result = -angle_pid_value
        self.publish_vel(vx, vy, result)

    def Tspeed_move(self, angle, target_point, dir, acc, time):
        error_max = 0
        error_now = 0
        speed = 0
        init_pos = 0
        total_move = 0
        acc_displacement = 0
        acc_flag = False
        sgn_flag = 0

        while not self.init_flag and not rospy.is_shutdown():
            pass
        if dir == 1:
            target_point += self.currant_distance_state[0]
            print("X Get Goal from ",self.currant_distance_state[0]," to " ,target_point)
            init_pos = self.currant_distance_state[0]
            error_max = target_point - self.currant_distance_state[0]
            total_move = abs(error_max)
            error_now = target_point - self.currant_distance_state[0]
            sgn_flag = 1 if error_max > 0 else -1

            while speed <= self.max_vel_x and not rospy.is_shutdown():
                if abs(error_now) > total_move / 2:
                    speed += acc
                    self.angle_loop(self.init_angle, sgn_flag * speed, 0)
                    rospy.sleep(time / 1000.0)
                else:
                    acc_flag = True
                    break
                error_now = target_point - self.currant_distance_state[0]

            if acc_flag:
                while speed > 0 and not rospy.is_shutdown():
                    speed -= acc
                    self.angle_loop(self.init_angle, sgn_flag * speed, 0)
                    rospy.sleep(time / 1000.0)
                self.publish_vel(0, 0, 0)
                print("X Arrived: ",self.currant_distance_state[0])
            else:
                acc_displacement = abs(self.currant_distance_state[0] - init_pos)
                while abs(error_now) > acc_displacement and not rospy.is_shutdown():
                    self.angle_loop(self.init_angle, sgn_flag * speed, 0)
                    rospy.sleep(1 / 1000.0)
                    error_now = target_point - self.currant_distance_state[0]
                while speed > 0 and not rospy.is_shutdown():
                    speed -= acc
                    self.angle_loop(self.init_angle, sgn_flag * speed, 0)
                    rospy.sleep(time / 1000.0)
                self.publish_vel(0, 0, 0)
                print("X Arrived: ",self.currant_distance_state[0])
        elif dir == 2:
            target_point += self.currant_distance_state[1]
            print("Y Get Goal from ",self.currant_distance_state[1]," to " ,target_point)
            init_pos = self.currant_distance_state[1]
            error_max = target_point - self.currant_distance_state[1]
            total_move = abs(error_max)
            error_now = target_point - self.currant_distance_state[1]
            sgn_flag = 1 if error_max > 0 else -1

            while speed <= self.max_vel_x and not rospy.is_shutdown():
                if abs(error_now) > total_move / 2:
                    speed += acc
                    self.angle_loop(self.init_angle, 0 ,sgn_flag * speed)
                    rospy.sleep(time / 1000.0)
                else:
                    acc_flag = True
                    break
                error_now = target_point - self.currant_distance_state[1]

            if acc_flag:
                while speed > 0 and not rospy.is_shutdown():
                    speed -= acc
                    self.angle_loop(self.init_angle, 0 ,sgn_flag * speed)
                    rospy.sleep(time / 1000.0)
                self.publish_vel(0, 0, 0)
                print("Y Arrived: ",self.currant_distance_state[1])
            else:
                acc_displacement = abs(self.currant_distance_state[1] - init_pos)
                while abs(error_now) > acc_displacement and not rospy.is_shutdown():
                    self.angle_loop(self.init_angle, 0 ,sgn_flag * speed)
                    rospy.sleep(1 / 1000.0)
                    error_now = target_point - self.currant_distance_state[1]
                while speed > 0 and not rospy.is_shutdown():
                    speed -= acc
                    self.angle_loop(self.init_angle, 0 ,sgn_flag * speed)
                    rospy.sleep(time / 1000.0)
                self.publish_vel(0, 0, 0)
                print("Y Arrived: ",self.currant_distance_state[1])

    def task(self, x, y):
        if x == 0.0:
            self.Tspeed_move(0, y, 2, self.acc, self.acc_time)
            rospy.sleep(0.5)
        if y == 0.0:
            self.Tspeed_move(0, x, 1, self.acc, self.acc_time)
            rospy.sleep(0.5)
    # def run(self):
    def position_loop(self,target_distance):
        if self.distance_state == 0:
            return
        elif self.distance_state == 2: #取左边位置
            speed = self.y_pid.calc(self.distance[1],target_distance)
            self.angle_loop(self.init_angle,0,-speed)
        elif self.distance_state == 3: #取右边位置
            speed = self.y_pid.calc(self.distance[2],target_distance)
            self.angle_loop(self.init_angle,0,speed)            
        elif self.distance_state == 1: #取前边位置
            speed = self.y_pid.calc(self.distance[0],target_distance)
            self.angle_loop(self.init_angle,-speed,0)          
if __name__ == '__main__':
    node = PIDTrackingNode()
    spinner = threading.Thread(target=rospy.spin)
    spinner.start()
    def control_task():
        # while(1):
        #     node.angle_loop(node.init_angle,0,0)
        #巡诊台
        node.task(node.goal_point[0][0], node.goal_point[0][1])
        #左边
        node.task(node.goal_point[1][0], node.goal_point[1][1])
        #左定位
        node.distance_state = 2
        while not rospy.is_shutdown() and (abs(node.distance[1]-node.target_distance[1])>0.01):#  
            # print("hello")
            node.position_loop(node.target_distance[1])
        node.publish_vel(0,0,0)
        #往前
        node.task(node.goal_point[2][0], node.goal_point[2][1]) 
        #左定位
        while not rospy.is_shutdown() and (abs(node.distance[1]-node.target_distance[1])>0.01):#
            node.position_loop(node.target_distance[1])
        node.publish_vel(0,0,0)
        #前定位
        node.distance_state = 1
        while not rospy.is_shutdown() and (abs(node.distance[0]-node.target_distance[0])>0.01):#
            node.position_loop(node.target_distance[0])
        #后退
        node.task(node.goal_point[3][0], node.goal_point[3][1])
        #右走
        node.task(node.goal_point[4][0], node.goal_point[4][1])
        #右定位
        node.distance_state = 3
        while not rospy.is_shutdown() and (abs(node.distance[2]-node.target_distance[2])>0.01):#
            node.position_loop(node.target_distance[2])
        #前进
        node.task(node.goal_point[5][0], node.goal_point[5][1])
        #右定位
        while not rospy.is_shutdown() and (abs(node.distance[2]-node.target_distance[2])>0.01):#
            node.position_loop(node.target_distance[2])
        #前定位
        node.distance_state = 1
        while not rospy.is_shutdown() and (abs(node.distance[0]-node.target_distance[0])>0.01):#
            node.position_loop(node.target_distance[0])
        #回起始点
        node.task(node.goal_point[6][0], node.goal_point[6][1])
        node.task(node.goal_point[7][0], node.goal_point[7][1]) 
    control_thread = threading.Thread(target=control_task)
    control_thread.start()
    # 保持主线程运行，等待中断信号
    spinner.join()
    control_thread.join()  # 等待所有线程退出



