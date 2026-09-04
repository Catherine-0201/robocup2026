#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import tf.transformations

def odom_callback(msg):
    # 获取四元数
    orientation = msg.pose.pose.orientation
    # 转换为欧拉角（roll, pitch, yaw）
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [orientation.x, orientation.y, orientation.z, orientation.w])
    # 发布偏航角（yaw，绕 Z 轴）
    yaw_pub.publish(yaw)

rospy.init_node('odom_yaw_converter', anonymous=True)
yaw_pub = rospy.Publisher('/odom_yaw', Float64, queue_size=10)
rospy.Subscriber('/odometry/filtered', Odometry, odom_callback)
rospy.spin()