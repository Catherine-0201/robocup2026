#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import tf.transformations
import math

class YawOffsetConverter:
    def __init__(self):
        rospy.init_node('odom_yaw_offset_compare', anonymous=True)
        self.yaw_pub_filtered = rospy.Publisher('/odom_yaw_offset_filtered', Float64, queue_size=10)
        self.yaw_pub_raw = rospy.Publisher('/odom_yaw_offset_raw', Float64, queue_size=10)
        self.initial_yaw_filtered = None
        self.initial_yaw_raw = None
        self.first_msg_filtered = False
        self.first_msg_raw = False
        rospy.Subscriber('/odometry/filtered', Odometry, self.filtered_callback)
        rospy.Subscriber('/odom', Odometry, self.raw_callback)
        rospy.spin()

    def filtered_callback(self, msg):
        orientation = msg.pose.pose.orientation
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])
        if not self.first_msg_filtered:
            self.initial_yaw_filtered = yaw
            self.first_msg_filtered = True
            rospy.loginfo("Filtered initial yaw: %f radians", self.initial_yaw_filtered)
        yaw_offset = yaw - self.initial_yaw_filtered
        yaw_offset = math.atan2(math.sin(yaw_offset), math.cos(yaw_offset))
        yaw_offset_msg = Float64()
        yaw_offset_msg.data = yaw_offset
        self.yaw_pub_filtered.publish(yaw_offset_msg)

    def raw_callback(self, msg):
        orientation = msg.pose.pose.orientation
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])
        if not self.first_msg_raw:
            self.initial_yaw_raw = yaw
            self.first_msg_raw = True
            rospy.loginfo("Raw initial yaw: %f radians", self.initial_yaw_raw)
        yaw_offset = yaw - self.initial_yaw_raw
        yaw_offset = math.atan2(math.sin(yaw_offset), math.cos(yaw_offset))
        yaw_offset_msg = Float64()
        yaw_offset_msg.data = yaw_offset
        self.yaw_pub_raw.publish(yaw_offset_msg)

if __name__ == '__main__':
    try:
        YawOffsetConverter()
    except rospy.ROSInterruptException:
        pass