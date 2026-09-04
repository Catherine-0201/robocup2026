#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

def publish_initial_pose():
    rospy.init_node('set_initial_pose', anonymous=True)
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    
    # 等待发布者注册
    rospy.sleep(1.0)
    
    msg = PoseWithCovarianceStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "map"
    msg.pose.pose.position.x = 0.0  # 调整为你的起始x坐标
    msg.pose.pose.position.y = 0.0  # 调整为你的起始y坐标
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.x = 0.0
    msg.pose.pose.orientation.y = 0.0
    msg.pose.pose.orientation.z = 0.0  
    msg.pose.pose.orientation.w = 1.0  
    
    # 协方差矩阵（示例值，与AMCL类似）
    msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                           0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
    
    pub.publish(msg)
    rospy.loginfo("Initial pose published to /initialpose")
    
    # 发布后关闭节点
    rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        publish_initial_pose()
    except rospy.ROSInterruptException:
        pass
