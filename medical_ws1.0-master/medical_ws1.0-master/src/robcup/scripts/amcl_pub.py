#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf
import numpy as np

class AmclPub:
    def __init__(self):
        rospy.init_node('amcl_pub')
        self.pose_pub = rospy.Publisher('/amcl_pose', PoseWithCovarianceStamped, queue_size=10)
        self.listener = tf.TransformListener()
        self.rate = rospy.Rate(10.0)  # 10Hz
        
        # 默认协方差矩阵：小值在对角线（表示高置信），其他为0
        # 6x6矩阵：位置(x,y,z)和旋转(roll,pitch,yaw)的协方差
        self.covariance = np.diag([0.01, 0.01, 0.01, 0.01, 0.01, 0.01]).flatten().tolist()

    def run(self):
        rospy.loginfo("AMCL Publisher started - publishing /amcl_pose from TF transform")
        while not rospy.is_shutdown():
            try:
                # 获取map到base_footprint的变换
                (trans, rot) = self.listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
                
                # 创建PoseWithCovarianceStamped消息
                pose_msg = PoseWithCovarianceStamped()
                pose_msg.header.stamp = rospy.Time.now()
                pose_msg.header.frame_id = "map"
                
                # 位置 (x, y, z)
                pose_msg.pose.pose.position.x = trans[0]
                pose_msg.pose.pose.position.y = trans[1]
                pose_msg.pose.pose.position.z = trans[2]
                
                # 方向（完整orientation）
                pose_msg.pose.pose.orientation.x = rot[0]
                pose_msg.pose.pose.orientation.y = rot[1]
                pose_msg.pose.pose.orientation.z = rot[2]
                pose_msg.pose.pose.orientation.w = rot[3]
                
                # 协方差
                pose_msg.pose.covariance = self.covariance
                
                # 发布
                self.pose_pub.publish(pose_msg)
                
                # 调试信息（可选，每10次打印一次）
                if rospy.get_time() % 1.0 < 0.1:  # 大约每秒打印一次
                    rospy.loginfo("Published pose: x=%.3f, y=%.3f, z=%.3f, q=[%.3f,%.3f,%.3f,%.3f]", 
                                 trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3])
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn("TF lookup failed: %s" % str(e))
                continue
            
            self.rate.sleep()

if __name__ == '__main__':
    try:
        amcl_pub = AmclPub()
        amcl_pub.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL Publisher stopped")