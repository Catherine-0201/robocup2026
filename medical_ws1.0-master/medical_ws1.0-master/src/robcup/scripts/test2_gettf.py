#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Point

def publish_tf_xy():
    # 初始化ROS节点
    rospy.init_node('tf_xy_publisher', anonymous=True)
    
    # 创建tf监听器
    listener = tf.TransformListener()
    
    # 创建发布者，发布x,y坐标
    pub = rospy.Publisher('tf_xy', Point, queue_size=10)
    
    # 等待TF缓冲区填充
    rospy.loginfo("Waiting for TF transform from 'map' to 'base_footprint'...")
    listener.waitForTransform('map', 'base_footprint', rospy.Time(), rospy.Duration(10.0))
    
    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        try:
            # 查找map到base_footprint的变换
            (trans, rot) = listener.lookupTransform('map', 'base_footprint', rospy.Time(0))
            
            # 创建Point消息，只包含x和y
            point_msg = Point()
            point_msg.x = trans[0]
            point_msg.y = trans[1]
            point_msg.z = 0.0  # z设为0，因为只需要x,y
            
            # 发布消息
            pub.publish(point_msg)
            rospy.loginfo("Published x: %.3f, y: %.3f", point_msg.x, point_msg.y)
            
        except tf.LookupException as e:
            rospy.logwarn("TF LookupException: %s", str(e))
        except tf.ConnectivityException as e:
            rospy.logwarn("TF ConnectivityException: %s", str(e))
        except tf.ExtrapolationException as e:
            rospy.logwarn("TF ExtrapolationException: %s", str(e))
            
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_tf_xy()
    except rospy.ROSInterruptException:
        pass