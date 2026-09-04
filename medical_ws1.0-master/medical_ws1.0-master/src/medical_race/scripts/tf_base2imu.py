#!/usr/bin/env python
import rospy
import tf2_ros
import geometry_msgs.msg

def static_transform_publisher():
    rospy.init_node('static_transform_publisher')


    br = tf2_ros.TransformBroadcaster()
    

    transform = geometry_msgs.msg.TransformStamped()
    transform.header.frame_id = "base_footprint"
    transform.child_frame_id = "imu_link"
    
    rate = rospy.Rate(10)  # 10Hz
    stamp = 0
    while not rospy.is_shutdown():
        last_stamp = stamp
        stamp = rospy.Time.now()
        if(stamp == last_stamp):
            continue
        transform.header.stamp = stamp
        
        #
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        
        #
        br.sendTransform(transform)
        
        rate.sleep()

if __name__ == '__main__':
    static_transform_publisher()
