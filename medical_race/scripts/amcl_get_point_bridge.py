#!/usr/bin/env python3
"""Expose the AMCL TF pose for the legacy get_point screen.

The bridge publishes position on /tf_xy and the map-relative heading on
/amcl_yaw.  The legacy screen can continue using /lidar_yaw for wall-angle
measurement without mixing the two yaw sources.
"""

import math

import rospy
import tf2_ros
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from tf.transformations import euler_from_quaternion


class AmclGetPointBridge:
    def __init__(self):
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.rate_hz = float(rospy.get_param("~rate", 10.0))
        self.yaw_topic = rospy.get_param("~yaw_topic", "/amcl_yaw")

        self.xy_pub = rospy.Publisher("/tf_xy", Point, queue_size=10)
        self.yaw_pub = rospy.Publisher(self.yaw_topic, Float32, queue_size=10)
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def publish_once(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rospy.Time(0),
                rospy.Duration(0.10),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            rospy.logwarn_throttle(
                2.0,
                "get_point AMCL bridge is waiting for %s -> %s: %s",
                self.map_frame,
                self.base_frame,
                exc,
            )
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        _, _, yaw_rad = euler_from_quaternion(
            [rotation.x, rotation.y, rotation.z, rotation.w]
        )

        self.xy_pub.publish(Point(x=translation.x, y=translation.y, z=0.0))
        # The legacy screen labels this topic in degrees and writes degrees to
        # its CSV, so keep that historical unit here.
        self.yaw_pub.publish(Float32(data=math.degrees(yaw_rad)))

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.publish_once()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("amcl_get_point_bridge")
    AmclGetPointBridge().spin()
