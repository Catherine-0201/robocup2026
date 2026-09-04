#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import message_filters
import numpy as np

class OdomConverter:
    def __init__(self):
        # 从参数服务器获取配置
        self.input_pose_topic = rospy.get_param('~input_pose_topic', '/robot_pose_ekf/odom_combined')
        self.input_odom_topic = rospy.get_param('~input_odom_topic', '/odom')
        self.output_topic = rospy.get_param('~output_topic', '/odom_converted')
        self.queue_size = rospy.get_param('~queue_size', 10)
        self.slop = rospy.get_param('~slop', 0.05)  # 默认时间同步容差 0.05 秒

        # 初始化节点
        rospy.init_node('odom_converter', anonymous=True)
        rospy.loginfo("Starting odom_converter node...")

        # 检查输入话题是否可用
        if not self._check_topics():
            rospy.logerr("Required topics not available. Shutting down.")
            rospy.signal_shutdown("Invalid topics")
            return

        # 发布者
        self.pub = rospy.Publisher(self.output_topic, Odometry, queue_size=self.queue_size)

        # 订阅者
        self.sub_pose = message_filters.Subscriber(self.input_pose_topic, PoseWithCovarianceStamped)
        self.sub_odom = message_filters.Subscriber(self.input_odom_topic, Odometry)

        # 时间同步器
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_pose, self.sub_odom],
            queue_size=self.queue_size,
            slop=self.slop
        )
        self.ts.registerCallback(self.callback)

    def _check_topics(self):
        """检查输入话题是否可用"""
        try:
            rospy.wait_for_message(self.input_pose_topic, PoseWithCovarianceStamped, timeout=5.0)
            rospy.wait_for_message(self.input_odom_topic, Odometry, timeout=5.0)
            rospy.loginfo("Input topics %s and %s are available.", self.input_pose_topic, self.input_odom_topic)
            return True
        except rospy.ROSException as e:
            rospy.logerr("Failed to find topics: %s", str(e))
            return False

    def _validate_covariance(self, pose_cov, twist_cov):
        """检查协方差矩阵是否有效（非负、对角线合理）"""
        try:
            pose_cov_np = np.array(pose_cov).reshape(6, 6)
            twist_cov_np = np.array(twist_cov).reshape(6, 6)
            if not (np.all(np.diag(pose_cov_np) >= 0) and np.all(np.diag(twist_cov_np) >= 0)):
                rospy.logwarn("Invalid covariance detected (negative diagonal elements).")
                return False
            return True
        except Exception as e:
            rospy.logwarn("Covariance validation failed: %s", str(e))
            return False

    def callback(self, pose_msg, odom_msg):
        # 创建新的 Odometry 消息
        odom_converted = Odometry()

        # 设置头部信息
        odom_converted.header = pose_msg.header
        odom_converted.header.frame_id = odom_msg.header.frame_id  # 动态继承 odom 话题的 frame_id
        odom_converted.child_frame_id = odom_msg.child_frame_id  # 动态继承 odom 话题的 child_frame_id

        # 检查协方差有效性
        if not self._validate_covariance(pose_msg.pose.covariance, odom_msg.twist.covariance):
            rospy.logwarn("Skipping message due to invalid covariance.")
            return

        # 使用 /robot_pose_ekf/odom_combined 的位姿
        odom_converted.pose = pose_msg.pose

        # 使用 /odom 的速度
        odom_converted.twist = odom_msg.twist

        # 发布转换后的消息
        self.pub.publish(odom_converted)
        rospy.logdebug("Published converted odometry message at time %s", odom_converted.header.stamp)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        converter = OdomConverter()
        converter.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("OdomConverter node interrupted.")
    except Exception as e:
        rospy.logerr("OdomConverter node failed: %s", str(e))