#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rosgraph
import rosgraph.masterapi
from geometry_msgs.msg import Twist
from datetime import datetime

# 用来查询当前 /cmd_vel 的发布节点
def get_cmd_vel_publishers():
    master = rosgraph.masterapi.Master('/rostopic')
    try:
        state = master.getSystemState()
        pubs, subs, srvs = state
        for topic, publishers in pubs:
            if topic == '/cmd_vel':
                return publishers
    except Exception as e:
        rospy.logwarn("查询发布者失败: %s", str(e))
    return []

def callback(msg):
    # 获取时间戳
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")

    # 查询当前 /cmd_vel 发布者
    publishers = get_cmd_vel_publishers()

    print("-----")
    print("时间戳: {}".format(now))
    print("当前发布节点: {}".format(publishers if publishers else "未知"))
    print("Twist消息: linear(x={:.3f}, y={:.3f}, z={:.3f}), angular(x={:.3f}, y={:.3f}, z={:.3f})".format(
        msg.linear.x, msg.linear.y, msg.linear.z,
        msg.angular.x, msg.angular.y, msg.angular.z
    ))

def listener():
    rospy.init_node('cmd_vel_monitor', anonymous=True)
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.loginfo("正在监听 /cmd_vel ...")
    rospy.spin()

if __name__ == '__main__':
    listener()
