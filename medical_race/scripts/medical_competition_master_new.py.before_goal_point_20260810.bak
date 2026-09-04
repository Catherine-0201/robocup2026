#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""基于 /target_done 的四段 B 样条导航总控。

执行顺序：家中出发 -> 护士台 -> 床位1 -> 床位3 -> 返回家。
护士台、床位1、床位3到达后各停留配置中的时间（默认配置为5秒）。

本节点只负责：
1. 从 ROS 参数（由 YAML 加载）读取各导航点坐标；
2. 依次向 move_base action 发送目标；
3. 等待 B 样条节点发布的 /target_done 计数递增，作为唯一到达依据。

本节点不使用机器人当前位置，也不通过距离判断到达。
"""

import math

import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int32


DEFAULT_ROUTE = ["nurse", "bed1", "bed3", "home"]


class TargetDoneCompetitionMaster:
    """依次发送四个目标，并等待 /target_done 确认每次到达。"""

    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.move_base_action = rospy.get_param("~move_base_action", "/move_base")
        self.target_done_topic = rospy.get_param("~target_done_topic", "/target_done")
        self.waypoints = rospy.get_param("~waypoints", {})
        self.route = rospy.get_param("~route", DEFAULT_ROUTE)
        self._validate_config()

        # None 表示还没有收到 B 样条节点锁存发布的初始计数。
        self.target_done_count = None
        rospy.Subscriber(
            self.target_done_topic,
            Int32,
            self._target_done_callback,
            queue_size=10,
        )

        self.move_base_client = actionlib.SimpleActionClient(
            self.move_base_action,
            MoveBaseAction,
        )

    def _validate_config(self):
        if list(self.route) != DEFAULT_ROUTE:
            raise rospy.ROSInitException(
                "~route must be exactly: nurse, bed1, bed3, home"
            )

        for name in self.route:
            if name not in self.waypoints:
                raise rospy.ROSInitException("missing waypoint: %s" % name)

            waypoint = self.waypoints[name]
            for key in ("x", "y", "yaw", "hold_seconds"):
                if key not in waypoint:
                    raise rospy.ROSInitException(
                        "waypoint %s missing %s" % (name, key)
                    )

            if float(waypoint["hold_seconds"]) < 0.0:
                raise rospy.ROSInitException(
                    "waypoint %s hold_seconds must be >= 0" % name
                )

        if float(self.waypoints["home"]["hold_seconds"]) != 0.0:
            raise rospy.ROSInitException("home hold_seconds must be 0")

    def _target_done_callback(self, msg):
        new_count = int(msg.data)
        if self.target_done_count is not None and new_count < self.target_done_count:
            rospy.logwarn(
                "/target_done decreased from %d to %d; B-spline node may have restarted",
                self.target_done_count,
                new_count,
            )
        self.target_done_count = new_count

    def _wait_for_initial_target_done(self):
        rospy.loginfo("Waiting for initial counter from %s", self.target_done_topic)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.target_done_count is not None:
                rospy.loginfo(
                    "Initial /target_done counter is %d", self.target_done_count
                )
                return True
            rate.sleep()
        return False

    def _build_goal(self, waypoint):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(waypoint["x"])
        goal.target_pose.pose.position.y = float(waypoint["y"])

        half_yaw = float(waypoint["yaw"]) * 0.5
        goal.target_pose.pose.orientation.z = math.sin(half_yaw)
        goal.target_pose.pose.orientation.w = math.cos(half_yaw)
        return goal

    def _wait_for_target_done(self, expected_count, name):
        rospy.loginfo(
            "Waiting for %s: /target_done must change to %d",
            name,
            expected_count,
        )
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            current = self.target_done_count
            if current == expected_count:
                return True
            if current is not None and current > expected_count:
                rospy.logerr(
                    "/target_done jumped to %d while waiting for %d; "
                    "navigation sequence is no longer synchronized",
                    current,
                    expected_count,
                )
                return False
            rate.sleep()
        return False

    def _navigate_once(self, index, name):
        waypoint = self.waypoints[name]
        label = waypoint.get("label", name)
        expected_count = self.target_done_count + 1

        rospy.loginfo("=" * 64)
        rospy.loginfo(
            "Navigation %d/%d -> %s (x=%.3f, y=%.3f, yaw=%.3f)",
            index,
            len(self.route),
            label,
            float(waypoint["x"]),
            float(waypoint["y"]),
            float(waypoint["yaw"]),
        )
        rospy.loginfo("Expected /target_done: %d", expected_count)
        self.move_base_client.send_goal(self._build_goal(waypoint))

        if not self._wait_for_target_done(expected_count, label):
            return False

        rospy.loginfo(
            "Arrived at %s; received /target_done=%d",
            label,
            expected_count,
        )

        hold_seconds = float(waypoint["hold_seconds"])
        if hold_seconds > 0.0:
            rospy.loginfo("Holding at %s for %.1f seconds", label, hold_seconds)
            rospy.sleep(hold_seconds)
            if rospy.is_shutdown():
                return False
            rospy.loginfo("Hold finished at %s", label)

        return True

    def run(self):
        rospy.loginfo("Waiting for move_base action: %s", self.move_base_action)
        self.move_base_client.wait_for_server()
        rospy.loginfo("move_base is ready")

        if not self._wait_for_initial_target_done():
            return

        rospy.loginfo("Route: %s", " -> ".join(self.route))
        for index, name in enumerate(self.route, start=1):
            if not self._navigate_once(index, name):
                rospy.logerr("Navigation stopped before completing the route")
                return

        rospy.loginfo("Returned home; all four B-spline goals completed")


if __name__ == "__main__":
    rospy.init_node("medical_competition_master_new")
    try:
        TargetDoneCompetitionMaster().run()
    except rospy.ROSInterruptException:
        pass

