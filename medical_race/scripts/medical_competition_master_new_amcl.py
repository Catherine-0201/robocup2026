#!/usr/bin/env python3
"""Simple four-goal B-spline master for the AMCL navigation stack.

This is the AMCL counterpart of medical_competition_master_new.py.  It keeps
the same route and /target_done arrival contract, but does not wait for the
B-spline node's one-shot initial /target_done=0 message.  That message is not
latched and can be lost while ROS publishers/subscribers are connecting.

No PID controller, pose topic, localization status, or start signal is used.
"""

import math

import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int32


DEFAULT_ROUTE = ["nurse", "bed1", "bed3", "home"]


class AmclTargetDoneCompetitionMaster:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.move_base_action = rospy.get_param("~move_base_action", "/move_base")
        self.target_done_topic = rospy.get_param("~target_done_topic", "/target_done")
        self.waypoints = rospy.get_param("~waypoints", {})
        self.route = rospy.get_param("~route", DEFAULT_ROUTE)
        self.target_done_count = int(rospy.get_param("~initial_target_done_count", 0))
        self._validate_config()

        self.target_done_subscriber = rospy.Subscriber(
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

    def _target_done_callback(self, message):
        new_count = int(message.data)
        if new_count < self.target_done_count:
            rospy.logwarn(
                "/target_done decreased from %d to %d; accepting the B-spline reset",
                self.target_done_count,
                new_count,
            )
        self.target_done_count = new_count

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
            if current > expected_count:
                rospy.logerr(
                    "/target_done jumped to %d while waiting for %d",
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

        rospy.loginfo("Arrived at %s; /target_done=%d", label, expected_count)
        hold_seconds = float(waypoint["hold_seconds"])
        if hold_seconds > 0.0:
            rospy.loginfo("Holding at %s for %.1f seconds", label, hold_seconds)
            rospy.sleep(hold_seconds)
        return not rospy.is_shutdown()

    def run(self):
        rospy.loginfo("Waiting for move_base action: %s", self.move_base_action)
        self.move_base_client.wait_for_server()
        rospy.loginfo("move_base is ready")
        rospy.loginfo(
            "Using initial /target_done counter=%d (no one-shot startup message required)",
            self.target_done_count,
        )
        rospy.loginfo("Route: %s", " -> ".join(self.route))

        for index, name in enumerate(self.route, start=1):
            if not self._navigate_once(index, name):
                rospy.logerr("Navigation stopped before completing the route")
                return
        rospy.loginfo("Returned home; all four B-spline goals completed")


if __name__ == "__main__":
    rospy.init_node("medical_competition_master_new_amcl")
    try:
        AmclTargetDoneCompetitionMaster().run()
    except rospy.ROSInterruptException:
        pass
