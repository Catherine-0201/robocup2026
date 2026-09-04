#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""护士台扫码后只访问一张床的测试总控。

执行顺序：
    护士台 -> 停留2秒 -> 等待GM65扫码
      -> 11/13前往床位1，31/33前往床位3
      -> 床位停留原配置时间（当前5秒）-> 任务结束

第二位为药品编号，目前只打印，不执行放药。导航目标发送和到达判断
沿用 medical_competition_master_new.py 的 move_base + /target_done 机制。
"""

import math
import threading

import actionlib
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int32, String


VALID_TASK_CODES = {"11", "13", "31", "33"}
TOTAL_GOALS = 2


def decode_task_code(raw_code):
    """返回任务码、目标床位和药品编号。"""
    code = str(raw_code).strip()
    if code not in VALID_TASK_CODES:
        raise ValueError("task code must be one of 11, 13, 31, 33")
    bed_name = "bed1" if code[0] == "1" else "bed3"
    return code, bed_name, int(code[1])


class QrSingleBedCompetitionMaster:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.move_base_action = rospy.get_param("~move_base_action", "/move_base")
        self.target_done_topic = rospy.get_param("~target_done_topic", "/target_done")
        self.scan_topic = rospy.get_param("~scan_topic", "/gm65_data")
        self.nurse_hold_seconds = max(
            0.0, float(rospy.get_param("~nurse_hold_seconds", 2.0))
        )
        self.waypoints = rospy.get_param("~waypoints", {})
        self._validate_config()

        self.target_done_count = None
        rospy.Subscriber(
            self.target_done_topic,
            Int32,
            self._target_done_callback,
            queue_size=10,
        )

        # 到达护士台并停留2秒之前收到的二维码全部忽略。
        self.scan_lock = threading.RLock()
        self.scan_event = threading.Event()
        self.accept_scan = False
        self.task_code = None
        self.target_bed = None
        self.medicine_id = None
        rospy.Subscriber(
            self.scan_topic,
            String,
            self._scan_callback,
            queue_size=10,
        )

        self.move_base_client = actionlib.SimpleActionClient(
            self.move_base_action,
            MoveBaseAction,
        )

    def _validate_config(self):
        for name in ("nurse", "bed1", "bed3"):
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

    def _target_done_callback(self, msg):
        new_count = int(msg.data)
        if self.target_done_count is not None and new_count < self.target_done_count:
            rospy.logwarn(
                "/target_done decreased from %d to %d; B-spline node may have restarted",
                self.target_done_count,
                new_count,
            )
        self.target_done_count = new_count

    def _scan_callback(self, msg):
        with self.scan_lock:
            if not self.accept_scan:
                rospy.loginfo(
                    "Ignoring QR data %r received before nurse-station scan window",
                    msg.data,
                )
                return
            if self.task_code is not None:
                rospy.loginfo(
                    "Ignoring repeated QR data %r; task code %s is already locked",
                    msg.data,
                    self.task_code,
                )
                return

        try:
            code, target_bed, medicine_id = decode_task_code(msg.data)
        except ValueError:
            rospy.logwarn(
                "Ignoring invalid QR data %r; expected 11, 13, 31, or 33",
                msg.data,
            )
            return

        with self.scan_lock:
            if not self.accept_scan or self.task_code is not None:
                return
            self.task_code = code
            self.target_bed = target_bed
            self.medicine_id = medicine_id
            self.scan_event.set()

        rospy.loginfo(
            "Accepted QR code %s: target=%s; medicine=%d (record only)",
            code,
            target_bed,
            medicine_id,
        )

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

    def _wait_for_nurse_scan(self):
        with self.scan_lock:
            self.task_code = None
            self.target_bed = None
            self.medicine_id = None
            self.scan_event.clear()
            self.accept_scan = True

        rospy.loginfo(
            "Nurse-station hold finished; waiting for fresh QR data on %s",
            self.scan_topic,
        )
        rospy.loginfo("Expected QR task code: 11, 13, 31, or 33")
        while not rospy.is_shutdown():
            if self.scan_event.wait(0.1):
                with self.scan_lock:
                    self.accept_scan = False
                return True
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

    def _navigate_once(self, index, name, hold_seconds_override=None):
        waypoint = self.waypoints[name]
        label = waypoint.get("label", name)
        expected_count = self.target_done_count + 1

        rospy.loginfo("=" * 64)
        rospy.loginfo(
            "Navigation %d/%d -> %s (x=%.3f, y=%.3f, yaw=%.3f)",
            index,
            TOTAL_GOALS,
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

        if hold_seconds_override is None:
            hold_seconds = float(waypoint["hold_seconds"])
        else:
            hold_seconds = float(hold_seconds_override)
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

        rospy.loginfo("Route: nurse -> QR scan -> one selected bed -> finish")
        if not self._navigate_once(1, "nurse", self.nurse_hold_seconds):
            rospy.logerr("Navigation stopped before reaching nurse station")
            return

        if not self._wait_for_nurse_scan():
            return

        with self.scan_lock:
            task_code = self.task_code
            target_bed = self.target_bed
            medicine_id = self.medicine_id

        rospy.loginfo(
            "QR task %s selected route: nurse -> %s -> finish",
            task_code,
            target_bed,
        )
        rospy.loginfo(
            "Medicine %d is recorded only; no medicine action will run",
            medicine_id,
        )

        if not self._navigate_once(2, target_bed):
            rospy.logerr("Navigation stopped before completing the selected bed")
            return

        rospy.loginfo(
            "QR single-bed task %s completed at %s; controller finished",
            task_code,
            target_bed,
        )


if __name__ == "__main__":
    rospy.init_node("medical_competition_master_qr_single_bed")
    try:
        QrSingleBedCompetitionMaster().run()
    except rospy.ROSInterruptException:
        pass
