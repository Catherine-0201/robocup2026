#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Strict command arbiter for the hybrid B-spline/precision controller.

Inputs:
  /cmd_vel_a                            raw coarse B-spline command
  /hybrid_control/internal_selected_cmd output of the original precision mux
  /hybrid_control/mode                  bspline or pid
  /move_base/goal                       starts a new goal state
  /target_done                          completes the active goal

Outputs:
  /cmd_vel_b                            fine command, zero outside FINE
  /cmd_vel                              the only chassis command in this launch
  /hybrid_control/strict_state          IDLE/COARSE/FINE/COMPLETE/FAULT

The original hybrid_precision_mux.py remains responsible for precision entry,
map/lidar/yaw corrections and /target_done.  This node only provides a strict,
fail-closed A/B output boundary so both command streams can be inspected.
"""

import math
import threading

import rospy
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionGoal
from std_msgs.msg import Int32, String


class StrictAbCmdMux:
    IDLE = "IDLE"
    COARSE = "COARSE"
    FINE = "FINE"
    COMPLETE = "COMPLETE"
    FAULT = "FAULT"

    def __init__(self):
        self.lock = threading.RLock()
        self.control_frequency = self._positive("~control_frequency", 30.0)
        self.cmd_timeout = self._positive("~cmd_timeout", 0.25)
        self.mode_guard_time = self._nonnegative("~mode_guard_time", 0.10)
        self.coarse_max_linear = self._positive("~coarse_max_linear", 0.36)
        self.coarse_max_angular = self._positive("~coarse_max_angular", 0.21)
        self.fine_max_linear = self._positive("~fine_max_linear", 0.18)
        self.fine_max_angular = self._positive("~fine_max_angular", 0.19)

        self.state = self.IDLE
        self.requested_mode = "unknown"
        self.active_goal_id = None
        self.goal_started = rospy.Time(0)
        self.fine_started = rospy.Time(0)
        self.latest_done = None
        self.goal_done_baseline = None
        self.fault_reason = ""

        self.last_a = Twist()
        self.last_a_time = rospy.Time(0)
        self.last_selected = Twist()
        self.last_selected_time = rospy.Time(0)

        self.final_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.fine_pub = rospy.Publisher("/cmd_vel_b", Twist, queue_size=10)
        self.state_pub = rospy.Publisher(
            "/hybrid_control/strict_state", String, queue_size=1, latch=True)
        self.state_pub.publish(String(data=self.state))

        rospy.Subscriber("/cmd_vel_a", Twist, self._a_cb, queue_size=10)
        rospy.Subscriber("/hybrid_control/internal_selected_cmd", Twist,
                         self._selected_cb, queue_size=10)
        rospy.Subscriber("/hybrid_control/mode", String,
                         self._mode_cb, queue_size=5)
        rospy.Subscriber("/move_base/goal", MoveBaseActionGoal,
                         self._goal_cb, queue_size=5)
        rospy.Subscriber("/target_done", Int32, self._done_cb, queue_size=10)

        rospy.Timer(rospy.Duration(1.0 / self.control_frequency), self._control)
        rospy.on_shutdown(self._stop)
        rospy.loginfo(
            "Strict A/B mux ready: A=/cmd_vel_a, B=/cmd_vel_b, final=/cmd_vel")

    @staticmethod
    def _positive(name, default):
        value = float(rospy.get_param(name, default))
        if not math.isfinite(value) or value <= 0.0:
            raise rospy.ROSInitException("%s must be finite and > 0" % name)
        return value

    @staticmethod
    def _nonnegative(name, default):
        value = float(rospy.get_param(name, default))
        if not math.isfinite(value) or value < 0.0:
            raise rospy.ROSInitException("%s must be finite and >= 0" % name)
        return value

    def _set_state(self, new_state, reason=""):
        if new_state == self.state and reason == self.fault_reason:
            return
        old_state = self.state
        self.state = new_state
        self.fault_reason = reason if new_state == self.FAULT else ""
        self.final_pub.publish(Twist())
        self.fine_pub.publish(Twist())
        self.state_pub.publish(String(data=new_state))
        if new_state == self.FAULT:
            rospy.logerr("Strict state %s -> FAULT: %s", old_state, reason)
        else:
            rospy.loginfo("Strict state %s -> %s", old_state, new_state)

    def _goal_cb(self, msg):
        goal_id = str(msg.goal_id.id)
        with self.lock:
            if goal_id and goal_id == self.active_goal_id:
                return
            self.active_goal_id = goal_id
            self.goal_started = rospy.Time.now()
            self.fine_started = rospy.Time(0)
            self.goal_done_baseline = self.latest_done
            self.requested_mode = "bspline"
            self.last_a_time = rospy.Time(0)
            self.last_selected_time = rospy.Time(0)
            self._set_state(self.COARSE)
            rospy.loginfo("New goal %r: strict controller starts in COARSE", goal_id)

    def _mode_cb(self, msg):
        mode = str(msg.data).strip().lower()
        with self.lock:
            if mode not in ("bspline", "pid"):
                self._set_state(self.FAULT, "invalid hybrid mode %r" % msg.data)
                return
            self.requested_mode = mode

    def _done_cb(self, msg):
        value = int(msg.data)
        with self.lock:
            if self.latest_done is not None and value < self.latest_done:
                self.latest_done = value
                if self.state in (self.COARSE, self.FINE):
                    self._set_state(self.FAULT, "/target_done decreased")
                return
            self.latest_done = value
            if self.state not in (self.COARSE, self.FINE):
                return
            if self.goal_done_baseline is None:
                self.goal_done_baseline = value
                return
            if value == self.goal_done_baseline + 1:
                self._set_state(self.COMPLETE)
            elif value > self.goal_done_baseline + 1:
                self._set_state(self.FAULT, "/target_done jumped by more than one")

    def _a_cb(self, msg):
        with self.lock:
            self.last_a = msg
            self.last_a_time = rospy.Time.now()

    def _selected_cb(self, msg):
        with self.lock:
            self.last_selected = msg
            self.last_selected_time = rospy.Time.now()

    @staticmethod
    def _command_ok(msg, max_linear, max_angular):
        values = (
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z,
        )
        if not all(math.isfinite(value) for value in values):
            return False, "NaN/Inf command"
        if (abs(msg.linear.z) > 1e-6 or abs(msg.angular.x) > 1e-6 or
                abs(msg.angular.y) > 1e-6):
            return False, "unsupported Twist axes are nonzero"
        if math.hypot(msg.linear.x, msg.linear.y) > max_linear + 1e-6:
            return False, "linear speed exceeds strict limit"
        if abs(msg.angular.z) > max_angular + 1e-6:
            return False, "angular speed exceeds strict limit"
        return True, ""

    def _fresh_after(self, timestamp, threshold, now):
        if timestamp == rospy.Time(0) or timestamp < threshold:
            return False
        age = (now - timestamp).to_sec()
        return 0.0 <= age <= self.cmd_timeout

    def _control(self, _event):
        now = rospy.Time.now()
        with self.lock:
            # A goal must begin in COARSE.  Once FINE is entered it is latched;
            # an unexpected return to bspline is treated as a fault, not a
            # silent hand-over that could mix two controllers.
            if self.state == self.COARSE:
                elapsed = (now - self.goal_started).to_sec()
                if (self.requested_mode == "pid" and
                        elapsed >= self.mode_guard_time):
                    self.fine_started = now
                    self.last_selected_time = rospy.Time(0)
                    self._set_state(self.FINE)
            elif self.state == self.FINE and self.requested_mode != "pid":
                self._set_state(
                    self.FAULT, "mode tried to return from FINE to COARSE")

            output = Twist()
            fine_visible = Twist()
            if self.state == self.COARSE:
                if not self._fresh_after(
                        self.last_a_time, self.goal_started, now):
                    rospy.logwarn_throttle(1.0, "COARSE waiting for fresh /cmd_vel_a")
                else:
                    ok, reason = self._command_ok(
                        self.last_a, self.coarse_max_linear,
                        self.coarse_max_angular)
                    if ok:
                        output = self.last_a
                    else:
                        self._set_state(self.FAULT, "cmd_vel_a: " + reason)
            elif self.state == self.FINE:
                if not self._fresh_after(
                        self.last_selected_time, self.fine_started, now):
                    rospy.logwarn_throttle(1.0, "FINE waiting for fresh fine command")
                else:
                    ok, reason = self._command_ok(
                        self.last_selected, self.fine_max_linear,
                        self.fine_max_angular)
                    if ok:
                        fine_visible = self.last_selected
                        output = self.last_selected
                    else:
                        self._set_state(self.FAULT, "cmd_vel_b: " + reason)

            # In IDLE, COMPLETE or FAULT both outputs are deliberately zero.
            if self.state != self.FINE:
                fine_visible = Twist()
            if self.state in (self.IDLE, self.COMPLETE, self.FAULT):
                output = Twist()
            self.fine_pub.publish(fine_visible)
            self.final_pub.publish(output)

    def _stop(self):
        for _ in range(5):
            self.fine_pub.publish(Twist())
            self.final_pub.publish(Twist())


if __name__ == "__main__":
    rospy.init_node("strict_ab_cmd_mux")
    StrictAbCmdMux()
    rospy.spin()
