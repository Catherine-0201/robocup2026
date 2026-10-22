#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Candidate-based nurse/bed arm master with plan-checked QR search.

The first valid QR is locked from node startup.  A QR received during nurse
navigation or search cancels that goal and starts the corresponding bed route.
Nurse and bed navigation coordinates come only from bed_candidates. Nurse
candidates are checked in saved order; there are no generated offset goals or
fallback to the legacy nurse waypoint. No in-place rotation is commanded.
"""

import math
import threading
import time

import actionlib
import rospy
import serial
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan, GetPlanRequest
from std_msgs.msg import Int32, String


DEFAULT_ROUTE = ["nurse", "bed1", "bed3", "home"]
VALID_TASK_CODES = {"11", "13", "31", "33"}
SECOND_BED_COMMANDS = {
    "11": "33",
    "33": "11",
    "13": "31",
    "31": "13",
}


def decode_task_code(raw_code):
    """返回规范化任务码、双床顺序和药品编号。"""
    code = str(raw_code).strip()
    if code not in VALID_TASK_CODES:
        raise ValueError("task code must be one of 11, 13, 31, 33")

    bed_route = ["bed1", "bed3"] if code[0] == "1" else ["bed3", "bed1"]
    return code, bed_route, int(code[1])


class QrArmCompetitionMaster:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.move_base_action = rospy.get_param("~move_base_action", "/move_base")
        self.target_done_topic = rospy.get_param("~target_done_topic", "/target_done")
        self.scan_topic = rospy.get_param("~scan_topic", "/gm65_data")
        self.arm_command_topic = rospy.get_param(
            "~arm_command_topic", "/arm_command_sent"
        )
        self.nurse_hold_seconds = max(
            0.0, float(rospy.get_param("~nurse_hold_seconds", 2.0))
        )
        self.arm_port = rospy.get_param("~arm_port", "/dev/arm")
        self.arm_baud_rate = int(rospy.get_param("~arm_baud_rate", 115200))
        self.arm_reply_timeout = max(
            0.1, float(rospy.get_param("~arm_reply_timeout", 40.0))
        )
        self.arm_first_success_reply = str(
            rospy.get_param("~arm_first_success_reply", "1")
        )
        self.arm_second_success_reply = str(
            rospy.get_param("~arm_second_success_reply", "2")
        )
        self.arm_command_suffix = str(rospy.get_param("~arm_command_suffix", "\r"))
        self.waypoints = rospy.get_param("~waypoints", {})
        self.bed_candidates = rospy.get_param("~bed_candidates", {})
        self.make_plan_service = rospy.get_param("~make_plan_service", "/move_base/make_plan")
        self.route = rospy.get_param("~route", DEFAULT_ROUTE)
        self._validate_config()

        self.target_done_count = None
        rospy.Subscriber(
            self.target_done_topic,
            Int32,
            self._target_done_callback,
            queue_size=10,
        )

        self.return_lock = threading.RLock()
        self.return_event = threading.Event()
        self.waiting_return_request = None
        self.return_request_pub = rospy.Publisher(
            "/hybrid_control/return_request", String, queue_size=5)
        rospy.Subscriber("/hybrid_control/return_done", String,
                         self._return_done_callback, queue_size=5)

        self.scan_lock = threading.RLock()
        self.scan_event = threading.Event()
        self.task_code = None
        self.bed_route = None
        self.medicine_id = None
        rospy.Subscriber(
            self.scan_topic,
            String,
            self._scan_callback,
            queue_size=10,
        )

        self.arm_command_pub = rospy.Publisher(
            self.arm_command_topic, String, queue_size=10
        )
        self.move_base_client = actionlib.SimpleActionClient(
            self.move_base_action, MoveBaseAction
        )
        self.make_plan = rospy.ServiceProxy(self.make_plan_service, GetPlan)

        self.arm_serial = None
        rospy.on_shutdown(self._close_arm_serial)

    def _validate_config(self):
        if list(self.route) != DEFAULT_ROUTE:
            raise rospy.ROSInitException(
                "~route must be exactly: nurse, bed1, bed3, home"
            )

        for name in self.route:
            if name not in self.waypoints:
                raise rospy.ROSInitException("missing waypoint: %s" % name)
            waypoint = self.waypoints[name]
            required = ("x", "y", "yaw", "hold_seconds") if name == "home" else (
                "yaw", "hold_seconds")
            for key in required:
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

        if not isinstance(self.bed_candidates, dict):
            raise rospy.ROSInitException("~bed_candidates must be a bed-name mapping")
        for name in ("nurse", "bed1", "bed3"):
            candidates = self.bed_candidates.get(name)
            if not isinstance(candidates, list) or not candidates:
                raise rospy.ROSInitException(
                    "bed_candidates/%s must contain at least one candidate; "
                    "navigation will not fall back to competition_waypoints.yaml"
                    % name
                )
            for candidate in candidates:
                try:
                    values = (float(candidate["x"]), float(candidate["y"]),
                              float(candidate.get("yaw", self.waypoints[name]["yaw"])))
                except (KeyError, TypeError, ValueError, OverflowError):
                    raise rospy.ROSInitException("invalid candidate for " + name)
                if not all(math.isfinite(value) for value in values):
                    raise rospy.ROSInitException("non-finite candidate for " + name)
                if name != "nurse" and candidate.get("fine_order", "side_first") not in (
                        "side_first", "front_first"):
                    raise rospy.ROSInitException("invalid fine_order for " + name)

    def _open_arm_serial(self):
        rospy.loginfo(
            "Opening STM32 arm serial port %s at %d baud",
            self.arm_port,
            self.arm_baud_rate,
        )
        try:
            self.arm_serial = serial.Serial(
                self.arm_port,
                self.arm_baud_rate,
                timeout=0.1,
                write_timeout=1.0,
            )
        except (serial.SerialException, OSError) as exc:
            rospy.logerr("Failed to open STM32 arm serial port: %s", exc)
            return False

        rospy.loginfo("STM32 arm serial port opened successfully")
        return True

    def _close_arm_serial(self):
        if self.arm_serial is not None and self.arm_serial.is_open:
            try:
                self.arm_serial.close()
                rospy.loginfo("STM32 arm serial port closed")
            except (serial.SerialException, OSError) as exc:
                rospy.logwarn("Failed to close STM32 arm serial port: %s", exc)

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
            if self.task_code is not None:
                rospy.loginfo(
                    "Ignoring repeated QR data %r; task code %s is already locked",
                    msg.data,
                    self.task_code,
                )
                return

        try:
            code, bed_route, medicine_id = decode_task_code(msg.data)
        except ValueError:
            rospy.logwarn(
                "Ignoring invalid QR data %r; expected 11, 13, 31, or 33",
                msg.data,
            )
            return

        with self.scan_lock:
            if self.task_code is not None:
                return
            self.task_code = code
            self.bed_route = bed_route
            self.medicine_id = medicine_id
            self.scan_event.set()

        rospy.loginfo(
            "Accepted QR code %s: %s -> %s; medicine=%d",
            code,
            bed_route[0],
            bed_route[1],
            medicine_id,
        )

    def _return_done_callback(self, msg):
        with self.return_lock:
            if (self.waiting_return_request is not None
                    and str(msg.data).strip() == self.waiting_return_request):
                self.return_event.set()

    def _return_to_fine_start(self, bed_name):
        """Called after the expected STM32 reply or the reply wait timeout.

        Include the docking arrival counter so an old bed-return reply cannot
        release this wait. Repeat requests are safe in the navigation node.
        """
        request = "%s:%d" % (bed_name, self.target_done_count)
        with self.return_lock:
            self.waiting_return_request = request
            self.return_event.clear()
        rospy.loginfo("Starting return at %s in reverse of its selected fine order", bed_name)
        rospy.loginfo("Waiting for matching return completion on /hybrid_control/return_done")
        last_sent = None
        try:
            while not rospy.is_shutdown():
                if self.return_event.is_set():
                    rospy.loginfo("%s returned to fine entry; next coarse goal may start", bed_name)
                    return True
                now = time.monotonic()
                if last_sent is None or now - last_sent >= 1.0:
                    self.return_request_pub.publish(String(data=request))
                    last_sent = now
                self.return_event.wait(0.05)
            return False
        finally:
            with self.return_lock:
                self.waiting_return_request = None

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
            if self.task_code is not None:
                rospy.loginfo("Nurse-station hold finished; using locked QR code %s",
                              self.task_code)
                return True

        rospy.loginfo(
            "Nurse-station hold finished; waiting for the first valid QR code on %s",
            self.scan_topic,
        )
        rospy.loginfo("Expected QR task code: 11, 13, 31, or 33")
        while not rospy.is_shutdown():
            if self.scan_event.wait(0.1):
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
                    "/target_done jumped to %d while waiting for %d; navigation sequence is no longer synchronized",
                    current,
                    expected_count,
                )
                return False
            rate.sleep()
        return False

    def _navigate_once(self, index, name, hold_seconds_override=None):
        waypoint = self.waypoints[name]
        if name in ("bed1", "bed3"):
            waypoint = self._select_bed_candidate(name)
            if waypoint is None:
                return False
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
        goal = self._build_goal(waypoint)
        if name in ("bed1", "bed3"):
            # Publish metadata before sending the action. Its exact goal stamp
            # binds bed identity, list index and order to this goal, even when
            # candidates share coordinates. The controller reads it once.
            stamp = goal.target_pose.header.stamp
            rospy.set_param("/hybrid_control/selected_bed_goal", {
                "stamp_secs": stamp.secs, "stamp_nsecs": stamp.nsecs,
                "bed": name, "candidate_index": waypoint["_candidate_index"],
                "fine_order": waypoint["fine_order"],
                "x": float(waypoint["x"]), "y": float(waypoint["y"]),
            })
        self.move_base_client.send_goal(goal)

        if not self._wait_for_target_done(expected_count, label):
            return False

        rospy.loginfo(
            "Arrived at %s; received /target_done=%d", label, expected_count
        )

        hold_seconds = (
            float(waypoint["hold_seconds"])
            if hold_seconds_override is None
            else float(hold_seconds_override)
        )
        if hold_seconds > 0.0:
            rospy.loginfo("Holding at %s for %.1f seconds", label, hold_seconds)
            rospy.sleep(hold_seconds)
            if rospy.is_shutdown():
                return False
            rospy.loginfo("Hold finished at %s", label)
        return True

    def _select_bed_candidate(self, name):
        """Query in order, without executing any candidate navigation goal."""
        # /target_done finishes our navigation, not necessarily the move_base
        # action. The make_plan service requires that action to be inactive.
        state = self.move_base_client.get_state()
        if state in (GoalStatus.PENDING, GoalStatus.ACTIVE,
                     GoalStatus.PREEMPTING, GoalStatus.RECALLING):
            self.move_base_client.cancel_goal()
            if not self.move_base_client.wait_for_result(rospy.Duration(5.0)):
                rospy.logerr("Previous navigation goal did not cancel; refusing bed handoff")
                return None
        if rospy.is_shutdown():
            return None
        rospy.wait_for_service(self.make_plan_service)
        candidates = self.bed_candidates[name]
        for index, candidate in enumerate(candidates, 1):
            if rospy.is_shutdown():
                return None
            waypoint = dict(self.waypoints[name])
            waypoint.update(candidate)
            request = GetPlanRequest()
            # An empty start frame asks move_base to use the current robot pose.
            request.start = PoseStamped()
            request.goal = self._build_goal(waypoint).target_pose
            request.tolerance = 0.0
            rospy.loginfo("Checking %s candidate %d/%d: x=%.3f y=%.3f",
                          name, index, len(candidates),
                          float(waypoint["x"]), float(waypoint["y"]))
            response = self.make_plan(request)
            if not response.plan.poses:
                rospy.loginfo("%s candidate %d has no path; checking next", name, index)
                continue
            end = response.plan.poses[-1]
            if (end.header.frame_id.lstrip("/") != self.frame_id.lstrip("/")
                    or not all(math.isfinite(value) for value in
                               (end.pose.position.x, end.pose.position.y))
                    or math.hypot(end.pose.position.x - float(waypoint["x"]),
                                  end.pose.position.y - float(waypoint["y"])) > 0.05):
                rospy.loginfo("%s candidate %d path does not reach its goal", name, index)
                continue
            rospy.loginfo("Selected %s candidate %d/%d for coarse navigation",
                          name, index, len(candidates))
            waypoint["_candidate_index"] = index
            waypoint["fine_order"] = candidate.get("fine_order", "side_first")
            rospy.loginfo("Selected fine navigation order: %s", waypoint["fine_order"])
            return waypoint
        return None

    def _run_bed_arm_cycle(self, bed_name, command_code, success_reply, bed_order):
        """Allow return after a correct reply or timeout; fail on serial errors."""
        if self.arm_serial is None or not self.arm_serial.is_open:
            rospy.logerr("STM32 arm serial port is not open")
            return False

        command = command_code + self.arm_command_suffix
        command_bytes = command.encode("ascii")

        try:
            self.arm_serial.reset_input_buffer()
            rospy.loginfo("=" * 64)
            rospy.loginfo(
                "Arm action at bed %d/2 (%s); sending two-digit command %s to STM32",
                bed_order,
                bed_name,
                command_code,
            )
            self.arm_serial.write(command_bytes)
            self.arm_serial.flush()
            self.arm_command_pub.publish(String(data=command_code))
            rospy.loginfo("STM32 serial command sent: %r", command)
        except (serial.SerialException, serial.SerialTimeoutException, OSError) as exc:
            rospy.logerr("Failed to send arm command to STM32: %s", exc)
            return False

        # STM32收到两位任务码后负责依次完成伸出、放药和缩回。
        rospy.loginfo(
            "STM32 is executing: extend arm -> dispense medicine -> retract arm"
        )
        rospy.loginfo(
            "Waiting up to %.1f seconds for STM32 completion reply %r",
            self.arm_reply_timeout,
            success_reply,
        )

        deadline = time.monotonic() + self.arm_reply_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                reply_bytes = self.arm_serial.readline()
            except (serial.SerialException, OSError) as exc:
                rospy.logerr("Failed while reading STM32 reply: %s", exc)
                return False

            if not reply_bytes:
                continue

            reply = reply_bytes.decode("utf-8", errors="replace").strip()
            rospy.loginfo("Received STM32 reply: %r", reply)
            if reply == success_reply:
                rospy.loginfo(
                    "STM32 confirmed bed %d/2 arm action complete with reply %r",
                    bed_order,
                    reply,
                )
                return True

            rospy.logwarn(
                "Ignoring STM32 reply %r; waiting for completion reply %r",
                reply,
                success_reply,
            )

        if rospy.is_shutdown():
            return False

        rospy.logwarn(
            "No STM32 completion reply %r within %.1f seconds; starting reverse fine navigation",
            success_reply,
            self.arm_reply_timeout,
        )
        return True

    def run(self):
        rospy.loginfo("Waiting for move_base action: %s", self.move_base_action)
        self.move_base_client.wait_for_server()
        rospy.loginfo("move_base is ready")

        if not self._open_arm_serial():
            rospy.logerr("Competition master stopped because arm serial is unavailable")
            return
        if not self._wait_for_initial_target_done():
            return

        rospy.loginfo(
            "Route: nurse -> QR scan -> first bed + arm + return -> second bed + arm + return -> home"
        )
        if not self._navigate_once(1, "nurse", self.nurse_hold_seconds):
            rospy.logerr("Navigation stopped before reaching nurse station")
            return
        if not self._wait_for_nurse_scan():
            return

        with self.scan_lock:
            bed_route = list(self.bed_route)
            task_code = self.task_code
            medicine_id = self.medicine_id

        rospy.loginfo(
            "QR task %s selected route: nurse -> %s -> %s -> home; medicine=%d",
            task_code,
            bed_route[0],
            bed_route[1],
            medicine_id,
        )
        rospy.loginfo(
            "Arm command plan: first bed %s sends %s and waits for %s; second bed %s sends %s and waits for %s",
            bed_route[0],
            task_code,
            self.arm_first_success_reply,
            bed_route[1],
            SECOND_BED_COMMANDS[task_code],
            self.arm_second_success_reply,
        )

        # 两个床位到达后立即执行机械臂，正确回执分别为1和2。
        if not self._navigate_once(2, bed_route[0], 0.0):
            rospy.logerr("Navigation stopped before completing the first bed")
            return
        if not self._run_bed_arm_cycle(
            bed_route[0], task_code, self.arm_first_success_reply, 1
        ):
            rospy.logerr(
                "First-bed arm serial cycle failed; holding at bed without starting return or next goal"
            )
            return
        if not self._return_to_fine_start(bed_route[0]):
            return

        if not self._navigate_once(3, bed_route[1], 0.0):
            rospy.logerr("Navigation stopped before completing the second bed")
            return
        second_command = SECOND_BED_COMMANDS[task_code]
        if not self._run_bed_arm_cycle(
            bed_route[1], second_command, self.arm_second_success_reply, 2
        ):
            rospy.logerr(
                "Second-bed arm serial cycle failed; holding at bed without starting return or next goal"
            )
            return
        if not self._return_to_fine_start(bed_route[1]):
            return

        if not self._navigate_once(4, "home"):
            rospy.logerr("Navigation stopped before returning home")
            return

        rospy.loginfo("Returned home; QR, arm and all navigation stages completed")


_ACTIVE_GOAL_STATES = (
    GoalStatus.PENDING, GoalStatus.ACTIVE,
    GoalStatus.PREEMPTING, GoalStatus.RECALLING,
)
_FAILED_GOAL_STATES = (
    GoalStatus.ABORTED, GoalStatus.REJECTED,
    GoalStatus.PREEMPTED, GoalStatus.RECALLED, GoalStatus.LOST,
)


class CandidateEightSearchMaster(QrArmCompetitionMaster):
    """Plan-check saved nurse candidates before sending any navigation goal."""

    def __init__(self):
        super().__init__()
        self.nurse_goal_timeout = self._positive("~nurse_goal_timeout", 60.0)
        self.search_goal_timeout = self._positive("~search_goal_timeout", 15.0)
        self.search_total_timeout = self._positive("~search_total_timeout", 120.0)
        self.nurse_plan_tolerance = self._positive(
            "~nurse_plan_tolerance", 0.15)
        self.search_pause = max(
            0.0, float(rospy.get_param("~search_pause", 1.0)))

    @staticmethod
    def _positive(key, default):
        value = float(rospy.get_param(key, default))
        if not math.isfinite(value) or value <= 0.0:
            raise rospy.ROSInitException("%s must be finite and > 0" % key)
        return value

    def _qr_ready(self):
        with self.scan_lock:
            return self.task_code is not None

    def _cancel_goal_if_active(self):
        if self.move_base_client.get_state() not in _ACTIVE_GOAL_STATES:
            return True
        self.move_base_client.cancel_goal()
        if not self.move_base_client.wait_for_result(rospy.Duration(5.0)):
            rospy.logerr("Previous navigation goal did not cancel")
            return False
        return True

    def _send_search_goal(self, waypoint):
        if not self._cancel_goal_if_active():
            return None
        if self._qr_ready():
            return None
        expected = self.target_done_count + 1
        self.move_base_client.send_goal(self._build_goal(waypoint))
        return expected

    def _nurse_candidate_plannable(self, waypoint, label):
        """Use the same exact-goal make_plan check as bedside candidates."""
        try:
            request = GetPlanRequest()
            request.start = PoseStamped()  # move_base uses the current robot pose
            request.goal = self._build_goal(waypoint).target_pose
            request.tolerance = 0.0
            response = self.make_plan(request)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn("Cannot check %s: %s", label, exc)
            return False
        if not response.plan.poses:
            rospy.loginfo("%s has no global path; checking next candidate", label)
            return False
        end = response.plan.poses[-1]
        if end.header.frame_id.lstrip("/") != self.frame_id.lstrip("/"):
            rospy.logwarn("%s plan returned a different frame", label)
            return False
        x, y = end.pose.position.x, end.pose.position.y
        if not (math.isfinite(x) and math.isfinite(y)):
            rospy.logwarn("%s plan returned invalid coordinates", label)
            return False
        error = math.hypot(x - float(waypoint["x"]),
                           y - float(waypoint["y"]))
        if error > self.nurse_plan_tolerance:
            rospy.loginfo("%s plan ends %.3f m from goal (> %.3f m); "
                          "checking next candidate",
                          label, error, self.nurse_plan_tolerance)
            return False
        rospy.loginfo("%s has a path (endpoint error %.3f m)", label, error)
        return True

    def _wait_goal_or_qr(self, expected, timeout):
        deadline = time.monotonic() + timeout
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self._qr_ready():
                return "qr"
            count = self.target_done_count
            if count == expected:
                return "done"
            if count is not None and count > expected:
                rospy.logerr(
                    "/target_done jumped past expected %d to %d",
                    expected, count)
                return "desync"
            if self.move_base_client.get_state() in _FAILED_GOAL_STATES:
                return "failed"
            if time.monotonic() >= deadline:
                return "timeout"
            rate.sleep()
        return "shutdown"

    def _pause_for_qr(self, seconds, deadline):
        until = min(deadline, time.monotonic() + seconds)
        while not rospy.is_shutdown() and time.monotonic() < until:
            if self._qr_ready():
                return True
            rospy.sleep(0.05)
        return self._qr_ready()

    def _nurse_candidate_positions(self):
        """Read only manually saved nurse candidates in priority order."""
        nurse = self.waypoints["nurse"]
        for candidate in self.bed_candidates["nurse"]:
            waypoint = dict(nurse)
            waypoint.update(candidate)
            yield waypoint

    def _finish_nurse_scan(self):
        if not self._cancel_goal_if_active():
            return False
        rospy.loginfo("QR %s locked; proceeding to %s",
                      self.task_code, self.bed_route[0])
        return True

    def _nurse_stage(self):
        # A QR may have arrived before run() reached this stage.
        if self._qr_ready():
            return self._finish_nurse_scan()

        try:
            rospy.wait_for_service(self.make_plan_service, timeout=3.0)
        except rospy.ROSException as exc:
            rospy.logerr("Nurse candidate planning service unavailable: %s", exc)
            return False

        candidates = list(self._nurse_candidate_positions())
        search_deadline = time.monotonic() + self.search_total_timeout
        had_plannable_point = False
        for index, waypoint in enumerate(candidates, 1):
            label = "护士台候选点%d" % index
            if rospy.is_shutdown():
                return False
            if self._qr_ready():
                return self._finish_nurse_scan()
            remaining = search_deadline - time.monotonic()
            if remaining <= 0.0:
                break
            if not self._cancel_goal_if_active():
                return False
            rospy.loginfo("Checking %s: x=%.3f y=%.3f",
                          label, float(waypoint["x"]), float(waypoint["y"]))
            if not self._nurse_candidate_plannable(waypoint, label):
                continue
            if self._qr_ready():
                return self._finish_nurse_scan()
            remaining = search_deadline - time.monotonic()
            if remaining <= 0.0:
                break
            first_plannable = not had_plannable_point
            goal_timeout = (self.nurse_goal_timeout if first_plannable else
                            self.search_goal_timeout)
            had_plannable_point = True
            timeout = min(goal_timeout, remaining)
            expected = self._send_search_goal(waypoint)
            if expected is None:
                return self._finish_nurse_scan() if self._qr_ready() else False
            rospy.loginfo(
                "Nurse navigation %d/%d -> %s (x=%.3f y=%.3f); "
                "expected /target_done=%d",
                index, len(candidates), label,
                float(waypoint["x"]), float(waypoint["y"]), expected)
            outcome = self._wait_goal_or_qr(expected, timeout)
            if outcome == "qr" or self._qr_ready():
                return self._finish_nurse_scan()
            if outcome in ("desync", "shutdown"):
                self._cancel_goal_if_active()
                return False
            if outcome != "done":
                if not self._cancel_goal_if_active():
                    return False
                rospy.logwarn("%s ended with %s; trying next plannable point",
                              label, outcome)
                continue
            pause = self.nurse_hold_seconds if first_plannable else self.search_pause
            if self._pause_for_qr(pause, search_deadline):
                return self._finish_nurse_scan()
            rospy.loginfo("No QR at %s; trying next plannable point", label)

        self._cancel_goal_if_active()
        if had_plannable_point:
            rospy.logerr("No valid QR after nurse candidate search")
        else:
            rospy.logerr("No saved nurse candidate has a global path")
        return False

    def _navigate_once(self, index, name, hold_seconds_override=None):
        if name == "nurse":
            return self._nurse_stage()
        return super()._navigate_once(index, name, hold_seconds_override)


if __name__ == "__main__":
    rospy.init_node("medical_competition_master_qr_arm_candidates_eight_search")
    try:
        CandidateEightSearchMaster().run()
    except rospy.ROSInterruptException:
        pass
