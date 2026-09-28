#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""二维码双床导航总控（两个床位均执行机械臂动作）。

流程：
    护士台 -> 停留2秒 -> 扫码
      -> 第一个床位，发送扫码得到的两位指令（例如 ``31\r``）
      -> 等待STM32返回 ``1\r\n``
      -> 第二个床位，发送配对的两位指令（例如 ``13\r``）
      -> 等待STM32返回 ``2\r\n`` -> 返回起点

注意：本节点直接独占机械臂串口，运行期间不要再启动 arm.py、task_new.py
或其他会打开同一个 /dev/arm 串口的程序。
"""

import math
import threading
import time

import actionlib
import rospy
import serial
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
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
            0.1, float(rospy.get_param("~arm_reply_timeout", 60.0))
        )
        self.arm_first_success_reply = str(
            rospy.get_param("~arm_first_success_reply", "1")
        )
        self.arm_second_success_reply = str(
            rospy.get_param("~arm_second_success_reply", "2")
        )
        self.arm_command_suffix = str(rospy.get_param("~arm_command_suffix", "\r"))
        self.waypoints = rospy.get_param("~waypoints", {})
        self.route = rospy.get_param("~route", DEFAULT_ROUTE)
        self._validate_config()

        self.target_done_count = None
        rospy.Subscriber(
            self.target_done_topic,
            Int32,
            self._target_done_callback,
            queue_size=10,
        )

        self.scan_lock = threading.RLock()
        self.scan_event = threading.Event()
        self.accept_scan = False
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
            code, bed_route, medicine_id = decode_task_code(msg.data)
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
            self.bed_route = None
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
                    "/target_done jumped to %d while waiting for %d; navigation sequence is no longer synchronized",
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

    def _run_bed_arm_cycle(self, bed_name, command_code, success_reply, bed_order):
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

        rospy.logerr(
            "Timed out waiting for STM32 completion reply %r after %.1f seconds",
            success_reply,
            self.arm_reply_timeout,
        )
        return False

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
            "Route: nurse -> QR scan -> first bed + arm -> second bed + arm -> home"
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
            rospy.logwarn(
                "First-bed arm action did not finish within the allowed time; continuing to the second bed"
            )

        if not self._navigate_once(3, bed_route[1], 0.0):
            rospy.logerr("Navigation stopped before completing the second bed")
            return
        second_command = SECOND_BED_COMMANDS[task_code]
        if not self._run_bed_arm_cycle(
            bed_route[1], second_command, self.arm_second_success_reply, 2
        ):
            rospy.logwarn(
                "Second-bed arm action did not finish within the allowed time; continuing to the home point"
            )

        if not self._navigate_once(4, "home"):
            rospy.logerr("Navigation stopped before returning home")
            return

        rospy.loginfo("Returned home; QR, arm and all navigation stages completed")


if __name__ == "__main__":
    rospy.init_node("medical_competition_master_qr_arm")
    try:
        QrArmCompetitionMaster().run()
    except rospy.ROSInterruptException:
        pass
