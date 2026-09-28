#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""GM65二维码到机械臂STM32的单次通信测试。

本节点不导航、不发布速度，也不使用 /target_done。收到第一个合法的两位
二维码后，将 ``任务码 + \r`` 写入 /dev/arm，并等待STM32完成机械臂动作后
返回 ``1\r\n``。测试结束后节点自动退出。

运行本节点时不要同时启动 arm.py、task_new.py、push_test.py 或其他占用
/dev/arm 的程序。
"""

import threading
import time

import rospy
import serial
from std_msgs.msg import String


VALID_TASK_CODES = {"11", "13", "31", "33"}


class QrArmSerialTest:
    def __init__(self):
        self.scan_topic = rospy.get_param("~scan_topic", "/gm65_data")
        self.arm_port = rospy.get_param("~arm_port", "/dev/arm")
        self.arm_baud_rate = int(rospy.get_param("~arm_baud_rate", 115200))
        self.reply_timeout = max(
            0.1, float(rospy.get_param("~reply_timeout", 30.0))
        )
        self.success_reply = str(rospy.get_param("~success_reply", "1"))

        self.scan_event = threading.Event()
        self.scan_lock = threading.Lock()
        self.task_code = None
        self.arm_serial = None

        rospy.Subscriber(
            self.scan_topic,
            String,
            self._scan_callback,
            queue_size=10,
        )
        self.command_pub = rospy.Publisher(
            "/arm_test_command_sent", String, queue_size=1, latch=True
        )
        rospy.on_shutdown(self._close_serial)

    def _scan_callback(self, msg):
        code = str(msg.data).strip()
        if code not in VALID_TASK_CODES:
            rospy.logwarn(
                "Ignoring invalid QR data %r; expected 11, 13, 31, or 33",
                msg.data,
            )
            return

        with self.scan_lock:
            if self.task_code is not None:
                rospy.loginfo(
                    "Ignoring repeated QR data %r; command %s is already locked",
                    msg.data,
                    self.task_code,
                )
                return
            self.task_code = code
            self.scan_event.set()

        rospy.loginfo("Accepted QR code: %s", code)

    def _open_serial(self):
        rospy.loginfo(
            "Opening STM32 serial port %s at %d baud",
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
            rospy.logerr("Unable to open STM32 serial port: %s", exc)
            return False

        rospy.loginfo("STM32 serial port opened successfully")
        return True

    def _close_serial(self):
        if self.arm_serial is not None and self.arm_serial.is_open:
            try:
                self.arm_serial.close()
                rospy.loginfo("STM32 serial port closed")
            except (serial.SerialException, OSError) as exc:
                rospy.logwarn("Failed to close STM32 serial port: %s", exc)

    def _send_and_wait(self):
        command = self.task_code + "\r"
        try:
            self.arm_serial.reset_input_buffer()
            rospy.loginfo("Sending command to STM32: %r", command)
            self.arm_serial.write(command.encode("ascii"))
            self.arm_serial.flush()
            self.command_pub.publish(String(data=self.task_code))
        except (serial.SerialException, serial.SerialTimeoutException, OSError) as exc:
            rospy.logerr("Failed to send command to STM32: %s", exc)
            return False

        rospy.loginfo(
            "Command sent. Waiting up to %.1f seconds for STM32 reply %r",
            self.reply_timeout,
            self.success_reply,
        )
        deadline = time.monotonic() + self.reply_timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            try:
                reply_bytes = self.arm_serial.readline()
            except (serial.SerialException, OSError) as exc:
                rospy.logerr("Failed while reading STM32 reply: %s", exc)
                return False

            if not reply_bytes:
                continue

            reply = reply_bytes.decode("utf-8", errors="replace").strip()
            rospy.loginfo("Received from STM32: %r", reply)
            if reply == self.success_reply:
                rospy.loginfo(
                    "TEST PASSED: Orin sent %r and received completion reply %r",
                    command,
                    reply,
                )
                return True

            rospy.logwarn(
                "Unexpected reply %r; still waiting for %r",
                reply,
                self.success_reply,
            )

        rospy.logerr(
            "TEST FAILED: timed out waiting for STM32 reply %r",
            self.success_reply,
        )
        return False

    def run(self):
        if not self._open_serial():
            return

        rospy.loginfo("=" * 64)
        rospy.loginfo("QR-to-arm test is ready; waiting on %s", self.scan_topic)
        rospy.loginfo("Scan one of: 11, 13, 31, 33")
        rospy.loginfo("No navigation or chassis command will be issued")

        while not rospy.is_shutdown() and not self.scan_event.wait(0.1):
            pass
        if rospy.is_shutdown():
            return

        self._send_and_wait()
        rospy.signal_shutdown("single QR-to-arm test finished")


if __name__ == "__main__":
    rospy.init_node("test_qr_arm_serial")
    try:
        QrArmSerialTest().run()
    except rospy.ROSInterruptException:
        pass
