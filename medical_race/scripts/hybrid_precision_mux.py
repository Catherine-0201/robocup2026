#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""B-spline/PID command arbiter for bedside precision docking.

The public interfaces remain /cmd_vel and /target_done.  Both followers run
behind private topics; B-spline controls normal travel and PID controls only
the final precision_radius around configured precision goals.
"""

import math
import threading

import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32MultiArray, Int32, String


class HybridPrecisionMux:
    def __init__(self):
        self.lock = threading.RLock()
        self.global_frame = rospy.get_param("~global_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.precision_radius = float(rospy.get_param("~precision_radius", 0.60))
        self.precision_settle_time = float(
            rospy.get_param("~precision_settle_time", 0.50))
        # Existing GetDistanceByLidarLine.py publishes [front, left, right].
        self.distance_topic = rospy.get_param(
            "~distance_topic", "/lidar_distances")
        self.distance_timeout = float(
            rospy.get_param("~distance_timeout", 0.30))
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.map_occupied_threshold = int(
            rospy.get_param("~map_occupied_threshold", 50))
        self.map_side_min_range = float(
            rospy.get_param("~map_side_min_range", 0.05))
        self.map_side_max_range = float(
            rospy.get_param("~map_side_max_range", 2.00))
        self.map_side_fan_deg = float(
            rospy.get_param("~map_side_fan_deg", 10.0))
        self.map_side_ray_count = int(
            rospy.get_param("~map_side_ray_count", 9))
        if self.map_side_ray_count < 1:
            raise rospy.ROSInitException("map_side_ray_count must be >= 1")
        self.front_target_distance = float(
            rospy.get_param("~front_target_distance", 0.30))
        self.side_target_distance = float(
            rospy.get_param("~side_target_distance", 0.25))
        self.front_tolerance = float(rospy.get_param("~front_tolerance", 0.03))
        self.side_tolerance = float(rospy.get_param("~side_tolerance", 0.03))
        self.yaw_tolerance = math.radians(float(
            rospy.get_param("~yaw_tolerance_deg", 3.0)))
        self.kp_front = float(rospy.get_param("~kp_front", 0.80))
        self.kp_side = float(rospy.get_param("~kp_side", 0.80))
        self.kp_yaw = float(rospy.get_param("~kp_yaw", 1.20))
        self.max_front_speed = float(rospy.get_param("~max_front_speed", 0.12))
        self.max_side_speed = float(rospy.get_param("~max_side_speed", 0.12))
        self.max_yaw_speed = float(rospy.get_param("~max_yaw_speed", 0.18))
        self.max_docking_range = float(
            rospy.get_param("~max_docking_range", 1.20))
        self.bed_sides = rospy.get_param(
            "~bed_sides", {"bed1": "left", "bed3": "right"})
        self.goal_match_tolerance = float(rospy.get_param("~goal_match_tolerance", 0.20))
        self.cmd_timeout = float(rospy.get_param("~cmd_timeout", 0.25))
        self.control_frequency = float(rospy.get_param("~control_frequency", 30.0))
        self.waypoints = rospy.get_param("~waypoints", {})
        names = rospy.get_param("~precision_waypoints", ["bed1", "bed3"])
        self.precision_goals = []
        for name in names:
            if name not in self.waypoints:
                raise rospy.ROSInitException("precision waypoint missing: %s" % name)
            point = self.waypoints[name]
            side = str(self.bed_sides.get(
                name, "left" if name == "bed1" else "right")).lower()
            if side not in ("left", "right"):
                raise rospy.ROSInitException(
                    "bed side for %s must be left or right" % name)
            self.precision_goals.append(
                (name, float(point["x"]), float(point["y"]),
                 float(point["yaw"]), side))

        self.active_goal = None
        self.mode = "bspline"
        self.last_cmd = {"bspline": Twist(), "pid": Twist()}
        self.last_cmd_time = {"bspline": rospy.Time(0), "pid": rospy.Time(0)}
        # Both controller counters start from zero.  The guarded B-spline
        # publishes that zero at startup, while the legacy PID publishes
        # nothing until its first real completion.  Using None here would
        # incorrectly discard the PID's first completion (value 1).
        self.raw_done = {"bspline": 0, "pid": 0}
        self.done_count = 0
        self.goal_completed = False
        self.precision_tolerance_since = None
        self.active_precision_goal = None
        self.latest_distances = None
        self.latest_distance_time = rospy.Time(0)
        self.latest_map = None

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.done_pub = rospy.Publisher("/target_done", Int32, queue_size=10, latch=True)
        self.mode_pub = rospy.Publisher("/hybrid_control/mode", String, queue_size=1, latch=True)
        self.done_pub.publish(Int32(data=0))
        self.mode_pub.publish(String(data=self.mode))

        rospy.Subscriber("/hybrid_control/bspline_cmd", Twist,
                         lambda msg: self._cmd_cb("bspline", msg), queue_size=10)
        rospy.Subscriber("/hybrid_control/pid_cmd", Twist,
                         lambda msg: self._cmd_cb("pid", msg), queue_size=10)
        rospy.Subscriber("/hybrid_control/bspline_done", Int32,
                         lambda msg: self._done_cb("bspline", msg), queue_size=10)
        rospy.Subscriber("/hybrid_control/pid_done", Int32,
                         lambda msg: self._done_cb("pid", msg), queue_size=10)
        rospy.Subscriber("/move_base/goal", MoveBaseActionGoal,
                         self._goal_cb, queue_size=5)
        rospy.Subscriber(self.distance_topic, Float32MultiArray,
                         self._distance_cb, queue_size=1)
        rospy.Subscriber(self.map_topic, OccupancyGrid,
                         self._map_cb, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.Timer(rospy.Duration(1.0 / self.control_frequency), self._control)
        rospy.on_shutdown(self._stop)
        rospy.loginfo("Hybrid mux ready: B-spline normally, PID within %.2f m of %s",
                      self.precision_radius,
                      ", ".join(name for name, _, _, _, _ in self.precision_goals))

    def _goal_cb(self, msg):
        pose = msg.goal.target_pose.pose.position
        with self.lock:
            self.active_goal = (pose.x, pose.y)
            self.active_precision_goal = self._match_precision_goal()
            self.goal_completed = False
            self.precision_tolerance_since = None
            self._set_mode("bspline")

    def _distance_cb(self, msg):
        with self.lock:
            if len(msg.data) < 3:
                rospy.logwarn_throttle(
                    1.0, "%s must contain [front, left, right]",
                    self.distance_topic)
                return
            values = (float(msg.data[0]), float(msg.data[1]), float(msg.data[2]))
            # Keep all three values. GetDistanceByLidarLine may legitimately
            # output inf on the unused side when no line is visible there;
            # _bedside_pid validates only front and the selected bed side.
            self.latest_distances = values
            self.latest_distance_time = rospy.Time.now()

    def _map_cb(self, msg):
        with self.lock:
            self.latest_map = msg

    def _map_cell_occupied(self, grid, world_x, world_y):
        """Return True for an occupied static-map cell, otherwise False."""
        info = grid.info
        # map_server maps normally have zero origin yaw, but handle a rotated
        # origin as well so world coordinates are converted correctly.
        q = info.origin.orientation
        origin_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        dx = world_x - info.origin.position.x
        dy = world_y - info.origin.position.y
        c = math.cos(origin_yaw)
        s = math.sin(origin_yaw)
        map_x = c * dx + s * dy
        map_y = -s * dx + c * dy
        mx = int(math.floor(map_x / info.resolution))
        my = int(math.floor(map_y / info.resolution))
        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return False
        value = grid.data[my * info.width + mx]
        # Unknown cells (-1) are not treated as a mapped bed surface.
        return value >= self.map_occupied_threshold

    def _raycast_map(self, grid, start_x, start_y, angle):
        """Distance from the robot centre to the first occupied map cell."""
        step = max(grid.info.resolution * 0.5, 0.01)
        distance = self.map_side_min_range
        while distance <= self.map_side_max_range:
            x = start_x + distance * math.cos(angle)
            y = start_y + distance * math.sin(angle)
            if self._map_cell_occupied(grid, x, y):
                return distance
            distance += step
        return None

    def _map_side_distance(self, robot_x, robot_y, robot_yaw, side):
        """Measure the selected side against occupied cells in /map."""
        if self.latest_map is None:
            rospy.logwarn_throttle(
                1.0, "Bedside PID waiting for occupancy grid %s",
                self.map_topic)
            return None
        grid = self.latest_map
        if grid.info.resolution <= 0.0 or not grid.data:
            rospy.logwarn_throttle(1.0, "Invalid occupancy grid on %s",
                                   self.map_topic)
            return None

        lateral = math.pi / 2.0 if side == "left" else -math.pi / 2.0
        centre_angle = robot_yaw + lateral
        count = self.map_side_ray_count
        fan = math.radians(max(0.0, self.map_side_fan_deg))
        distances = []
        for index in range(count):
            offset = 0.0 if count == 1 else (-fan + 2.0 * fan * index / (count - 1))
            measured = self._raycast_map(
                grid, robot_x, robot_y, centre_angle + offset)
            if measured is not None:
                distances.append(measured)
        if not distances:
            rospy.logwarn_throttle(
                1.0,
                "No mapped obstacle found on %s within %.2f m",
                side, self.map_side_max_range)
            return None
        distances.sort()
        middle = len(distances) // 2
        if len(distances) % 2:
            return distances[middle]
        return 0.5 * (distances[middle - 1] + distances[middle])

    def _cmd_cb(self, source, msg):
        with self.lock:
            self.last_cmd[source] = msg
            self.last_cmd_time[source] = rospy.Time.now()

    def _done_cb(self, source, msg):
        with self.lock:
            value = int(msg.data)
            previous = self.raw_done[source]
            self.raw_done[source] = value
            # Initial zero/latched values, duplicate counters and
            # inactive-controller completions are deliberately not forwarded.
            if value <= 0 or value <= previous:
                return
            if source != self.mode or self.goal_completed:
                return
            # A precision goal must always be certified by PID, even if the
            # still-running B-spline follower reaches its looser tolerance.
            if source == "bspline" and self._is_precision_goal():
                return
            # Precision goals are certified only by the three simultaneous
            # bedside conditions in _control.  The legacy PID may announce
            # completion merely because its path index reached the last point.
            if source == "pid" and self._is_precision_goal():
                return
            self._complete_goal(source)

    def _complete_goal(self, source):
        if self.goal_completed:
            return
        self.goal_completed = True
        self.done_count += 1
        self.cmd_pub.publish(Twist())
        self.done_pub.publish(Int32(data=self.done_count))
        rospy.loginfo("Hybrid goal complete via %s; /target_done=%d",
                      source, self.done_count)

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, value))

    @staticmethod
    def _normalize_angle(value):
        return math.atan2(math.sin(value), math.cos(value))

    def _match_precision_goal(self):
        if self.active_goal is None:
            return None
        gx, gy = self.active_goal
        for item in self.precision_goals:
            _, x, y, _, _ = item
            if math.hypot(gx - x, gy - y) <= self.goal_match_tolerance:
                return item
        return None

    def _is_precision_goal(self):
        if self.active_goal is None:
            return False
        return self._match_precision_goal() is not None

    def _bedside_pid(self, now, robot_x, robot_y, current_yaw):
        """Return (command, all_errors_inside_tolerance), or (None, False)."""
        if self.latest_distances is None:
            return None, False
        if ((now - self.latest_distance_time).to_sec()
                > self.distance_timeout):
            rospy.logwarn_throttle(
                1.0, "Bedside PID waiting for fresh %s", self.distance_topic)
            return None, False

        _, _, _, target_yaw, side = self.active_precision_goal
        front = self.latest_distances[0]
        side_range = self._map_side_distance(
            robot_x, robot_y, current_yaw, side)
        if not (math.isfinite(front) and front > 0.0):
            rospy.logwarn_throttle(
                1.0, "Invalid front lidar distance: front=%s", str(front))
            return None, False
        if side_range is None or not math.isfinite(side_range):
            return None, False
        if front > self.max_docking_range:
            rospy.logwarn_throttle(
                1.0,
                "Bedside PID front surface not found: front_lidar=%.2f",
                front)
            return None, False

        front_error = front - self.front_target_distance
        side_error = side_range - self.side_target_distance
        yaw_error = self._normalize_angle(target_yaw - current_yaw)
        side_sign = 1.0 if side == "left" else -1.0

        command = Twist()
        command.linear.x = self._clamp(
            self.kp_front * front_error, self.max_front_speed)
        command.linear.y = self._clamp(
            side_sign * self.kp_side * side_error, self.max_side_speed)
        command.angular.z = self._clamp(
            self.kp_yaw * yaw_error, self.max_yaw_speed)

        # Remove tiny commands independently to reduce limit-cycle shaking.
        if abs(front_error) <= self.front_tolerance:
            command.linear.x = 0.0
        if abs(side_error) <= self.side_tolerance:
            command.linear.y = 0.0
        if abs(yaw_error) <= self.yaw_tolerance:
            command.angular.z = 0.0

        settled = (abs(front_error) <= self.front_tolerance and
                   abs(side_error) <= self.side_tolerance and
                   abs(yaw_error) <= self.yaw_tolerance)
        rospy.loginfo_throttle(
            0.5,
            "Bedside PID %s: front_lidar=%.3f err=%.3f, "
            "side_map=%.3f err=%.3f, "
            "yaw_err=%.2f deg, settled=%s",
            side, front, front_error, side_range, side_error,
            math.degrees(yaw_error), str(settled))
        return command, settled

    def _set_mode(self, mode):
        if mode == self.mode:
            return
        self.cmd_pub.publish(Twist())
        self.mode = mode
        self.mode_pub.publish(String(data=mode))
        rospy.loginfo("Hybrid controller switched to %s", mode)

    def _control(self, _event):
        now = rospy.Time.now()
        with self.lock:
            # Once either controller has certified the current goal, the
            # master may deliberately hold for several seconds.  Both
            # followers can still receive the old global plan during that
            # hold and keep emitting small correction commands.  Never pass
            # those stale commands to the chassis; _goal_cb() clears this
            # flag as soon as the master sends the next goal.
            if self.goal_completed:
                self.cmd_pub.publish(Twist())
                return

            precision_distance = None
            robot_yaw = None
            robot_x = None
            robot_y = None
            if self.active_goal is not None and self._is_precision_goal():
                try:
                    tf_msg = self.tf_buffer.lookup_transform(
                        self.global_frame, self.base_frame, rospy.Time(0),
                        rospy.Duration(0.02))
                    p = tf_msg.transform.translation
                    robot_x = p.x
                    robot_y = p.y
                    q = tf_msg.transform.rotation
                    robot_yaw = math.atan2(
                        2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                    precision_distance = math.hypot(
                        self.active_goal[0] - p.x,
                        self.active_goal[1] - p.y)
                    if precision_distance <= self.precision_radius:
                        self._set_mode("pid")
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException):
                    pass

            if (self.mode == "pid" and robot_yaw is not None and
                    robot_x is not None and robot_y is not None):
                command, settled = self._bedside_pid(
                    now, robot_x, robot_y, robot_yaw)
                if command is None:
                    self.precision_tolerance_since = None
                    self.cmd_pub.publish(Twist())
                    return
                if settled:
                    if self.precision_tolerance_since is None:
                        self.precision_tolerance_since = now
                    if ((now - self.precision_tolerance_since).to_sec()
                            >= self.precision_settle_time):
                        self._complete_goal("lidar_yaw_pid")
                    self.cmd_pub.publish(Twist())
                    return
                self.precision_tolerance_since = None
                self.cmd_pub.publish(command)
                return

            age = (now - self.last_cmd_time[self.mode]).to_sec()
            output = self.last_cmd[self.mode] if age <= self.cmd_timeout else Twist()

            self.cmd_pub.publish(output)

    def _stop(self):
        for _ in range(5):
            self.cmd_pub.publish(Twist())


if __name__ == "__main__":
    rospy.init_node("hybrid_precision_mux")
    HybridPrecisionMux()
    rospy.spin()
