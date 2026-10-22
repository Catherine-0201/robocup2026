#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Single controller for B-spline selection, staged wall P and strict output.

Inputs: /move_base/goal, /cmd_vel_a, /hybrid_control/bspline_done,
        /lidar_distances, /handsfree/imu and map -> base_footprint TF.
Map TF selects the handoff radius; odom TF measures the 3 cm return step.
Fine yaw comes exclusively
from IMU orientation and returns to this node's first valid IMU yaw.
Outputs: /cmd_vel, /cmd_vel_b, /target_done and hybrid status topics.
After arm reply or timeout, /hybrid_control/return_request starts a reverse-order
return: the front stage backs up 3 cm in odom, and the side stage restores the
side distance captured at fine entry. /hybrid_control/return_done
acknowledges that return without incrementing /target_done.
Each return axis is completed once; unlike docking, return has no final
cross-axis verification or retry of an already completed axis.
No independent PID follower, occupancy-map raycast or intermediate mux.
Only formal COARSE-goal paths are relayed from /hybrid_control/raw_global_plan
to /move_base/GlobalPlanner/plan; candidate query paths are not tracked.
States: IDLE, COARSE, FINE, RETURN, COMPLETE. Invalid commands stop the current
cycle without locking navigation; valid data resumes control automatically.
"""

import math
import threading

import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Path
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, Int32, String


class BedsideNavigationController:
    def __init__(self):
        self.lock = threading.RLock()
        self.global_frame = rospy.get_param("~global_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.distance_topic = rospy.get_param("~distance_topic", "/lidar_distances")
        defaults = {
            "precision_radius": 0.60, "precision_settle_time": 1.0,
            "distance_timeout": 0.30,
            "side_wall_max_range": 2.50,
            "front_tolerance": 0.008, "side_tolerance": 0.008,
            "yaw_tolerance_deg": 0.3,
            "kp_front": 0.80, "kp_side": 0.80, "kp_yaw": 1.20,
            "max_front_speed": 0.12, "max_side_speed": 0.12,
            "max_yaw_speed": 0.18, "max_docking_range": 1.60,
            "goal_match_tolerance": 0.20, "cmd_timeout": 0.25,
            "control_frequency": 30.0,
            "coarse_max_linear": 0.36, "coarse_max_angular": 0.21,
            "fine_max_linear": 0.18, "fine_max_angular": 0.19,
        }
        for name, default in defaults.items():
            setattr(self, name, self._positive(name, default))
        self.mode_guard_time = float(rospy.get_param("~mode_guard_time", 0.10))
        if not math.isfinite(self.mode_guard_time) or self.mode_guard_time < 0:
            raise rospy.ROSInitException("mode_guard_time must be finite and >= 0")
        for name in ("lateral_ready_samples", "verification_samples"):
            value = int(rospy.get_param("~" + name, 3))
            if value < 1:
                raise rospy.ROSInitException(name + " must be >= 1")
            setattr(self, name, value)

        targets = {
            "bed1": ("left", "bed1_front_target_distance", 0.47,
                     "bed1_left_target_distance", 1.30),
            "bed3": ("right", "bed3_front_target_distance", 0.47,
                     "bed3_right_target_distance", 2.12),
        }
        waypoints = rospy.get_param("~waypoints", {})
        bed_candidates = rospy.get_param("~bed_candidates", {})
        if not isinstance(bed_candidates, dict):
            raise rospy.ROSInitException("~bed_candidates must be a bed-name mapping")
        self.precision_goals = []
        self.distance_targets = {}
        for name in rospy.get_param("~precision_waypoints", ["bed1", "bed3"]):
            if name not in waypoints or name not in targets:
                raise rospy.ROSInitException("unsupported or missing bed: " + name)
            side, front_param, front_default, side_param, side_default = targets[name]
            point = waypoints[name]
            # Waypoint x/y identifies beds and the handoff position. Its yaw
            # is intentionally not read: only IMU defines fine heading.
            coordinates = tuple(float(point[key]) for key in ("x", "y"))
            if not all(math.isfinite(value) for value in coordinates):
                raise rospy.ROSInitException("non-finite waypoint: " + name)
            self.precision_goals.append((name,) + coordinates + (side, "side_first", 0))
            candidates = bed_candidates.get(name, [])
            if not isinstance(candidates, list):
                raise rospy.ROSInitException("bed_candidates/%s must be a list" % name)
            for candidate_index, candidate in enumerate(candidates, 1):
                try:
                    xy = (float(candidate["x"]), float(candidate["y"]))
                except (KeyError, TypeError, ValueError, OverflowError):
                    raise rospy.ROSInitException("invalid candidate for " + name)
                if not all(math.isfinite(value) for value in xy):
                    raise rospy.ROSInitException("non-finite candidate for " + name)
                fine_order = candidate.get("fine_order", "side_first")
                if fine_order not in ("side_first", "front_first"):
                    raise rospy.ROSInitException("invalid fine_order for " + name)
                self.precision_goals.append((name,) + xy + (side, fine_order, candidate_index))
            self.distance_targets[name] = (
                self._positive(front_param, front_default),
                self._positive(side_param, side_default))

        self.active_goal = None
        self.active_goal_id = None
        self.active_goal_stamp = rospy.Time(0)
        self.latest_raw_path = None
        self.active_precision_goal = None
        self.active_fine_order = "side_first"
        self.active_candidate_index = 0
        self.goal_started = rospy.Time(0)
        self.last_a = Twist()
        self.last_a_time = rospy.Time(0)
        self.raw_bspline_done = 0
        self.coarse_arrival_reported = False
        self.done_count = 0
        self.state = "IDLE"
        self.mode = "bspline"
        self.latest_distances = None
        self.latest_distance_time = rospy.Time(0)
        self.distance_sequence = 0
        self.front_target_distance = None
        self.side_target_distance = None
        self.imu_yaw = None
        self.initial_imu_yaw = None
        self.fine_entry_distances = None
        self.return_back_start = None
        self.return_back_last_stamp = rospy.Time(0)
        self.return_front_direction = None
        self.active_return_request = None
        self.completed_return_request = None
        self._reset_docking_sequence()

        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.global_path_pub = rospy.Publisher(
            "/move_base/GlobalPlanner/plan", Path, queue_size=1)
        self.fine_pub = rospy.Publisher("/cmd_vel_b", Twist, queue_size=10)
        self.done_pub = rospy.Publisher("/target_done", Int32, queue_size=10, latch=True)
        self.return_done_pub = rospy.Publisher(
            "/hybrid_control/return_done", String, queue_size=1, latch=True)
        self.mode_pub = rospy.Publisher("/hybrid_control/mode", String, queue_size=1, latch=True)
        self.phase_pub = rospy.Publisher(
            "/hybrid_control/precision_phase", String, queue_size=1, latch=True)
        self.state_pub = rospy.Publisher(
            "/hybrid_control/strict_state", String, queue_size=1, latch=True)
        self.done_pub.publish(Int32(data=0))
        self.return_done_pub.publish(String(data=""))
        self.mode_pub.publish(String(data=self.mode))
        self.phase_pub.publish(String(data=self.docking_phase))
        self.state_pub.publish(String(data=self.state))

        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.Subscriber("/cmd_vel_a", Twist, self._a_cb, queue_size=10)
        rospy.Subscriber("/hybrid_control/bspline_done", Int32,
                         self._bspline_done_cb, queue_size=10)
        rospy.Subscriber("/move_base/goal", MoveBaseActionGoal,
                         self._goal_cb, queue_size=5)
        rospy.Subscriber("/hybrid_control/raw_global_plan", Path,
                         self._raw_path_cb, queue_size=1)
        rospy.Subscriber(self.distance_topic, Float32MultiArray,
                         self._distance_cb, queue_size=1)
        rospy.Subscriber("/handsfree/imu", Imu, self._imu_cb, queue_size=10)
        rospy.Subscriber("/hybrid_control/return_request", String,
                         self._return_request_cb, queue_size=5)
        rospy.Timer(rospy.Duration(1.0 / self.control_frequency), self._control)
        rospy.on_shutdown(self._stop)
        rospy.loginfo("Bedside controller ready: B-spline -> wall P -> arm -> return")

    @staticmethod
    def _positive(name, default):
        value = float(rospy.get_param("~" + name, default))
        if not math.isfinite(value) or value <= 0:
            raise rospy.ROSInitException(name + " must be finite and > 0")
        return value

    def _publish_output(self, command=None):
        command = command if command is not None else Twist()
        self.cmd_pub.publish(command)
        self.fine_pub.publish(command if self.state in ("FINE", "RETURN") else Twist())

    def _set_state(self, state):
        previous = self.state
        self.state = state
        self._publish_output()
        self.state_pub.publish(String(data=state))
        rospy.loginfo("Bedside state %s -> %s", previous, state)

    def _goal_cb(self, msg):
        with self.lock:
            goal_id = str(msg.goal_id.id)
            if goal_id and goal_id == self.active_goal_id:
                return
            self.active_goal_id = goal_id
            self.goal_started = rospy.Time.now()
            self.last_a_time = rospy.Time(0)
            self.coarse_arrival_reported = False
            self.active_precision_goal = None
            self.active_fine_order = "side_first"
            self.active_candidate_index = 0
            self.fine_entry_distances = None
            self.return_back_start = None
            self.return_back_last_stamp = rospy.Time(0)
            self.return_front_direction = None
            self.active_return_request = None
            self.completed_return_request = None
            self.return_done_pub.publish(String(data=""))
            self._reset_docking_sequence()
            self.mode = "bspline"
            self.mode_pub.publish(String(data=self.mode))
            pose = msg.goal.target_pose
            self.active_goal_stamp = (pose.header.stamp if pose.header.stamp != rospy.Time(0)
                                      else self.goal_started)
            x, y = float(pose.pose.position.x), float(pose.pose.position.y)
            self.active_goal = (x, y)
            if (pose.header.frame_id.lstrip("/") != self.global_frame.lstrip("/")
                    or not all(math.isfinite(value) for value in (x, y))):
                # Discard an unusable goal and wait for a valid one. Do not
                # remember its ID, so a corrected retry can use the same ID.
                self.active_goal = None
                self.active_goal_id = None
                self._set_state("IDLE")
                rospy.logwarn_throttle(
                    1.0, "Invalid goal: waiting for finite coordinates in global_frame")
                return
            # Candidate coordinates identify the same bed, while active_goal
            # remains the selected coarse entrance used for the handoff radius.
            for item in sorted(self.precision_goals,
                               key=lambda item: math.hypot(x - item[1], y - item[2])):
                name, bx, by = item[:3]
                if math.hypot(x - bx, y - by) <= self.goal_match_tolerance:
                    self.active_precision_goal = item
                    self.active_fine_order = item[4]
                    self.active_candidate_index = item[5]
                    self.front_target_distance, self.side_target_distance = self.distance_targets[name]
                    break
            selection = rospy.get_param("/hybrid_control/selected_bed_goal", {})
            if (isinstance(selection, dict)
                    and selection.get("stamp_secs") == pose.header.stamp.secs
                    and selection.get("stamp_nsecs") == pose.header.stamp.nsecs
                    and selection.get("x") == x and selection.get("y") == y
                    and selection.get("bed") in self.distance_targets
                    and selection.get("fine_order") in ("side_first", "front_first")):
                name = selection["bed"]
                side = next(item[3] for item in self.precision_goals if item[0] == name)
                self.active_fine_order = selection["fine_order"]
                self.active_candidate_index = int(selection["candidate_index"])
                self.active_precision_goal = (
                    name, x, y, side, self.active_fine_order, self.active_candidate_index)
                self.front_target_distance, self.side_target_distance = self.distance_targets[name]
            self._reset_docking_sequence()
            if self.active_precision_goal is not None:
                rospy.loginfo("Bed %s candidate %d: fine=%s; return reverses this order",
                              self.active_precision_goal[0], self.active_candidate_index,
                              self.active_fine_order)
            else:
                rospy.loginfo("目标未识别为1床或3床，按普通粗导航执行，不进入细导航；"
                              "请检查床位候选配置和本次目标绑定信息")
            self._set_state("COARSE")
            # ROS may deliver the first plan before its goal callback.
            if self.latest_raw_path is not None:
                self._forward_global_path(self.latest_raw_path)

    def _raw_path_cb(self, msg):
        with self.lock:
            self.latest_raw_path = msg
            self._forward_global_path(msg)

    def _forward_global_path(self, msg):
        """Relay only fresh paths for an executing coarse-navigation goal.

        make_plan also publishes paths. Queries occur between completed goals,
        so those paths never reach the unchanged B-spline follower. The stamp
        check also rejects a delayed query response after the next formal goal.
        """
        if (self.state != "COARSE" or self.active_goal is None or not msg.poses
                or msg.header.frame_id.lstrip("/") != self.global_frame.lstrip("/")
                or msg.header.stamp < self.active_goal_stamp):
            return
        end = msg.poses[-1].pose.position
        if (not all(math.isfinite(value) for value in (end.x, end.y))
                or math.hypot(end.x - self.active_goal[0],
                              end.y - self.active_goal[1]) > 0.35):
            return
        self.global_path_pub.publish(msg)

    def _a_cb(self, msg):
        with self.lock:
            self.last_a = msg
            self.last_a_time = rospy.Time.now()

    def _return_request_cb(self, msg):
        """Requests contain bed_name:arrival_count, e.g. bed1:2.

        The task master sends one after the expected STM32 arm reply or timeout.
        Retries are idempotent, including retries after a completed return.
        """
        request = str(msg.data).strip()
        with self.lock:
            if request and request == self.completed_return_request:
                self.return_done_pub.publish(String(data=request))
                return
            if self.active_precision_goal is None or self.fine_entry_distances is None:
                rospy.logwarn_throttle(1.0, "Return request ignored: no saved bed entry distances")
                return
            expected = "%s:%d" % (self.active_precision_goal[0], self.done_count)
            if request != expected:
                rospy.logwarn_throttle(1.0, "Return request does not match current bed/arrival")
                return
            if self.state == "RETURN" and request == self.active_return_request:
                return
            if self.state != "COMPLETE":
                rospy.logwarn_throttle(1.0, "Return request ignored: docking is not complete")
                return
            self.active_return_request = request
            self.return_back_start = None
            self.return_back_last_stamp = rospy.Time(0)
            self.return_front_direction = None
            self._reset_verification()
            self._set_docking_phase(self._ordered_phases(returning=True)[0])
            self._set_state("RETURN")
            rospy.loginfo("Returning %s: fixed front step=0.020 m; side target=%.3f m",
                          self.active_precision_goal[0], self.fine_entry_distances[1])

    def _complete_return(self):
        # Return is part of the same bed visit, not another navigation goal.
        self.completed_return_request = self.active_return_request
        self._set_docking_phase("RETURN_COMPLETE")
        self._set_state("COMPLETE")
        self.return_done_pub.publish(String(data=self.completed_return_request))
        rospy.loginfo("Bed return complete: %s; /target_done remains %d",
                      self.completed_return_request, self.done_count)

    def _bspline_done_cb(self, msg):
        with self.lock:
            value = int(msg.data)
            previous = self.raw_bspline_done
            self.raw_bspline_done = value
            if self.state not in ("COARSE", "FINE"):
                return
            if value < previous or value > previous + 1:
                # Rebase to the received count, but do not certify arrival
                # from a reset or a skipped count. The next normal increment
                # can complete an ordinary goal without requiring a new goal.
                rospy.logwarn_throttle(
                    1.0, "B-spline completion counter changed unexpectedly; rebased")
            elif value == previous + 1 and self.active_precision_goal is None:
                self._complete_goal("bspline")
            elif value == previous + 1 and self.state == "COARSE":
                self.coarse_arrival_reported = True
            # Bed goals are completed only by staged distance/yaw verification.

    def _distance_cb(self, msg):
        with self.lock:
            if len(msg.data) < 3:
                self.latest_distances = None
                self._reset_verification()
                return
            self.latest_distances = tuple(float(value) for value in msg.data[:3])
            self.latest_distance_time = rospy.Time.now()
            self.distance_sequence += 1

    def _imu_cb(self, msg):
        with self.lock:
            q = msg.orientation
            values = (q.x, q.y, q.z, q.w)
            norm = math.hypot(*values)
            if (not all(math.isfinite(value) for value in values)
                    or not math.isfinite(norm) or norm <= 1e-12):
                return
            x, y, z, w = (value / norm for value in values)
            self.imu_yaw = math.atan2(2.0 * (w * z + x * y),
                                      1.0 - 2.0 * (y * y + z * z))
            if self.initial_imu_yaw is None:
                self.initial_imu_yaw = self.imu_yaw
                rospy.loginfo("Fine initial IMU yaw=%.6f rad (%.2f deg)",
                              self.initial_imu_yaw, math.degrees(self.initial_imu_yaw))

    def _imu_ready(self):
        # Only initial reception matters; use the latest received yaw without
        # an IMU timeout or a stop/recovery state.
        return self.imu_yaw is not None and self.initial_imu_yaw is not None

    def _side_distance(self, side):
        if self.latest_distances is None:
            return None
        distance = self.latest_distances[1 if side == "left" else 2]
        if not math.isfinite(distance) or not 0 < distance <= self.side_wall_max_range:
            return None
        return distance

    def _lidar_ready(self, now):
        if self.latest_distances is None:
            return False
        age = (now - self.latest_distance_time).to_sec()
        front = self.latest_distances[0]
        return (0 <= age <= self.distance_timeout and math.isfinite(front)
                and 0 < front <= self.max_docking_range
                and self._side_distance(self.active_precision_goal[3]) is not None)

    def _complete_goal(self, source):
        if self.state not in ("COARSE", "FINE"):
            return
        self._set_state("COMPLETE")
        self.done_count += 1
        self.done_pub.publish(Int32(data=self.done_count))
        rospy.loginfo("Goal complete via %s; /target_done=%d", source, self.done_count)

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, value))

    @staticmethod
    def _normalize_angle(value):
        return math.atan2(math.sin(value), math.cos(value))

    @staticmethod
    def _command_ok(msg, max_linear, max_angular):
        values = (msg.linear.x, msg.linear.y, msg.linear.z,
                  msg.angular.x, msg.angular.y, msg.angular.z)
        if not all(math.isfinite(value) for value in values):
            return False
        return (abs(msg.linear.z) <= 1e-6 and abs(msg.angular.x) <= 1e-6
                and abs(msg.angular.y) <= 1e-6
                and math.hypot(msg.linear.x, msg.linear.y) <= max_linear + 1e-6
                and abs(msg.angular.z) <= max_angular + 1e-6)

    def _ordered_phases(self, returning=False):
        phases = (("LATERAL", "FRONT") if self.active_fine_order == "side_first"
                  else ("FRONT", "LATERAL"))
        if returning:
            return tuple("RETURN_" + phase for phase in reversed(phases))
        return phases

    def _bedside_p(self, now, current_yaw):
        return self._staged_wall_p(
            now, current_yaw, self.front_target_distance,
            self.side_target_distance)

    def _return_p(self, now, current_yaw):
        """Run each return axis once, in reverse order of fine docking."""
        first, second = self._ordered_phases(returning=True)
        if self.docking_phase not in (first, second):
            rospy.logerr("Unknown return phase %r; restarting at %s",
                         self.docking_phase, first)
            self._set_docking_phase(first)
            return Twist(), False

        if self.docking_phase.endswith("FRONT"):
            command, axis_done = self._return_front_p(now, current_yaw)
        else:
            command, axis_done = self._return_side_p(now, current_yaw)

        if not axis_done:
            return command, False
        if self.docking_phase == first:
            self._set_docking_phase(second)
            return Twist(), False
        return Twist(), True

    def _return_front_p(self, now, current_yaw):
        """Move 3 cm along the starting heading, measured in odom."""
        command = Twist()
        yaw_error = self._normalize_angle(self.initial_imu_yaw - current_yaw)
        yaw_ok = abs(math.degrees(yaw_error)) <= self.yaw_tolerance_deg
        if not yaw_ok:
            command.angular.z = self._clamp(self.kp_yaw * yaw_error,
                                            self.max_yaw_speed)
        try:
            transform = self.tf_buffer.lookup_transform(
                "odom", self.base_frame, rospy.Time(0), rospy.Duration(0.02))
            stamp = transform.header.stamp
            age = (now - stamp).to_sec()
            position = transform.transform.translation
            rotation = transform.transform.rotation
            values = (position.x, position.y, rotation.x, rotation.y,
                      rotation.z, rotation.w)
            quaternion_norm = sum(value * value for value in values[2:])
            if (not all(math.isfinite(value) for value in values)
                    or not 0.5 <= quaternion_norm <= 1.5
                    or not 0.0 <= age <= self.distance_timeout):
                raise ValueError("stale or non-finite odom pose")
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException, ValueError) as exc:
            self.stage_good_samples = 0
            rospy.logwarn_throttle(1.0, "Fixed 3 cm return waiting for odom TF: %s", exc)
            return Twist(), False

        if self.return_back_start is None:
            front_age = (now - self.latest_distance_time).to_sec()
            front = (self.latest_distances[0]
                     if self.latest_distances is not None else float("nan"))
            if (not 0.0 <= front_age <= self.distance_timeout
                    or not math.isfinite(front)
                    or not 0.0 < front <= self.max_docking_range):
                self.stage_good_samples = 0
                return Twist(), False
            self.return_front_direction = (
                -1 if front >= self.front_target_distance else 1)
            q = rotation
            heading = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self.return_back_start = (position.x, position.y, heading)
            self.return_back_last_stamp = stamp
            rospy.loginfo("Fixed return start: front=%.3f m, target=%.3f m, direction=%s",
                          front, self.front_target_distance,
                          "backward" if self.return_front_direction < 0 else "forward")
            return Twist(), False

        x0, y0, heading = self.return_back_start
        forward_displacement = ((position.x - x0) * math.cos(heading)
                                + (position.y - y0) * math.sin(heading))
        remaining = 0.030 - self.return_front_direction * forward_displacement
        if remaining > 0.0:
            self.stage_good_samples = 0
            command.linear.x = self.return_front_direction * min(
                self.max_front_speed, max(0.01, self.kp_front * remaining))
        elif not yaw_ok:
            self.stage_good_samples = 0
        elif stamp != self.return_back_last_stamp:
            self.stage_good_samples += 1
        self.return_back_last_stamp = stamp
        return command, self.stage_good_samples >= self.lateral_ready_samples

    def _return_side_p(self, now, current_yaw):
        """Return side distance to its saved fine-entry value once."""
        command = Twist()
        yaw_error = self._normalize_angle(self.initial_imu_yaw - current_yaw)
        yaw_ok = abs(math.degrees(yaw_error)) <= self.yaw_tolerance_deg
        if not yaw_ok:
            command.angular.z = self._clamp(self.kp_yaw * yaw_error,
                                            self.max_yaw_speed)
        if not self._lidar_ready(now):
            self.stage_good_samples = 0
            rospy.logwarn_throttle(1.0, "Return side waiting for valid wall distances")
            return command, False

        side = self.active_precision_goal[3]
        error = self._side_distance(side) - self.fine_entry_distances[1]
        side_sign = 1.0 if side == "left" else -1.0
        if abs(error) > self.side_tolerance:
            self.stage_good_samples = 0
            self.last_stage_distance_sequence = self.distance_sequence
            command.linear.y = self._clamp(side_sign * self.kp_side * error,
                                           self.max_side_speed)
        elif not yaw_ok:
            self.stage_good_samples = 0
        elif self.distance_sequence != self.last_stage_distance_sequence:
            self.last_stage_distance_sequence = self.distance_sequence
            self.stage_good_samples += 1
        return command, self.stage_good_samples >= self.lateral_ready_samples

    def _staged_wall_p(self, now, current_yaw, target_front, target_side):
        """Adjust one axis per stage; verify all axes after the second stage."""
        command = Twist()
        yaw_error = self._normalize_angle(self.initial_imu_yaw - current_yaw)
        yaw_error_deg = math.degrees(yaw_error)
        yaw_ok = abs(yaw_error_deg) <= self.yaw_tolerance_deg
        if not yaw_ok:
            command.angular.z = self._clamp(self.kp_yaw * yaw_error, self.max_yaw_speed)
        if not self._lidar_ready(now):
            self.stage_good_samples = 0
            self._reset_verification()
            rospy.logwarn_throttle(1.0, "Wall P waiting for valid wall distances")
            return command, False

        side = self.active_precision_goal[3]
        front_error = self.latest_distances[0] - target_front
        side_error = self._side_distance(side) - target_side
        side_sign = 1.0 if side == "left" else -1.0
        first, second = self._ordered_phases()
        verify = "VERIFY"
        errors = {
            "FRONT": (front_error, self.front_tolerance),
            "LATERAL": (side_error, self.side_tolerance),
        }

        def axis_ok(phase):
            error, tolerance = errors[phase]
            return abs(error) <= tolerance

        def drive_axis(phase):
            if phase == "FRONT":
                command.linear.x = self._clamp(self.kp_front * front_error,
                                               self.max_front_speed)
            else:
                command.linear.y = self._clamp(side_sign * self.kp_side * side_error,
                                               self.max_side_speed)

        if self.docking_phase == first:
            if not axis_ok(first):
                self.stage_good_samples = 0
                self.last_stage_distance_sequence = self.distance_sequence
                drive_axis(first)
            elif self.distance_sequence != self.last_stage_distance_sequence:
                self.last_stage_distance_sequence = self.distance_sequence
                self.stage_good_samples += 1
                if self.stage_good_samples >= self.lateral_ready_samples:
                    self._set_docking_phase(second)

        elif self.docking_phase == second:
            # Docking still performs the original all-axis verification.
            if not axis_ok(second):
                drive_axis(second)
            else:
                self._set_docking_phase(verify)

        elif self.docking_phase == verify:
            if not axis_ok(first):
                self._set_docking_phase(first)
                drive_axis(first)
            elif not axis_ok(second):
                self._set_docking_phase(second)
                drive_axis(second)
            elif not yaw_ok:
                self._reset_verification()
            else:
                if self.precision_tolerance_since is None:
                    self.precision_tolerance_since = now
                if self.distance_sequence != self.last_verify_distance_sequence:
                    self.last_verify_distance_sequence = self.distance_sequence
                    self.verification_good_samples += 1
                if (self.verification_good_samples >= self.verification_samples
                        and (now - self.precision_tolerance_since).to_sec()
                        >= self.precision_settle_time):
                    return command, True
        else:
            rospy.logerr("Unknown wall P phase %r; restarting at %s",
                         self.docking_phase, first)
            self._set_docking_phase(first)

        rospy.loginfo_throttle(
            0.5, "Wall P phase=%s order=%s side=%s: front_error=%.3f "
            "side_error=%.3f yaw_error=%.2f deg",
            self.docking_phase, self.active_fine_order, side,
            front_error, side_error, yaw_error_deg)
        return command, False

    def _reset_verification(self):
        self.precision_tolerance_since = None
        self.verification_good_samples = 0
        self.last_verify_distance_sequence = -1

    def _reset_docking_sequence(self):
        self.docking_phase = self._ordered_phases()[0]
        self.stage_good_samples = 0
        self.last_stage_distance_sequence = -1
        self._reset_verification()
        if hasattr(self, "phase_pub"):
            self.phase_pub.publish(String(data=self.docking_phase))

    def _set_docking_phase(self, phase):
        if phase == self.docking_phase:
            return
        old_phase = self.docking_phase
        self.docking_phase = phase
        self._publish_output()
        self.stage_good_samples = 0
        self.last_stage_distance_sequence = -1
        self._reset_verification()
        self.phase_pub.publish(String(data=phase))
        rospy.loginfo("Precision phase %s -> %s", old_phase, phase)


    def _handoff_blockers(self, now, robot_pose):
        """Evaluate every handoff condition, without short-circuit diagnostics."""
        reasons = []
        if robot_pose is None:
            reasons.append("无法取得有效的 %s -> %s TF坐标" % (
                self.global_frame, self.base_frame))
        else:
            distance = math.hypot(self.active_goal[0] - robot_pose[0],
                                  self.active_goal[1] - robot_pose[1])
            if distance > self.precision_radius:
                reasons.append("距候选点%.3fm，要求<=%.3fm" % (
                    distance, self.precision_radius))
        elapsed = (now - self.goal_started).to_sec()
        if elapsed < self.mode_guard_time:
            reasons.append("收到目标仅%.3fs，要求>=%.3fs" % (
                elapsed, self.mode_guard_time))
        if self.latest_distances is None:
            reasons.append("尚未收到包含前/左/右三个距离的有效 %s 数据" % self.distance_topic)
        else:
            age = (now - self.latest_distance_time).to_sec()
            if not 0 <= age <= self.distance_timeout:
                reasons.append("雷达数据时间间隔%.3fs，要求0~%.3fs" % (
                    age, self.distance_timeout))
            front = self.latest_distances[0]
            if not math.isfinite(front) or not 0 < front <= self.max_docking_range:
                reasons.append("前方距离%s m，要求有效且0<距离<=%.3fm" % (
                    str(front), self.max_docking_range))
            side = self.active_precision_goal[3]
            side_value = self.latest_distances[1 if side == "left" else 2]
            if self._side_distance(side) is None:
                reasons.append("%s距离%s m，要求有效且0<距离<=%.3fm" % (
                    "左方" if side == "left" else "右方", str(side_value),
                    self.side_wall_max_range))
        if not self._imu_ready():
            reasons.append("尚未收到有效的 /handsfree/imu 姿态，初始Yaw未建立")
        return reasons

    def _control(self, _event):
        now = rospy.Time.now()
        with self.lock:
            if self.state not in ("COARSE", "FINE", "RETURN"):
                self._publish_output()
                return
            robot_pose = None
            if self.state == "COARSE" and self.active_precision_goal is not None:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.global_frame, self.base_frame, rospy.Time(0),
                        rospy.Duration(0.02))
                    p = transform.transform.translation
                    if not all(math.isfinite(value) for value in (p.x, p.y)):
                        raise ValueError("non-finite TF")
                    robot_pose = (p.x, p.y)
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException, ValueError):
                    rospy.logwarn_throttle(1.0, "Bed handoff waiting for TF position")

            if self.state == "COARSE":
                blockers = (self._handoff_blockers(now, robot_pose)
                            if self.active_precision_goal is not None else None)
                if blockers is not None and not blockers:
                    # Capture once, immediately before the first fine command.
                    # This snapshot remains unchanged throughout docking/arm.
                    self.fine_entry_distances = (
                        self.latest_distances[0],
                        self._side_distance(self.active_precision_goal[3]))
                    rospy.loginfo("Saved %s fine entry: front=%.3f side=%.3f",
                                  self.active_precision_goal[0], *self.fine_entry_distances)
                    self.mode = "pid"
                    self._reset_docking_sequence()
                    self.mode_pub.publish(String(data=self.mode))
                    self._set_state("FINE")
                    rospy.loginfo("%s，进入细导航：床位=%s，候选点=%d，顺序=%s",
                                  "粗导航到达成功" if self.coarse_arrival_reported else
                                  "已进入粗导航目标切换范围，条件全部通过",
                                  self.active_precision_goal[0], self.active_candidate_index,
                                  self.active_fine_order)
                    return  # Explicit zero output on handoff.
                if blockers:
                    if self.coarse_arrival_reported:
                        rospy.logwarn_throttle(
                            2.0, "粗导航已报告到达，但进入细导航失败（继续等待）：%s",
                            "；".join(blockers))
                    else:
                        rospy.loginfo_throttle(
                            2.0, "粗导航中，尚未进入细导航，未通过条件：%s",
                            "；".join(blockers))
                age = (now - self.last_a_time).to_sec()
                if (self.last_a_time == rospy.Time(0)
                        or self.last_a_time < self.goal_started
                        or not 0 <= age <= self.cmd_timeout):
                    self._publish_output()
                elif self._command_ok(self.last_a, self.coarse_max_linear, self.coarse_max_angular):
                    self._publish_output(self.last_a)
                else:
                    self._publish_output()
                    rospy.logwarn_throttle(
                        1.0, "Invalid or over-limit B-spline command; waiting for valid command")
                return

            # Docking and return both use B output and the same fixed P gains.
            returning = self.state == "RETURN"
            command, verified = (self._return_p(now, self.imu_yaw) if returning
                                 else self._bedside_p(now, self.imu_yaw))
            if not self._command_ok(command, self.fine_max_linear, self.fine_max_angular):
                self._reset_verification()
                self._publish_output()
                rospy.logwarn_throttle(
                    1.0, "Invalid or over-limit fine command; waiting for valid command")
            elif verified:
                if returning:
                    self._complete_return()
                else:
                    self._complete_goal("staged_wall_p")
            else:
                self._publish_output(command)

    def _stop(self):
        for _ in range(5):
            self._publish_output()


if __name__ == "__main__":
    rospy.init_node("bedside_navigation_controller")
    BedsideNavigationController()
    rospy.spin()


