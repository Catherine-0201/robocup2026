#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Publish robust front/left/right distances from a 2-D LaserScan.

Public interface (unchanged):
    /lidar_distances: Float32MultiArray [front, left, right]
"""

import math
from collections import deque

import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray


class LidarLineDistanceNode:
    DIRECTIONS = (
        ("front", 0.0),
        ("left", math.pi / 2.0),
        ("right", -math.pi / 2.0),
    )

    def __init__(self):
        rospy.init_node("lidar_line_distance_node", anonymous=True)

        self.sector_half_angle = math.radians(float(
            rospy.get_param("~sector_half_angle_deg", 12.0)))
        self.minimum_range = float(rospy.get_param("~minimum_range", 0.20))
        self.maximum_range = float(rospy.get_param("~maximum_range", 1.50))
        self.min_valid_points = int(rospy.get_param("~min_valid_points", 5))
        self.minimum_outlier_band = float(
            rospy.get_param("~minimum_outlier_band", 0.05))
        self.mad_scale = float(rospy.get_param("~mad_scale", 3.5))
        self.temporal_window = max(
            1, int(rospy.get_param("~temporal_window", 3)))
        self.invalid_hold_time = max(
            0.0, float(rospy.get_param("~invalid_hold_time", 0.20)))

        self.history = {
            name: deque(maxlen=self.temporal_window)
            for name, _ in self.DIRECTIONS
        }
        self.last_valid_value = {
            name: float("inf") for name, _ in self.DIRECTIONS
        }
        self.last_valid_time = {
            name: rospy.Time(0) for name, _ in self.DIRECTIONS
        }

        self.distances_pub = rospy.Publisher(
            "/lidar_distances", Float32MultiArray, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)

        rospy.loginfo(
            "Robust lidar distances ready: sector=+/-%.1f deg, min_points=%d, "
            "range=[%.2f, %.2f] m",
            math.degrees(self.sector_half_angle), self.min_valid_points,
            self.minimum_range, self.maximum_range)

    @staticmethod
    def _angle_difference(angles, target):
        return np.arctan2(np.sin(angles - target), np.cos(angles - target))

    def _sector_distance(self, scan_data, ranges, angles, target_angle):
        delta = self._angle_difference(angles, target_angle)
        lower_range = max(float(scan_data.range_min), self.minimum_range)
        upper_range = min(float(scan_data.range_max), self.maximum_range)
        valid = (
            np.isfinite(ranges)
            & (ranges >= lower_range)
            & (ranges <= upper_range)
            & (np.abs(delta) <= self.sector_half_angle)
        )
        if int(np.count_nonzero(valid)) < self.min_valid_points:
            return float("inf")

        # Project every ray onto the centre axis of the sector.  For a wall,
        # this is approximately its perpendicular distance from the lidar.
        projected = ranges[valid] * np.cos(delta[valid])
        projected = projected[np.isfinite(projected) & (projected > 0.0)]
        if projected.size < self.min_valid_points:
            return float("inf")

        median = float(np.median(projected))
        mad = float(np.median(np.abs(projected - median)))
        robust_sigma = 1.4826 * mad
        band = max(self.minimum_outlier_band, self.mad_scale * robust_sigma)
        inliers = projected[np.abs(projected - median) <= band]
        if inliers.size < self.min_valid_points:
            return float("inf")
        return float(np.median(inliers))

    def _temporally_filter(self, name, value, now):
        if math.isfinite(value) and value > 0.0:
            self.history[name].append(value)
            filtered = float(np.median(np.asarray(self.history[name])))
            self.last_valid_value[name] = filtered
            self.last_valid_time[name] = now
            return filtered

        if (self.last_valid_time[name].to_sec() > 0.0
                and (now - self.last_valid_time[name]).to_sec()
                <= self.invalid_hold_time):
            return self.last_valid_value[name]

        self.history[name].clear()
        return float("inf")

    def scan_callback(self, scan_data):
        count = len(scan_data.ranges)
        if count == 0 or scan_data.angle_increment == 0.0:
            return

        ranges = np.asarray(scan_data.ranges, dtype=np.float64)
        angles = (float(scan_data.angle_min)
                  + np.arange(count, dtype=np.float64)
                  * float(scan_data.angle_increment))
        now = rospy.Time.now()

        output = []
        for name, target_angle in self.DIRECTIONS:
            raw_value = self._sector_distance(
                scan_data, ranges, angles, target_angle)
            output.append(self._temporally_filter(name, raw_value, now))

        message = Float32MultiArray()
        message.data = output
        self.distances_pub.publish(message)

        rospy.loginfo_throttle(
            1.0, "Lidar distances: front=%s left=%s right=%s",
            "%.3f" % output[0] if math.isfinite(output[0]) else "inf",
            "%.3f" % output[1] if math.isfinite(output[1]) else "inf",
            "%.3f" % output[2] if math.isfinite(output[2]) else "inf")

    @staticmethod
    def run():
        rospy.spin()


if __name__ == "__main__":
    try:
        LidarLineDistanceNode().run()
    except rospy.ROSInterruptException:
        pass
