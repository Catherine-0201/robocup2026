#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Wall-referenced staged bedside PID controller.

This keeps the staged controller and strict A/B command interfaces unchanged,
but measures the selected side from GetDistanceByLidarLine.py instead of
ray-casting the static occupancy map.  /lidar_distances is expected to contain
[front, left, right].
"""

import math

import rospy

from hybrid_precision_mux_staged import HybridPrecisionMux


class WallReferencedHybridPrecisionMux(HybridPrecisionMux):
    def __init__(self):
        self.side_distance_label = "side_wall_lidar"
        self.side_wall_max_range = float(
            rospy.get_param("~side_wall_max_range", 2.50))
        if not math.isfinite(self.side_wall_max_range) or self.side_wall_max_range <= 0.0:
            raise rospy.ROSInitException("side_wall_max_range must be finite and > 0")
        super().__init__()
        rospy.loginfo(
            "Wall-referenced side PID enabled: target=%.3f m, max=%.3f m",
            self.side_target_distance, self.side_wall_max_range)

    def _map_side_distance(self, robot_x, robot_y, robot_yaw, side):
        """Return the fitted lidar-wall distance on the selected bed side."""
        del robot_x, robot_y, robot_yaw
        if self.latest_distances is None:
            return None

        index = 1 if side == "left" else 2
        distance = float(self.latest_distances[index])
        if (not math.isfinite(distance) or distance <= 0.0 or
                distance > self.side_wall_max_range):
            rospy.logwarn_throttle(
                1.0,
                "Selected %s wall is not valid on %s: distance=%s, max=%.2f",
                side, self.distance_topic, str(distance),
                self.side_wall_max_range)
            return None
        return distance


if __name__ == "__main__":
    rospy.init_node("hybrid_precision_mux_staged_wall")
    WallReferencedHybridPrecisionMux()
    rospy.spin()
