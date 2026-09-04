# Robust AMCL A/B test

This test uses the upstream ROS1 AMCL implementation with
`likelihood_field_prob` and beam skipping.  It is independent of the custom
localizers.

Only one node may broadcast `map -> odom`.  Stop `lidar_loc`,
`lidar_loc_copy`, `dynamic_lidar_localizer`, and localization-monitor fallback
TF before starting this launch.

```bash
rospack find amcl
roslaunch jie_ware amcl_robust_test.launch
```

If another process already provides the correct `/map`:

```bash
roslaunch jie_ware amcl_robust_test.launch start_map_server:=false
```

In RViz set Fixed Frame to `map`, add `/map`, `/scan`, `/particlecloud`, and
`/amcl_pose`, then use **2D Pose Estimate**.  Test the same route in an empty
field and with the competition boxes.  Record `/scan`, `/odom`, `/tf`,
`/tf_static`, `/amcl_pose`, and `/particlecloud` for comparison.

The navigation costmaps must continue to consume raw `/scan`; beam skipping
only affects localization and must not hide boxes from collision avoidance.
