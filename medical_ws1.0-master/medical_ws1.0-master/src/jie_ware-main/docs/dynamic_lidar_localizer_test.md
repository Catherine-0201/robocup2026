# Dynamic lidar localizer test

`dynamic_lidar_localizer` is an independent experimental executable.  It does
not change `lidar_loc`, `lidar_loc_copy`, `dynamic_obstacle_filter.py`, or
`lidar_loc_test.launch`.  Never run it together with the old localizer because
both would publish `map -> odom`.

## Build and start

```bash
cd ~/medical_ws
catkin_make
source devel/setup.bash
roslaunch jie_ware dynamic_lidar_loc_test.launch
```

Give an initial pose in RViz or with the existing `set_initial_pose.py` node.
Inspect `/scan_static_for_loc` (used/near-map endpoints),
`/scan_dynamic_removed` (map-inconsistent endpoints), and
`/dynamic_lidar_loc/quality`.

The dynamic mask uses two checks together: endpoint distance to the static
map, and ray casting.  If a laser return arrives noticeably before the wall
predicted along that beam, it is treated as foreground and excluded even when
the person is standing close to the wall.  Tune
`raycast_foreground_margin_m` from bags; making it too small can reject walls
when the initial pose or map is inaccurate.

## A/B bags

Record the following once per test run:

```bash
rosbag record -O dynamic_case.bag /scan /map /tf /tf_static /initialpose /cmd_vel
```

For each bag, replay the old launch and the new launch separately, from the
same initial pose.  Do not record or replay a competing `map -> odom` source.

Run at least ten repeats for: empty/static space; one person 0.2, 0.5, and
1.0 m from the vehicle; a person crossing the scan; max-speed straight motion;
max-speed turning; and max speed with a nearby person.

Compare: accepted/rejected count, `static_inlier_ratio`, pose jump count
(`map->odom` translation > 0.10 m or yaw > 3 degrees per scan), maximum pose
error at surveyed stop points, recovery time after the person leaves, and CPU
time.  Tune only the YAML thresholds after preserving the baseline bags.
