# 送药巡诊机器人：导航与定位总控

这是一个先把“导航 + 定位 + 比赛时序”跑通的版本。扫码、机械臂和药箱动作暂不执行，但已留下 ROS 接口，后续可以逐项接入，不需要重写总控状态机。

## 已覆盖的比赛流程

1. 等待定位稳定；
2. 等待实体启动按钮发出 `/competition/start=true`；
3. 导航到护士站；
4. 读取任务码（当前可用参数模拟，支持 11、13、31、33）；
5. 按任务码依次去 1 号床和 3 号床；
6. 到床位后停车并发布本次床号、药箱号，暂不执行机械臂；
7. 预留至少 35 秒返航，回到起点后持续停车超过 5 秒；
8. 支持紧急停止、导航失败重试、清理代价地图，以及 ICP 失效后的短时后备定位。

## 本次完善的关键点

- 新目标下发前先把PID跟踪器切换到静止，避免新路径尚未生成时继续追踪上一条路径；
- 下发目标后等待0.6秒，再启用PID跟踪器；
- 启动前检查`move_base`和`/pid_planner_status`是否可用，依赖缺失时保持停车；
- `/target_done`必须在本次目标下发后递增，避免使用上一次导航留下的旧到点信号；
- 到点必须同时满足“到点信号”和`/unified_pose`位置误差，默认禁止仅凭距离误判完成；
- 扫码结果默认要求在到达护士台、发出扫码请求后新收到，避免沿用上一次比赛的旧二维码；
- 修正床位动作完成事件的清除顺序，避免动作节点快速响应时丢失`station_done`；
- 返航后只调用一次停止服务，并持续发布零速度超过5秒；
- 所有异常退出和程序崩溃路径都会取消导航、关闭PID运动并发布零速度；
- 导航结果会写入`/competition/task_context`，包括位置误差、航向误差和到点信号来源。

任务码映射严格按规则：

| 任务码 | 第一次 | 第二次 |
|---|---|---|
| 11 | 1号床 / 1号药箱 | 3号床 / 3号药箱 |
| 13 | 1号床 / 3号药箱 | 3号床 / 1号药箱 |
| 31 | 3号床 / 1号药箱 | 1号床 / 3号药箱 |
| 33 | 3号床 / 3号药箱 | 1号床 / 1号药箱 |

## 放入工作空间

把本目录中的 `medical_race` 合并到仓库的 `src/medical_race`。其中 `CMakeLists.txt` 和 `package.xml` 已按当前仓库版本补好依赖与安装规则：

- `scripts/medical_competition_master.py`
- `config/competition_waypoints.yaml`
- `launch/competition_master.launch`
- `CMakeLists.txt`
- `package.xml`

在 Ubuntu/ROS 电脑上给脚本执行权限，然后重新编译当前工作空间。若你的工程目录是 `/home/jetson/code_files/robocup_copy/medical_ws`，可按下面方式执行：

```bash
chmod +x /home/jetson/code_files/robocup_copy/medical_ws/src/medical_race/scripts/medical_competition_master.py
cd /home/jetson/code_files/robocup_copy/medical_ws
catkin_make
source devel/setup.bash
```

## 启动顺序

不要再启动仓库中的 `newtask.py`、`newtask_test.py` 或 `newtask_all.py`，否则两个总控会同时下发导航命令。

先按机器人原有方式启动底盘、雷达和 IMU，然后分别启动：

```bash
# 终端1：定位（发布 /unified_pose 和 /localization_status）
roslaunch jie_ware lidar_loc_test.launch

# 终端2：move_base + 仓库自定义PID跟踪器
roslaunch auto_nav test_navi_simple_meca_car_pid.launch

# 终端3：新总控，当前用任务码11模拟二维码结果
roslaunch medical_race competition_master.launch default_task_code:=11
```

如果你的拷贝目录是 `/home/jetson/code_files/robocup_copy/medical_ws`，请以当前工作区里实际可成功运行的 ROS 包名为准。当前这份拷贝中，包定义文件 `package.xml` 里声明的包名仍是 `jie_ware`，因此这里按 `jie_ware` 运行即可；如果你后续修改了 `package.xml` 的 `<name>`，启动命令也要同步改成新的包名。

## 测试与比赛启动

台架测试可以暂时自动开始：

```bash
roslaunch medical_race competition_master.launch auto_start:=true default_task_code:=13
```

默认会在护士台等待5秒获取新的`/competition/task_code`。仅测试导航、不运行扫码节点时，可立即使用模拟值：

```bash
roslaunch medical_race competition_master.launch \
  auto_start:=true default_task_code:=13 task_code_wait_seconds:=0.0
```

正式比赛必须使用实体按钮。按钮/GPIO节点只需发布一次：

```bash
rostopic pub -1 /competition/start std_msgs/Bool "data: true"
```

上面的命令只用于联调，比赛中不能用键盘代替按钮。

紧急停车接口：

```bash
rostopic pub -1 /competition/emergency_stop std_msgs/Bool "data: true"
```

## 后续接扫码与机械臂

- 扫码节点识别出 11、13、31 或 33 后，向 `/competition/task_code` 发布 `std_msgs/Int32`。同时把启动参数 `task_code_wait_seconds` 调成扫码允许的等待时间，例如 `5.0`。
- 到床位后，总控向 `/competition/task_context` 发布 JSON，含任务码、访问顺序、床号和药箱号。
- 机械臂流程完成后，向 `/competition/station_done` 发布 `std_msgs/Bool(true)`；届时启动时设置 `skip_station_actions:=false`。

## 必须在赛场复核的内容

`competition_waypoints.yaml` 中的坐标来自仓库旧脚本，只能作为初值。必须在正式地图上重新记录护士站、1号床、3号床和起点的 `/unified_pose`。

特别注意：规则要求机器人本体完整进入床旁半径 300 mm 的圆。单看机器人中心点到达并不等价于本体完全入圈。需要根据底盘外形尺寸设置目标点，并现场验证自定义 PID 控制器的停止误差。当前控制器还会用 `unified_pose` 检查最终中心位置，但无法代替底盘外廓检查。

当前自适应PID控制器在`test_adaptive_pid_follow_planner.launch`中的停止阈值为`0.25 m`。因此总控中的床位粗导航容差不能设置成小于`0.25 m`，否则可能出现“PID已经停车并发布到点信号，但总控认为距离仍不合格，最终等待超时”。本版本把床位粗导航容差设为`0.28 m`，它只代表进入床位附近；要满足半径300 mm圆圈规则，必须在床位动作模块中继续执行视觉/激光精确停靠，再发布`/competition/station_done=true`。

如果以后把自适应PID控制器的停止阈值调小，总控的`competition_waypoints.yaml`容差也应同步修改，并确保：

```text
总控容差 >= PID停止阈值
精确停靠中心误差 + 底盘外接圆半径 <= 0.30 m
```

## 主要接口

| 方向 | 话题 | 类型 | 用途 |
|---|---|---|---|
| 输入 | `/localization_status` | `std_msgs/Bool` | ICP定位是否可靠 |
| 输入 | `/unified_pose` | `geometry_msgs/PoseWithCovarianceStamped` | 激光/后备统一位姿 |
| 输入 | `/target_done` | `std_msgs/Int32` | 仓库PID控制器的累计到点计数 |
| 输入 | `/competition/start` | `std_msgs/Bool` | 实体启动按钮 |
| 输入 | `/competition/task_code` | `std_msgs/Int32` | 后续扫码结果 |
| 输入 | `/competition/station_done` | `std_msgs/Bool` | 后续机械臂完成信号 |
| 输入 | `/competition/emergency_stop` | `std_msgs/Bool` | 急停 |
| 输出 | `/competition/state` | `std_msgs/String` | 当前总控状态 |
| 输出 | `/competition/current_station` | `std_msgs/String` | 当前目标站点 |
| 输出 | `/competition/task_context` | `std_msgs/String` | 后续动作所需任务上下文 |
| 输出 | `/competition/finished` | `std_msgs/Bool` | 完赛标志 |

`/competition/task_context`还会发布两类请求：

- `event=request_task_scan`：到达护士台后，请扫码节点读取新的任务码；
- `event=navigation_arrived`：粗导航到点，包含`station`、`distance_error`、`yaw_error`和`arrival_source`。

床位动作节点收到带`bed_id`和`medicine_box`的上下文后，应按“精确停靠→条码→开箱→放药→播报→巡诊”顺序执行，全部完成后再发布`/competition/station_done=true`。

## 定位失效策略

初始定位必须可靠才允许开始。行驶中 `/localization_status=false` 时，总控允许仓库现有 `/unified_pose` 后备链路短时继续工作，默认最多 10 秒；超过后停车等待重新定位 8 秒，再重发目标。这样既能穿过短暂高遮挡，又不会让 IMU/里程计积分误差无限累积。具体 10 秒应结合 IMU 误差测试结果调整。
