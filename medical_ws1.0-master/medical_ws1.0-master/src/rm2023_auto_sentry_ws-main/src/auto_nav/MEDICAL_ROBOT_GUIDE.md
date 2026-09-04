# 医疗机器人导航系统配置指南

## 参数详解

### 1. inflation_radius 详解

在costmap配置中有两个不同的inflation_radius：

#### obstacle_layer.inflation_radius (0.2m)
- **作用**: 对激光雷达检测到的**动态障碍物**进行膨胀
- **设置原则**: 应等于机器人半径，确保机器人不会碰到动态障碍物
- **您的机器人**: 传感器在中心，建议设置为机器人实际半径

#### inflation_layer.inflation_radius (0.8m → 建议0.4-0.6m)
- **作用**: 对所有障碍物创建**渐变代价区域**，用于路径规划
- **设置原则**: 可以大于机器人半径，提供安全缓冲区
- **建议值**: 机器人半径的2-3倍（0.4-0.6m）

### 2. TEB局部规划器关键参数

#### 轨迹参数
```yaml
dt_ref: 0.3                    # 轨迹时间分辨率（秒），越小轨迹越平滑
max_global_plan_lookahead_dist: 0.8  # 向前看距离，决定跟踪范围
global_plan_viapoint_sep: 0.25       # 路径点间距
```

#### 速度和加速度限制
```yaml
max_vel_x: 1.0          # 最大前进速度 (m/s)
max_vel_y: 0.8          # 最大横移速度 (m/s) 
max_vel_theta: 2.0      # 最大角速度 (rad/s)
acc_lim_x: 1.5          # 最大前进加速度 (m/s²)
acc_lim_y: 1.5          # 最大横移加速度 (m/s²)
acc_lim_theta: 2.0      # 最大角加速度 (rad/s²)
holonomic_robot: True   # 全向机器人标志
```

#### 目标容差
```yaml
xy_goal_tolerance: 0.2   # 位置到达精度 (m)
yaw_goal_tolerance: 0.3  # 角度到达精度 (rad)
```

#### 障碍物避障
```yaml
min_obstacle_dist: 0.4   # 与障碍物最小距离 (m)
inflation_dist: 0.5      # 障碍物缓冲区大小 (m)
```

#### 优化权重（数值越大约束越强）
```yaml
weight_max_vel_x: 15.0      # 前进速度约束权重
weight_max_vel_y: 15.0      # 横移速度约束权重  
weight_max_vel_theta: 800.0 # 角速度约束权重
weight_obstacle: 150        # 障碍物避障权重
weight_viapoint: 100        # 路径跟踪权重
```

### 3. move_base参数解释

#### 第24-28行参数
```xml
<param name="base_global_planner" value="global_planner/GlobalPlanner" />
<!-- 全局路径规划器，使用A*或Dijkstra算法 -->

<param name="planner_frequency" value="3.0" />
<!-- 全局规划器运行频率 (Hz)，3Hz表示每秒规划3次 -->

<param name="planner_patience" value="5.0" />
<!-- 全局规划超时时间 (秒)，5秒内无法规划则报错 -->

<param name="use_dijkstra" value="false" />
<!-- false=使用A*算法, true=使用Dijkstra算法 -->
```

#### 第34行参数
```xml
<param name="clearing_rotation_allowed" value="false" />
<!-- 禁止清除代价地图时的旋转动作，医疗机器人应保持稳定 -->
```

#### 第16行 - move_base包
- **包名**: `move_base`
- **作用**: ROS导航栈的核心包，协调全局规划和局部规划
- **功能**: 
  - 接收目标点
  - 调用全局规划器生成路径
  - 调用局部规划器进行实时避障
  - 发布速度命令

### 4. 医疗机器人PID控制器修改

#### 移除的功能
- ✅ **云台控制**: 移除所有gimbal相关代码
- ✅ **游戏状态**: 移除RoboMaster比赛状态检查
- ✅ **小陀螺**: 到达目标后停止，不进行自转
- ✅ **关节状态**: 移除云台关节信息订阅

#### 简化的状态机
```cpp
// 规划状态定义
planner_state_ = 0;  // 静止状态
planner_state_ = 1;  // 原地旋转（如需要）
planner_state_ = 2;  // 路径跟踪状态（默认）
```

#### 关键修改点
```cpp
// 1. 到达目标后停止
cmd_vel.angular.z = 0;  // 不进行小陀螺

// 2. 路径跟踪时自然转向
cmd_vel.angular.z = 0;  // 移除额外角速度

// 3. 简化状态检查
if (planner_state_ == 2)  // 只检查规划状态，不检查游戏状态
```

## 使用步骤

### 1. 编译
```bash
cd /home/jetson/code_files/robocup/medical_ws
catkin_make
source devel/setup.bash
```

### 2. 启动顺序
```bash
# 终端1: 启动底盘
roslaunch medical_race chassis_all_run.launch

# 终端2: 启动定位
roslaunch jie_ware lidar_loc_test.launch

# 终端3: 启动导航
roslaunch auto_nav test_navi_simple_meca_car_pid.launch
```

### 3. 控制机器人状态
```bash
# 设置为路径跟踪模式
rosservice call /pid_planner_status "planner_state: 2
max_x_speed: 0.5
max_y_speed: 0.5  
yaw_speed: 0.0"

# 设置为静止模式
rosservice call /pid_planner_status "planner_state: 0
max_x_speed: 0.0
max_y_speed: 0.0
yaw_speed: 0.0"
```

### 4. 发送导航目标
在RViz中使用"2D Nav Goal"工具点击目标位置

## 调试建议

### 检查话题连接
```bash
# 检查速度命令
rostopic echo /cmd_vel

# 检查全局路径
rostopic echo /move_base/GlobalPlanner/plan

# 检查tf变换
rosrun tf view_frames
```

### 参数调优建议
1. **速度过慢**: 增加`max_x_speed`, `max_y_speed`参数
2. **转向不灵敏**: 调整PID的`p_value`参数
3. **避障过于保守**: 减小`inflation_radius`和`min_obstacle_dist`
4. **路径跟踪不准确**: 调整`goal_dist_tolerance`和PID参数

### 常见问题排查
1. **机器人不动**: 检查话题重映射和planner_state
2. **路径规划失败**: 检查地图加载和起始位置设置
3. **避障异常**: 检查激光雷达数据和costmap配置
4. **坐标变换错误**: 检查tf树完整性

## 文件结构
```
auto_nav/
├── launch/
│   ├── test_navi_simple_meca_car_pid.launch     # 主导航launch
│   └── test_pid_follow_planner.launch           # PID控制器launch
├── src/
│   └── test_pid_position_follow.cpp             # 医疗机器人PID控制器
├── config/point_pid_params/test_medical_params/
│   ├── test_costmap_common_params.yaml          # 通用代价地图参数
│   ├── test_global_costmap_params.yaml          # 全局代价地图参数
│   ├── test_local_costmap_params.yaml           # 局部代价地图参数
│   └── test_teb_local_planner_params.yaml       # TEB规划器参数
└── CMakeLists.txt                               # 编译配置
```
