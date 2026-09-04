# NavfnRuckig医疗机器人路径规划器

## 概述
NavfnRuckig是一个为医疗机器人设计的路径规划器，集成了NavfnROS全局规划和ruckig轨迹生成技术。

**主要特性：**
- ✅ 使用NavfnROS作为全局路径规划器
- ✅ 使用ruckig进行轨迹生成（支持加速度约束）
- ✅ **速度分段加速度限制**（参数化配置，无需重新编译）
- ✅ 阈值化重规划（减少路径抖动）
- ✅ 复用原有路径跟随算法
- ✅ 适配medical_race底盘和jie_ware定位系统

## 系统架构

```
医疗机器人导航系统
├── 底盘系统: medical_race/chassis_all_run.launch (全向移动)
├── 定位系统: jie_ware/robust_lidar_test.launch (鲁棒激光雷达定位)  
├── 全局规划: move_base + NavfnROS
└── 局部规划: NavfnRuckig (Python + ruckig)
    ├── 速度分段加速度限制
    ├── 阈值化重规划
    └── 轨迹跟随控制
```

## 核心功能：速度分段加速度限制

### 工作原理
根据机器人当前速度，动态调整ruckig的加速度限制参数：

```yaml
velocity_acceleration_limits:
  # [速度下限, 速度上限, x加速度限制, y加速度限制, 角加速度限制]
  - [0.0, 0.5, 0.2, 0.2, 0.5]  # 静止到低速：保守加速度
  - [0.5, 1.0, 0.3, 0.3, 0.8]  # 中速区间：平衡性能
  # 可继续添加更多速度区间...
```

### 加速度控制机制
1. **实时监测**：持续监测机器人当前速度
2. **区间匹配**：根据当前速度查找对应的加速度限制区间
3. **参数更新**：将加速度限制传递给ruckig轨迹生成器
4. **轨迹生成**：ruckig生成满足约束的平滑轨迹

## 安装和依赖

### Python依赖
```bash
# 安装ruckig Python版本
pip install ruckig

# 验证安装
python3 -c "import ruckig; print('ruckig安装成功')"
```

### ROS依赖
- navfn (NavfnROS全局规划器)
- move_base (导航框架)
- tf (坐标变换)
- 现有的roborts_msgs

## 使用方法

### 1. 完整启动（推荐）
```bash
# 启动底盘和定位
roslaunch medical_race chassis_all_run.launch car_mode:=senior_omni
roslaunch jie_ware robust_lidar_test.launch

# 启动NavfnRuckig规划器
roslaunch auto_nav navfn_ruckig_medical_robot.launch
```

### 2. 单独启动规划器
```bash
# 仅启动规划器（假设底盘和定位已启动）
roslaunch auto_nav navfn_ruckig_planner.launch
```

### 3. 参数调整
编辑配置文件（无需重新编译）：
```bash
vim config/navfn_ruckig_config.yaml
# 或者直接在launch文件中修改参数
```

## 参数配置详解

### 速度和加速度参数
```yaml
# 基础速度参数
max_velocity: 1.0              # 整体最大速度 [m/s]
plan_frequency: 20             # 规划频率 [Hz]

# 核心功能：速度分段加速度限制
velocity_acceleration_limits:
  - [0.0, 0.5, 0.2, 0.2, 0.5]  # 低速区间：平稳启动
  - [0.5, 1.0, 0.3, 0.3, 0.8]  # 中速区间：平衡性能
```

### 阈值化重规划参数
```yaml
enable_path_threshold: true          # 是否启用
path_distance_threshold: 0.3         # 距离阈值 [m]
path_angle_threshold: 0.52           # 角度阈值 [rad] (~30度)
path_update_min_interval: 1.0        # 时间阈值 [s]
```

### ruckig轨迹生成参数
```yaml
trajectory_time_step: 0.1            # 轨迹时间步长 [s]
trajectory_duration: 3.0             # 轨迹最大时长 [s]
```

## 调试和优化

### 常见问题排查

**1. 启动过慢**
```yaml
# 增大低速区间的加速度限制
- [0.0, 0.5, 0.3, 0.3, 0.5]  # 0.2 -> 0.3
```

**2. 高速震荡**
```yaml
# 减小高速区间的加速度限制
- [0.5, 1.0, 0.25, 0.25, 0.8]  # 0.3 -> 0.25
```

**3. 响应不够敏捷**
```yaml
# 减小轨迹时间步长
trajectory_time_step: 0.05      # 0.1 -> 0.05
```

**4. ruckig不可用**
系统会自动回退到简化轨迹生成，在日志中查看：
```
[WARN] 未找到ruckig Python库，将使用简化轨迹生成
```

### 性能监控
```bash
# 查看规划器状态
rostopic echo /navfn_ruckig_planner/local_path

# 监控速度指令
rostopic echo /cmd_vel

# 调试模式（详细日志）
rosparam set /navfn_ruckig_planner/debug_mode true
```

## 话题和服务

### 订阅话题
- `/move_base/NavfnROS/plan` (nav_msgs/Path) - 全局路径
- `/tf` - 坐标变换

### 发布话题
- `/cmd_vel` (geometry_msgs/Twist) - 速度指令
- `/navfn_ruckig_planner/local_path` (nav_msgs/Path) - 局部路径

### 服务
- `/navfn_ruckig_planner/planner_status` - 规划器状态控制

## 与原系统的兼容性

| 功能 | 原adaptive_pid | NavfnRuckig | 状态 |
|------|---------------|-------------|------|
| 速度自适应 | ✅ P值调整 | ✅ 加速度限制 | ✅ 升级 |
| 阈值化重规划 | ✅ | ✅ | ✅ 复用 |
| 路径跟随 | ✅ | ✅ | ✅ 复用 |
| 全向移动支持 | ✅ | ✅ | ✅ 兼容 |
| 参数化配置 | ✅ | ✅ | ✅ 改进 |

## 开发路线图

- [x] **阶段1**: 基础框架 + 速度分段加速度限制
- [ ] **阶段2**: 更细分的速度区间和转弯半径限制  
- [ ] **阶段3**: 障碍物避让和轨迹评分
- [ ] **阶段4**: 恢复行为和异常处理
- [ ] **阶段5**: 性能优化和完整ROS插件接口

## 技术支持

如有问题或建议，请检查：
1. **日志输出**: `rosnode info navfn_ruckig_planner`
2. **参数配置**: `rosparam list | grep navfn_ruckig`
3. **依赖检查**: `python3 -c "import ruckig"`
4. **话题连接**: `rostopic list | grep -E "(plan|cmd_vel)"`



