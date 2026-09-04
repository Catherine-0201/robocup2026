# 测试导航系统使用说明

## 概述
此测试导航系统是为了将rm2023_auto_sentry_ws-main的导航功能适配到medical_race底盘和jie_ware定位系统而创建的。

## 文件结构

### 主要Launch文件
- `test_navi_simple_meca_car_pid.launch` - 主导航launch文件（测试版本）
- `test_pid_follow_planner.launch` - PID跟踪控制器launch文件（测试版本）

### 配置文件目录
`config/point_pid_params/test_medical_params/`
- `test_costmap_common_params.yaml` - 通用代价地图参数（适配医疗场景）
- `test_global_costmap_params.yaml` - 全局代价地图参数（使用base_footprint坐标系）
- `test_local_costmap_params.yaml` - 局部代价地图参数（使用base_footprint坐标系）
- `test_teb_local_planner_params.yaml` - TEB局部规划器参数（降低速度，提高安全性）

## 使用步骤

### 1. 启动底盘系统
```bash
roslaunch medical_race chassis_all_run.launch
```

### 2. 启动定位系统
```bash
roslaunch jie_ware lidar_loc_test.launch
```

### 3. 启动导航系统
```bash
roslaunch auto_nav test_navi_simple_meca_car_pid.launch
```

## 主要修改点

### 话题重映射
- PID控制器输出从`/base_vel`重映射到`/cmd_vel`，直接控制wheeltec底盘
- move_base输出重映射到`/move_base_cmd_vel`，避免冲突

### 坐标系统一
- 所有配置文件中的`robot_base_frame`从`base_link`改为`base_footprint`
- 与medical_race系统保持一致

### 医疗场景适配
- 降低最大速度和加速度限制
- 增大机器人安全半径和障碍物检测距离
- 提高位置和角度精度要求
- 禁用后退运动，增强安全性

### 系统兼容性
- 注释掉重复的map_server和定位节点
- 禁用仿真时间参数
- 使用jie_ware的定位系统而非scan_to_map

## 调试建议

### 检查话题连接
```bash
# 检查速度命令话题
rostopic echo /cmd_vel

# 检查全局路径规划
rostopic echo /move_base/GlobalPlanner/plan

# 检查定位信息
rostopic echo /tf
```

### 参数调优
如果机器人运动不理想，可以调整以下参数：
- `test_pid_follow_planner.launch`中的PID参数（p_value, i_value, d_value）
- `test_costmap_common_params.yaml`中的安全半径
- `test_teb_local_planner_params.yaml`中的速度限制

### 可能的问题
1. **坐标变换错误**: 检查tf树是否完整
2. **路径规划失败**: 检查地图是否正确加载
3. **机器人不动**: 检查话题重映射是否正确
4. **运动过慢**: 调整速度参数和PID增益

## 下一步优化
1. 根据实际测试效果调整PID参数
2. 优化代价地图参数以适应具体环境
3. 根据需要添加动态障碍物处理
4. 考虑添加路径平滑算法
