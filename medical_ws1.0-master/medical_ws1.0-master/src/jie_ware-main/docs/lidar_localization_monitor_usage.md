# 激光雷达定位监控节点使用说明

## 概述

这个监控节点通过检测连续帧之间的位姿突变来识别激光雷达定位异常，在检测到异常时自动切换到基于里程计的备用定位模式。

## 工作原理

### 1. 突变检测
- **位移突变**: 连续帧之间位移变化超过阈值
- **角度突变**: 连续帧之间角度变化超过阈值  
- **速度异常**: 计算得到的瞬时速度超过合理范围
- **角速度异常**: 计算得到的瞬时角速度超过合理范围

### 2. 备用定位机制
- 检测到突变时，立即切换到里程计定位
- 计算并保存里程计漂移补偿值
- 使用补偿后的里程计位姿发布map→odom变换

### 3. 恢复机制
- 监控连续稳定帧数
- 达到设定的稳定帧数后恢复激光雷达定位
- 超时保护：超过最大备用时间后强制恢复

## 参数配置

### 核心参数 (config/localization_monitor.yaml)

```yaml
# 突变检测阈值
position_jump_threshold: 0.3    # 位移突变阈值 (米)
rotation_jump_threshold: 0.2    # 角度突变阈值 (弧度)
velocity_threshold: 2.0         # 最大合理速度 (米/秒)
angular_velocity_threshold: 1.0 # 最大合理角速度 (弧度/秒)

# 恢复参数
stable_frames_required: 5       # 恢复需要的连续稳定帧数
max_fallback_duration: 10.0     # 最大备用定位时间 (秒)
```

### 参数调优建议

**对于慢速机器人 (< 0.5 m/s)**:
```yaml
position_jump_threshold: 0.1
velocity_threshold: 1.0
angular_velocity_threshold: 0.5
```

**对于快速机器人 (> 1.0 m/s)**:
```yaml
position_jump_threshold: 0.5
velocity_threshold: 3.0
angular_velocity_threshold: 2.0
```

**环境干扰较多时**:
```yaml
stable_frames_required: 10
max_fallback_duration: 15.0
```

## 话题接口

### 订阅话题
- `/odom` (nav_msgs/Odometry): 里程计数据

### 发布话题
- `/localization_status` (std_msgs/Bool): 定位可靠性状态
- `/localization_info` (std_msgs/String): 详细状态信息
- `/backup_pose` (geometry_msgs/PoseWithCovarianceStamped): 备用位姿

### TF变换
- 发布 `map` → `odom` 变换（备用模式时）

## 使用方法

### 1. 启动节点
```bash
# 方法一：单独启动
rosrun jie_ware lidar_localization_monitor.py

# 方法二：通过launch文件启动
roslaunch jie_ware lidar_loc_test.launch
```

### 2. 监控状态
```bash
# 监控定位状态
rostopic echo /localization_status

# 查看详细信息
rostopic echo /localization_info

# 测试节点
rosrun jie_ware test_localization_monitor.py
```

## 状态信息解释

### localization_info 消息内容
```python
{
    'reliable': True/False,           # 当前定位是否可靠
    'is_jump': True/False,           # 当前帧是否检测到突变
    'stable_frames': 5,              # 连续稳定帧数
    'position_change': 0.05,         # 位移变化量 (米)
    'angle_change': 0.02,            # 角度变化量 (弧度)
    'velocity': 0.1,                 # 瞬时速度 (米/秒)
    'angular_velocity': 0.05,        # 瞬时角速度 (弧度/秒)
    'fallback_duration': 2.5         # 备用模式持续时间 (秒)
}
```

## 故障排除

### 1. 频繁误触发
**现象**: 正常运动时也触发突变检测

**解决方案**:
- 增大 `position_jump_threshold` 和 `rotation_jump_threshold`
- 检查机器人运动是否平滑
- 确认激光雷达定位本身是否稳定

### 2. 不能及时检测异常
**现象**: 明显的定位跳变没有被检测到

**解决方案**:
- 减小突变检测阈值
- 检查监控频率是否足够 (20Hz)
- 确认TF树正常发布

### 3. 备用定位效果差
**现象**: 切换到备用模式后位置漂移严重

**解决方案**:
- 检查里程计质量
- 减少 `max_fallback_duration`
- 考虑增加IMU融合

### 4. 无法恢复正常定位
**现象**: 长时间停留在备用模式

**解决方案**:
- 减少 `stable_frames_required`
- 检查激光雷达定位是否已恢复正常
- 查看是否超过 `max_fallback_duration`

## 性能优化

### 1. 监控频率
- 默认: 20Hz (0.05s)
- 高精度需求: 50Hz (0.02s)
- 低功耗需求: 10Hz (0.1s)

### 2. 内存使用
- 位姿历史缓存: 最大20帧
- 自动清理过期数据

### 3. CPU使用
- 主要计算: 位姿差分和速度计算
- 优化: 避免频繁的TF查询

## 集成建议

1. **与导航系统集成**: 监听 `/localization_status` 调整路径规划策略
2. **与安全系统集成**: 定位异常时降低机器人速度
3. **与用户界面集成**: 显示定位状态和异常警告
4. **数据记录**: 记录突变事件用于后续分析

## 版本更新日志

- v1.0: 基础的帧间变化检测功能
- v1.1: 添加里程计漂移补偿
- v1.2: 增加超时保护和稳定性改进
