# 三点分析节点使用说明

## 功能描述

`three_points_analyzer.py` 节点用于分析雷达数据中半径20cm内的三个模块点，计算距离最小的两个点连成的线与前方墙壁/障碍物的平行性。

## 主要功能

1. **模块点检测**: 在雷达数据中检测半径20cm内的三个点（模块）
2. **距离计算**: 计算三个点之间的两两距离
3. **最短距离识别**: 找到距离最小的两个点
4. **平行性分析**: 比较这两点连成的线与前方墙壁/障碍物是否平行
5. **结果输出**: 输出分析结果和平行性判断

## 输入数据

- **订阅话题**: `/scan` (sensor_msgs/LaserScan)
  - 原始雷达数据（过滤前的数据）

## 输出数据

- **发布话题**: 
  - `/three_points_analysis_result` (std_msgs/String): 详细的分析结果
  - `/is_parallel_to_wall` (std_msgs/Bool): 平行性判断结果

## 配置参数

配置文件: `config/three_points_analyzer.yaml`

```yaml
# 检测半径（米）- 在此半径内寻找三个模块点
detection_radius: 0.20

# 最小点数要求
min_points: 3

# 墙壁平行性判断阈值（度）
# 如果两条线的角度差小于此阈值，则认为平行
wall_parallel_threshold: 10.0

# 日志节流周期（秒）
log_throttle_period: 1.0
```

## 使用方法

### 1. 直接启动节点

```bash
rosrun jie_ware three_points_analyzer.py
```

### 2. 通过launch文件启动

使用 `lidar_loc_test.launch` 文件，节点会自动启动：

```bash
roslaunch jie_ware lidar_loc_test.launch
```

### 3. 查看输出结果

```bash
# 查看详细分析结果
rostopic echo /three_points_analysis_result

# 查看平行性判断结果
rostopic echo /is_parallel_to_wall
```

## 算法原理

### 1. 点检测
- 扫描激光雷达数据中距离小于等于20cm的所有点
- 将极坐标转换为笛卡尔坐标系
- 如果检测到超过3个点，选择距离原点最近的3个点

### 2. 距离计算
- 计算三个点之间的两两欧几里得距离
- 找到距离最小的两个点作为分析对象

### 3. 墙壁检测
- 分析前方区域（±30度范围内）的激光点
- 使用最小二乘法拟合直线来估计墙壁角度
- 过滤掉过近（<30cm）或过远（>3m）的点以提高准确性

### 4. 平行性判断
- 计算两点连线的角度
- 计算前方墙壁的角度
- 比较两个角度的差值，如果小于阈值则认为平行

## 输出示例

控制台输出：
```
[INFO] 三点分析节点已启动，检测半径: 0.20m
[INFO] 两点连线角度: 15.3°, 前方墙壁角度: 12.7°, 平行性: 是
```

话题输出：
```bash
# /three_points_analysis_result
data: "两点连线角度: 15.3°, 前方墙壁角度: 12.7°, 平行性: 是"

# /is_parallel_to_wall
data: True
```

## 故障排除

### 1. 检测不到足够的点
```
[WARN] 检测到的点数不足: 2/3
```
**解决方案**: 确保机器人前方20cm范围内有足够的障碍物/模块

### 2. 无法检测到前方墙壁
```
[WARN] 无法检测到前方墙壁
```
**解决方案**: 确保机器人前方有明显的墙壁或障碍物，距离在30cm-3m之间

### 3. 节点启动失败
**检查事项**:
- 确保雷达数据正常发布到 `/scan` 话题
- 检查配置文件路径是否正确
- 确保脚本有执行权限

## 参数调优

### 调整检测半径
```yaml
detection_radius: 0.15  # 减小到15cm
```

### 调整平行性阈值
```yaml
wall_parallel_threshold: 5.0  # 更严格的平行性要求
```

### 调整日志频率
```yaml
log_throttle_period: 2.0  # 每2秒输出一次日志
```

















