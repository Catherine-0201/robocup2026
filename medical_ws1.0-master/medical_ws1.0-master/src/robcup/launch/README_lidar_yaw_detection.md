# 激光雷达Yaw检测Launch文件使用说明

## 文件说明
- `lidar_yaw_detection.launch`: 激光雷达yaw角度检测节点的launch文件

## 使用方法

### 1. 基本使用（使用默认参数）
```bash
roslaunch robcup lidar_yaw_detection.launch
```

### 2. 指定墙壁方向
```bash
# 检测前面墙壁
roslaunch robcup lidar_yaw_detection.launch wall_direction:=1

# 检测左面墙壁
roslaunch robcup lidar_yaw_detection.launch wall_direction:=2

# 检测右面墙壁
roslaunch robcup lidar_yaw_detection.launch wall_direction:=3
```

### 3. 自定义角度范围
```bash
# 检测前面墙壁，角度范围±45°
roslaunch robcup lidar_yaw_detection.launch wall_direction:=1 front_angle_range:=45

# 检测左面墙壁，角度范围45°-135°
roslaunch robcup lidar_yaw_detection.launch wall_direction:=2 left_angle_min:=45 left_angle_max:=135

# 检测右面墙壁，角度范围-135°到-45°
roslaunch robcup lidar_yaw_detection.launch wall_direction:=3 right_angle_min:=-135 right_angle_max:=-45
```

### 4. 自定义距离范围
```bash
# 检测距离0.2m-1.0m范围内的点云
roslaunch robcup lidar_yaw_detection.launch min_distance:=0.2 max_distance:=1.0
```

### 5. 自定义最小点数阈值
```bash
# 设置最小点数阈值为20
roslaunch robcup lidar_yaw_detection.launch min_points:=20
```

### 6. 启动可视化
```bash
# 同时启动rviz进行可视化
roslaunch robcup lidar_yaw_detection.launch use_rviz:=true
```

### 7. 组合使用多个参数
```bash
# 检测左面墙壁，距离0.5m-1.2m，角度范围60°-120°，最小点数15
roslaunch robcup lidar_yaw_detection.launch \
    wall_direction:=2 \
    min_distance:=0.5 \
    max_distance:=1.2 \
    left_angle_min:=60 \
    left_angle_max:=120 \
    min_points:=15
```

## 参数说明

| 参数名 | 默认值 | 说明 |
|--------|--------|------|
| `wall_direction` | 1 | 墙壁方向：1=前面，2=左面，3=右面 |
| `front_angle_range` | 60 | 前面墙壁角度范围（±度数） |
| `left_angle_min` | 30 | 左面墙壁最小角度（度） |
| `left_angle_max` | 150 | 左面墙壁最大角度（度） |
| `right_angle_min` | -150 | 右面墙壁最小角度（度） |
| `right_angle_max` | -30 | 右面墙壁最大角度（度） |
| `min_distance` | 0.3 | 最小检测距离（米） |
| `max_distance` | 0.7 | 最大检测距离（米） |
| `min_points` | 10 | 最小点数阈值 |
| `use_rviz` | false | 是否启动rviz可视化 |

## 输出话题
- `/lidar_yaw`: 发布检测到的yaw角度（Float32类型，单位为度）

## 输入话题
- `/scan`: 激光雷达扫描数据（LaserScan类型）

## 注意事项
1. 确保激光雷达节点已启动并发布`/scan`话题
2. 根据实际环境调整角度和距离参数
3. 如果检测不到足够的点云数据，会输出警告信息
4. 角度范围基于激光雷达坐标系：x轴朝前，y轴朝左
