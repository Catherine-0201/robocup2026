#ifndef PID_POSITION_FOLLOW_H
#define PID_POSITION_FOLLOW_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>      // 新增：for std::vector
#include <cmath>       // 新增：for std::sqrt, std::min 等

// ... 你的原有#include和类定义 ...

class RobotCtrl {
public:
    // ... 你的原有public成员 ...

    void GenTraj(const nav_msgs::Path& prune_path, nav_msgs::Path& local_path);

private:
    // ... 你的原有private成员 ...

    // 新增：B样条辅助结构体
    struct Points {
        std::vector<double> points_x;
        std::vector<double> points_y;
    };

    // 新增：参考速度生成函数（私有辅助）
    void GenerateRefVelocity(nav_msgs::Path& local_path, const Points& points);

    // ... 其他private成员 ...
};

#endif  // PID_POSITION_FOLLOW_H
