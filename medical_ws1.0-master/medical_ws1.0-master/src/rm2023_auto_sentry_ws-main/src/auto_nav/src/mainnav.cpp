#include <ros/ros.h>
#include "Bspline_follow.h"  // 引入之前的头文件，包含 RobotCtrl 类的定义

int main(int argc, char** argv) {
    // 初始化 ROS 系统
    ros::init(argc, argv, "test_Bspline_follow");  // 初始化 ROS 节点，节点名为 "test_Bspline_follow"
    
    // 打印启动信息
    ROS_INFO("Starting Medical Robot B-Spline Position Follow Controller...");

    // 创建 RobotCtrl 对象
    RobotCtrl robotctrl;

    // 定时器设置，10Hz
    ros::Rate loop_rate(10);  // 设置循环频率为 10Hz

    while (ros::ok()) {
        // 这里可以调用 RobotCtrl 类中的方法进行轨迹生成和路径跟踪
        // 比如生成轨迹：robotctrl.GenTraj(...);
        // 路径跟踪：robotctrl.FollowTraj(...);

        // 如果需要处理其他 ROS 消息，可以在这里进行
        ros::spinOnce();  // 处理 ROS 消息，处理订阅的消息

        loop_rate.sleep();  // 控制循环频率
    }

    return 0;
}
