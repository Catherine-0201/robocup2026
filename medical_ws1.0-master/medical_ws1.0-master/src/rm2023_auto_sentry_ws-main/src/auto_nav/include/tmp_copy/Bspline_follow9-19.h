#ifndef BSPLINE_FOLLOW_H
#define BSPLINE_FOLLOW_H

#include <cmath>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_listener.h>
#include <vector>
#include <algorithm>
#include <std_msgs/Float64MultiArray.h>

// B-Spline辅助结构体
struct Points {
    std::vector<double> points_x;
    std::vector<double> points_y;
};

class RobotCtrl {
public:
    RobotCtrl();
    ~RobotCtrl() = default;

    void GenTraj(const nav_msgs::Path& prune_path, nav_msgs::Path& local_path);
    void FollowTraj(const geometry_msgs::PoseStamped& robot_pose,
                    const nav_msgs::Path& traj,
                    geometry_msgs::Twist& cmd_vel);
    
    // 公共接口函数
    bool hasPlan() const { return plan_; }
    void setPlan(bool plan) { plan_ = plan; }
    const nav_msgs::Path& getGlobalPath() const { return global_path_; }
    const nav_msgs::Path& getLocalPath() const { return local_path_; }
    void setLocalPath(const nav_msgs::Path& path) { local_path_ = path; }
    void publishCmdVel(const geometry_msgs::Twist& cmd_vel) { cmd_vel_pub_.publish(cmd_vel); }
    void getRobotPose(geometry_msgs::PoseStamped& robot_pose) { GetRobotPose(robot_pose); }
    
private:
    // 新增成员变量
    std::vector<double> ref_vels_;      // 存储参考速度
    std::vector<double> ref_times_;     // 存储参考时间
    ros::Time traj_base_time_;          // 轨迹基准时间

    double a_max_;                      // 线加速度上限
    double d_max_;                      // 线减速度上限
    double a_lat_max_;                  // 横向加速度上限
    double max_linear_speed_;           // 最大线速度限制
    double max_angular_speed_;          // 最大角速度限制
    int lookahead_idx_;                 // 前视索引，避免抖动
    double yaw_;                        // 机器人当前朝向
    double initial_yaw_;                // 初始姿态，将在IMU回调中设置
    double goal_dist_tolerance_;        // 目标距离容忍度
    bool plan_;                         // 是否启用路径规划的标志
    bool has_global_path_;              // 是否有全局路径

    // 工具函数
    static std::vector<double> ComputeArc(const Points& pts);
    static std::vector<double> ComputeCurvature(const Points& pts);
    std::vector<double> BuildVelocityLimit(const Points& pts, const std::vector<double>& s);
    std::vector<double> TimeParameterize(const Points& pts, const std::vector<double>& s, const std::vector<double>& vlimit);
    std::vector<double> BuildTimeStamps(const std::vector<double>& s, const std::vector<double>& v);
    
    // 辅助函数
    double NormalizeAngle(double angle);
    double GetEuclideanDistance(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2);
    void GlobalPathCallback(const nav_msgs::Path::ConstPtr& msg);
    void ImuCallback(const sensor_msgs::Imu& msg);
    void GetRobotPose(geometry_msgs::PoseStamped& robot_pose);

    // ROS 相关成员
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher local_path_pub_;      // 发布B样条轨迹用于可视化
    ros::Subscriber global_path_sub_;
    ros::Subscriber imu_sub_;
    std::shared_ptr<tf::TransformListener> tf_listener_;
    
    // 存储全局路径和局部路径
    nav_msgs::Path global_path_;
    nav_msgs::Path local_path_;
};

#endif // BSPLINE_FOLLOW_H
