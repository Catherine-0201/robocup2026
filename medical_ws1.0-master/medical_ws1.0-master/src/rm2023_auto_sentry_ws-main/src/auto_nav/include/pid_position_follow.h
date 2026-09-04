#ifndef PID_POSITION_FOLLOW_H
#define PID_POSITION_FOLLOW_H

#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include"geometry_msgs/Twist.h"
#include "std_msgs/Float64MultiArray.h"
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/JointState.h"

#include "utility.h"
#include <tf2/utils.h>
#include <iostream>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "cubic_spline/cubic_spline_ros.h"
#include "utility.h"
#include <Eigen/Eigen>
#include <chrono>

#include <roborts_msgs/GameStatus.h>
#include <roborts_msgs/PidPlannerStatus.h>
#include <std_msgs/Int32.h>

class RobotCtrl {
    public:
    RobotCtrl();
    ~RobotCtrl() = default;
    void ImuCallback(const sensor_msgs::Imu &msg);
    void JointstateCallback(const sensor_msgs::JointStateConstPtr &msg);
    void GimbalCtrl();
    void GlobalPathCallback(const nav_msgs::PathConstPtr & msg);
    void Game_StateCallback(const roborts_msgs::GameStatusPtr &msg );

    bool Follower_StateReq(roborts_msgs::PidPlannerStatus::Request& req, roborts_msgs::PidPlannerStatus::Response& resp);

    void FollowTraj(const geometry_msgs::PoseStamped& robot_pose,
                        const nav_msgs::Path& traj,
                        geometry_msgs::Twist& cmd_vel);
    void FindNearstPose(geometry_msgs::PoseStamped& robot_pose,nav_msgs::Path& path, int& prune_index, double prune_ahead_dist);
    void Plan(const ros::TimerEvent& event);

private:
    ros::Publisher gimbal_yaw_position_cmd_;
    ros::Publisher gimbal_pitch_position_cmd_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher local_path_pub_;
    ros::Publisher monitor_pub_;             // 阶段3+新增：监控数据发布器
    ros::Publisher target_done_pub_;         // 目标完成计数发布器

    ros::Subscriber imu_sub_;
    ros::Subscriber global_path_sub_;
    ros::Subscriber jointstate_sub_;
    ros::Subscriber game_state_sub_;
    ros::Timer plan_timer_;

    ros::ServiceServer planner_server_;

    std::shared_ptr<tf::TransformListener> tf_listener_;
    tf::StampedTransform global2path_transform_;

    nav_msgs::Path global_path_;

    bool plan_ = false;
    int prune_index_ = 0;

    double max_x_speed_;
    double max_y_speed_;

    double set_yaw_speed_;

    double p_value_;
    double i_value_;
    double d_value_;

    int plan_freq_;
    double goal_dist_tolerance_;
    double prune_ahead_dist_;

    std::string global_frame_;

    double a_gimbal_yaw_position;  //
    double a_gimbal_pitch_position;  //

    double cur_gimbal_yaw_position;
    double cur_gimbal_pitch_position;

    double yaw_;  //机器人航向角
    
    uint8_t game_state_ = 4;
    int planner_state_ = 2;   //规划状态 0：静止  1：原地小陀螺  2：路径跟踪

    // 阶段2新增：速度自适应相关变量
    double min_p_value_;      // 最小P值
    double max_p_value_;      // 最大P值
    double v_max_for_adaptation_; // 自适应参考最大速度
    double current_speed_;    // 当前速度
    geometry_msgs::Twist prev_cmd_vel_; // 上一次的速度指令
    
    // 阶段3新增：阈值化重规划相关变量
    bool enable_path_threshold_;         // 是否启用阈值化重规划
    double path_distance_threshold_;     // 路径距离阈值 [m]
    double path_angle_threshold_;        // 路径角度阈值 [rad]
    double path_update_min_interval_;    // 路径更新最小时间间隔 [s]
    int path_comparison_points_;         // 用于比较的路径点数量
    ros::Time last_path_update_time_;    // 上次路径更新时间
    
    // 阶段3+新增：监控相关变量
    bool enable_monitor_;                // 是否启用监控
    std::string monitor_csv_file_;       // CSV文件路径
    
    // 目标完成计数相关变量
    int target_count_;                   // 目标完成计数器
    geometry_msgs::PoseStamped last_completed_goal_;  // 上次完成的目标位置
    bool goal_just_completed_;           // 刚刚完成目标标志

public:
    // 阶段2新增：速度自适应相关函数
    double CalculateAdaptiveP(double current_speed);
    void UpdateCurrentSpeed(const geometry_msgs::Twist& cmd_vel);
    
    // 阶段3新增：阈值化重规划相关函数
    bool ShouldAcceptNewPath(const nav_msgs::Path& new_path);
    
    // 阶段3+新增：监控相关函数
    void InitializeMonitor();
    void PublishMonitorData(double distance_diff, double angle_diff, double time_interval, bool accepted);

};

//弧度制归一化
double normalizeRadian(const double angle)
{
   double n_angle = std::fmod(angle, 2 * M_PI);
   n_angle = n_angle > M_PI ? n_angle - 2 * M_PI : n_angle < -M_PI ? 2 * M_PI + n_angle : n_angle;
   return n_angle;
}

double ABS_limit(double value,double limit)
{
  if(value<limit && value>-limit)
  {
    return 0;
  }
  else
  {
    return value;
  }

}


#endif 
