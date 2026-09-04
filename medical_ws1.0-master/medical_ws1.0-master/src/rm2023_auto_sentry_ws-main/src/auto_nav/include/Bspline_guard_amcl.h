#ifndef BSPLINE_FOLLOW_H
#define BSPLINE_FOLLOW_H

// Interface declaration for the independent AMCL build of
// test_Bspline_guard.cpp.  The implementation and all trajectory logic remain
// in the original source file.

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_listener.h>

struct Points {
    std::vector<double> points_x;
    std::vector<double> points_y;
};

class RobotCtrl {
public:
    RobotCtrl();
    ~RobotCtrl();

    void GenTraj(const nav_msgs::Path& prune_path, nav_msgs::Path& local_path);
    void FollowTraj(const geometry_msgs::PoseStamped& robot_pose,
                    const nav_msgs::Path& traj,
                    geometry_msgs::Twist& cmd_vel);

    bool hasPlan() const { return plan_; }
    void setPlan(bool plan) { plan_ = plan; }
    const nav_msgs::Path& getGlobalPath() const { return global_path_; }
    const nav_msgs::Path& getLocalPath() const { return local_path_; }
    void setLocalPath(const nav_msgs::Path& path) { local_path_ = path; }
    void publishCmdVel(const geometry_msgs::Twist& cmd_vel) {
        cmd_vel_pub_.publish(cmd_vel);
    }
    void getRobotPose(geometry_msgs::PoseStamped& robot_pose) {
        GetRobotPose(robot_pose);
    }

private:
    static std::vector<double> ComputeArc(const Points& pts);
    static std::vector<double> ComputeCurvature(const Points& pts);
    std::vector<double> BuildVelocityLimit(
        const Points& pts, const std::vector<double>& s);
    std::vector<double> TimeParameterize(
        const Points& pts,
        const std::vector<double>& s,
        const std::vector<double>& vlimit);
    std::vector<double> BuildTimeStamps(
        const std::vector<double>& s, const std::vector<double>& v);

    double NormalizeAngle(double angle);
    double GetEuclideanDistance(
        const geometry_msgs::PoseStamped& pose1,
        const geometry_msgs::PoseStamped& pose2);
    void MoveBaseGoalCallback(
        const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg);
    void MoveBaseSimpleGoalCallback(
        const geometry_msgs::PoseStamped::ConstPtr& msg);
    void GlobalPathCallback(const nav_msgs::Path::ConstPtr& msg);
    void ImuCallback(const sensor_msgs::Imu& msg);
    void GetRobotPose(geometry_msgs::PoseStamped& robot_pose);

    std::vector<double> ref_vels_;
    std::vector<double> ref_times_;
    ros::Time traj_base_time_;

    double a_max_;
    double d_max_;
    double a_lat_max_;
    double max_linear_speed_;
    double max_angular_speed_;
    int lookahead_idx_;
    double yaw_pid_;
    double yaw_;
    double initial_yaw_;
    double goal_dist_tolerance_;
    double path_goal_match_tolerance_;
    std::string cmd_vel_topic_;

    bool plan_;
    bool has_global_path_;
    int target_done_;
    geometry_msgs::PoseStamped former_goal_;
    int target_complete_;

    bool active_goal_received_;
    bool active_goal_path_ready_;
    std::string active_goal_id_;
    geometry_msgs::PoseStamped active_goal_;

    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher local_path_pub_;
    ros::Publisher target_done_pub_;
    ros::Subscriber move_base_goal_sub_;
    ros::Subscriber move_base_simple_goal_sub_;
    ros::Subscriber global_path_sub_;
    ros::Subscriber imu_sub_;
    std::shared_ptr<tf::TransformListener> tf_listener_;

    nav_msgs::Path global_path_;
    nav_msgs::Path local_path_;
};

#endif  // BSPLINE_FOLLOW_H
