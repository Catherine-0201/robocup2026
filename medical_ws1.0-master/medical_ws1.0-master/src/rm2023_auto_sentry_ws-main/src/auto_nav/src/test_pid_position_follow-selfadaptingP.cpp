#include "pid_position_follow.h"
#include <limits>

/**
 * 阶段2：简单速度自适应PID控制器
 * 功能：根据当前速度线性调整P值
 * 公式：P = P_min + (P_max - P_min) * (current_speed / v_max)
 */

RobotCtrl::RobotCtrl()
{
    ros::NodeHandle nh("~");
    
    // 基础参数加载（继承阶段1）
    nh.param<double>("max_x_speed", max_x_speed_, 1.0);
    nh.param<double>("max_y_speed", max_y_speed_, 1.0);
    nh.param<double>("set_yaw_speed", set_yaw_speed_, 0.0);
    nh.param<double>("p_value", p_value_, 0.5);
    nh.param<double>("i_value", i_value_, 0);
    nh.param<double>("d_value", d_value_, 0);
    
    nh.param<int>("plan_frequency", plan_freq_, 30);
    nh.param<double>("goal_dist_tolerance", goal_dist_tolerance_, 0.2);
    nh.param<double>("prune_ahead_distance", prune_ahead_dist_, 0.5);
    nh.param<std::string>("global_frame", global_frame_, "map");

    // 阶段2新增：速度自适应参数
    nh.param<double>("min_p_value", min_p_value_, 0.2);   // 最小P值
    nh.param<double>("max_p_value", max_p_value_, 0.8);   // 最大P值
    nh.param<double>("v_max_for_adaptation", v_max_for_adaptation_, 0.8);  // 自适应参考最大速度
    
    // 阶段3新增：阈值化重规划参数
    nh.param<bool>("enable_path_threshold", enable_path_threshold_, true);
    nh.param<double>("path_distance_threshold", path_distance_threshold_, 0.3);
    nh.param<double>("path_angle_threshold", path_angle_threshold_, 0.5);
    nh.param<double>("path_update_min_interval", path_update_min_interval_, 0.5);
    nh.param<int>("path_comparison_points", path_comparison_points_, 5);

    // ROS发布器和订阅器初始化
    local_path_pub_= nh.advertise<nav_msgs::Path>("path", 5);
    global_path_sub_ = nh.subscribe("/move_base/GlobalPlanner/plan", 5, &RobotCtrl::GlobalPathCallback,this);
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/base_vel",10);
    target_done_pub_ = nh.advertise<std_msgs::Int32>("/target_done", 10);
    planner_server_ = nh.advertiseService("/pid_planner_status",&RobotCtrl::Follower_StateReq, this);

    tf_listener_ = std::make_shared<tf::TransformListener>();
    plan_timer_ = nh.createTimer(ros::Duration(1.0/plan_freq_),&RobotCtrl::Plan,this);

    // 医疗机器人默认设置
    game_state_ = 4;        // 始终启用
    planner_state_ = 2;     // 默认为路径跟踪状态
    
    // 阶段2新增：速度历史记录初始化
    current_speed_ = 0.0;
    prev_cmd_vel_.linear.x = 0.0;
    prev_cmd_vel_.linear.y = 0.0;
    
    // 阶段3新增：阈值化重规划初始化
    last_path_update_time_ = ros::Time::now();
    
    // 目标完成计数器初始化
    target_count_ = 0;
    goal_just_completed_ = false;
    // 初始化上次完成目标位置为无效值
    last_completed_goal_.pose.position.x = std::numeric_limits<double>::max();
    last_completed_goal_.pose.position.y = std::numeric_limits<double>::max();
    
    ROS_INFO("阶段2：速度自适应PID控制器已启动");
    ROS_INFO("P值范围: [%.3f, %.3f], 参考最大速度: %.2f m/s", 
             min_p_value_, max_p_value_, v_max_for_adaptation_);
    ROS_INFO("阶段3：阈值化重规划已启用 - 距离阈值: %.2f m, 角度阈值: %.2f rad, 时间间隔: %.2f s",
             path_distance_threshold_, path_angle_threshold_, path_update_min_interval_);
}

double RobotCtrl::CalculateAdaptiveP(double current_speed)
{
    // 阶段2核心算法：线性速度自适应
    // P = P_min + (P_max - P_min) * (current_speed / v_max)
    
    double speed_ratio = std::min(current_speed / v_max_for_adaptation_, 1.0);
    double adaptive_p = min_p_value_ + (max_p_value_ - min_p_value_) * speed_ratio;
    
    // 调试信息（每秒最多输出一次）
    static ros::Time last_debug_time = ros::Time::now();
    if ((ros::Time::now() - last_debug_time).toSec() > 1.0) {
        ROS_INFO("速度自适应 - 当前速度: %.3f m/s, 速度比率: %.3f, 自适应P值: %.3f", 
                 current_speed, speed_ratio, adaptive_p);
        last_debug_time = ros::Time::now();
    }
    
    return adaptive_p;
}

void RobotCtrl::UpdateCurrentSpeed(const geometry_msgs::Twist& cmd_vel)
{
    // 计算当前速度大小
    current_speed_ = std::sqrt(cmd_vel.linear.x * cmd_vel.linear.x + 
                              cmd_vel.linear.y * cmd_vel.linear.y);
}

bool RobotCtrl::ShouldAcceptNewPath(const nav_msgs::Path& new_path)
{
    // 阶段3：阈值化重规划判断函数
    
    // 1. 检查是否启用阈值化重规划
    if (!enable_path_threshold_) {
        return true;  // 未启用，直接接受新路径
    }
    
    // 2. 检查时间间隔阈值
    ros::Time current_time = ros::Time::now();
    if ((current_time - last_path_update_time_).toSec() < path_update_min_interval_) {
        return false;  // 时间间隔太短，拒绝新路径
    }
    
    // 3. 检查是否有当前路径进行比较
    if (global_path_.poses.empty() || new_path.poses.empty()) {
        return true;  // 没有当前路径或新路径为空，直接接受
    }
    
    // 4. 计算路径距离偏差
    int compare_points = std::min(path_comparison_points_, 
                                 std::min((int)global_path_.poses.size(), (int)new_path.poses.size()));
    double total_distance_diff = 0.0;
    
    for (int i = 0; i < compare_points; i++) {
        double dx = new_path.poses[i].pose.position.x - global_path_.poses[i].pose.position.x;
        double dy = new_path.poses[i].pose.position.y - global_path_.poses[i].pose.position.y;
        total_distance_diff += std::sqrt(dx * dx + dy * dy);
    }
    
    double avg_distance_diff = total_distance_diff / compare_points;
    
    // 5. 计算路径角度偏差（比较前几个点的方向变化）
    double angle_diff = 0.0;
    if (compare_points >= 2) {
        // 计算当前路径的初始方向
        double old_dx = global_path_.poses[1].pose.position.x - global_path_.poses[0].pose.position.x;
        double old_dy = global_path_.poses[1].pose.position.y - global_path_.poses[0].pose.position.y;
        double old_angle = std::atan2(old_dy, old_dx);
        
        // 计算新路径的初始方向
        double new_dx = new_path.poses[1].pose.position.x - new_path.poses[0].pose.position.x;
        double new_dy = new_path.poses[1].pose.position.y - new_path.poses[0].pose.position.y;
        double new_angle = std::atan2(new_dy, new_dx);
        
        // 计算角度差异
        angle_diff = std::abs(new_angle - old_angle);
        if (angle_diff > M_PI) {
            angle_diff = 2 * M_PI - angle_diff;  // 处理角度环绕
        }
    }
    
    // 6. 判断是否接受新路径
    bool accept_path = (avg_distance_diff > path_distance_threshold_) || 
                      (angle_diff > path_angle_threshold_);
    
    // 7. 调试信息（每秒最多输出一次）
    static ros::Time last_debug_time = ros::Time::now();
    if ((current_time - last_debug_time).toSec() > 1.0) {
        ROS_INFO("阈值化重规划 - 距离偏差: %.3f m (阈值: %.3f), 角度偏差: %.3f rad (阈值: %.3f), 接受: %s",
                 avg_distance_diff, path_distance_threshold_, 
                 angle_diff, path_angle_threshold_,
                 accept_path ? "是" : "否");
        last_debug_time = current_time;
    }
    
    return accept_path;
}

void RobotCtrl::Plan(const ros::TimerEvent& event){
    // 简化状态检查：医疗机器人只需要检查规划状态
    if (planner_state_ == 2)  // 路径跟踪状态
    {            
        if (plan_ ){
            auto begin = std::chrono::steady_clock::now();
            auto start = ros::Time::now();
            
            // 1. Update the transform from global path frame to local planner frame
            UpdateTransform(tf_listener_, global_frame_,
                            global_path_.header.frame_id, global_path_.header.stamp,
                            global2path_transform_);
            std::cout<<ros::Time::now()- start<<std::endl;

            // 2. Get current robot pose in global path frame
            geometry_msgs::PoseStamped robot_pose;
            GetGlobalRobotPose(tf_listener_, global_path_.header.frame_id, robot_pose);

            // 3. Check if robot has already arrived with given distance tolerance
            if (GetEuclideanDistance(robot_pose,global_path_.poses.back())<= goal_dist_tolerance_
                || prune_index_ == global_path_.poses.size() - 1){
                plan_ = false;
                geometry_msgs::Twist cmd_vel;
                cmd_vel.linear.x = 0;
                cmd_vel.linear.y = 0;
                cmd_vel.angular.z = 0;
                cmd_vel.linear.z = 1;   // bool success or not
                cmd_vel_pub_.publish(cmd_vel);
                
                // 新增：目标完成计数和发布（带重复保护）
                geometry_msgs::PoseStamped current_goal = global_path_.poses.back();
                double distance_to_last_goal = GetEuclideanDistance(current_goal, last_completed_goal_);
                
                // 只有当前目标与上次完成的目标距离足够远时才增加计数
                if (distance_to_last_goal > goal_dist_tolerance_ * 2.0 || target_count_ == 0) {
                    target_count_++;
                    std_msgs::Int32 done_msg;
                    done_msg.data = target_count_;
                    target_done_pub_.publish(done_msg);
                    
                    // 记录当前完成的目标
                    last_completed_goal_ = current_goal;
                    goal_just_completed_ = true;
                    
                    ROS_INFO("阶段2控制器 - 医疗机器人已到达目标位置！完成目标数: %d", target_count_);
                } else {
                    ROS_DEBUG("目标重复，跳过计数 - 与上次目标距离: %.3f", distance_to_last_goal);
                }
                return;
            }

            // 4. Get prune index from given global path
            FindNearstPose(robot_pose, global_path_, prune_index_, prune_ahead_dist_);

            // 5. Generate the prune path and transform it into local planner frame
            nav_msgs::Path prune_path, local_path;

            local_path.header.frame_id = global_frame_;
            prune_path.header.frame_id = global_frame_;

            geometry_msgs::PoseStamped tmp_pose;
            tmp_pose.header.frame_id = global_frame_;

            TransformPose(global2path_transform_, robot_pose, tmp_pose);
            prune_path.poses.push_back(tmp_pose);
            
            int i = prune_index_;

            while (i < global_path_.poses.size() && i - prune_index_< 20 ){
                TransformPose(global2path_transform_, global_path_.poses[i], tmp_pose);
                prune_path.poses.push_back(tmp_pose);
                i++;
            }

            // 6. Generate the cubic spline trajectory from above prune path
            GenTraj(prune_path, local_path);
            local_path_pub_.publish(local_path);

            // 7. Follow the trajectory and calculate the velocity (阶段2改进版本)
            geometry_msgs::Twist cmd_vel;
            FollowTraj(robot_pose, local_path, cmd_vel);
            cmd_vel_pub_.publish(cmd_vel);

            auto plan_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - begin);
            ROS_INFO("阶段2医疗机器人规划耗时 %f ms，已通过 %d/%d 路径点",
                     plan_time.count()/1000.,
                     prune_index_,
                     static_cast<int>(global_path_.poses.size()));
        }
        else
        {
            // 无路径时保持静止
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            cmd_vel.angular.z = 0;
            cmd_vel.linear.z = 0;
            cmd_vel_pub_.publish(cmd_vel);
        }
    }
    else if(planner_state_ == 1)  // 原地旋转状态
    {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.angular.z = set_yaw_speed_;
        cmd_vel.linear.z = 0;
        cmd_vel_pub_.publish(cmd_vel);
        ROS_INFO("医疗机器人 - 原地旋转");
    }
    else  // planner_state_ == 0, 静止状态
    {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.angular.z = 0;
        cmd_vel.linear.z = 0;
        cmd_vel_pub_.publish(cmd_vel);
    }
}

void RobotCtrl::FindNearstPose(geometry_msgs::PoseStamped& robot_pose,nav_msgs::Path& path, int& prune_index, double prune_ahead_dist){
    double dist_threshold = 10;// threshold is 10 meters (basically never over 10m i suppose)
    double sq_dist_threshold = dist_threshold * dist_threshold;
    double sq_dist;
    if(prune_index!=0){
        sq_dist = GetEuclideanDistance(robot_pose,path.poses[prune_index-1]);
    }else{
        sq_dist = 1e10;
    }

    double new_sq_dist = 0;
    while (prune_index < (int)path.poses.size()) {
        new_sq_dist = GetEuclideanDistance(robot_pose,path.poses[prune_index]);
        if (new_sq_dist > sq_dist && sq_dist < sq_dist_threshold) {

            //Judge if it is in the same direction and sq_dist is further than 0.3 meters
            if ((path.poses[prune_index].pose.position.x - robot_pose.pose.position.x) *
                (path.poses[prune_index-1].pose.position.x - robot_pose.pose.position.x) +
                (path.poses[prune_index].pose.position.y - robot_pose.pose.position.y) *
                (path.poses[prune_index-1].pose.position.y - robot_pose.pose.position.y) > 0
                && sq_dist > prune_ahead_dist) {
                prune_index--;
            }else{
                sq_dist = new_sq_dist;
            }

            break;
        }
        sq_dist = new_sq_dist;
        ++prune_index;
    }

    prune_index = std::min(prune_index, (int)(path.poses.size()-1));
}

void RobotCtrl::FollowTraj(const geometry_msgs::PoseStamped& robot_pose,
                        const nav_msgs::Path& traj,
                        geometry_msgs::Twist& cmd_vel){

    geometry_msgs::PoseStamped robot_pose_1;
    GetGlobalRobotPose(tf_listener_, global_path_.header.frame_id, robot_pose_1); 

    yaw_ = tf::getYaw(robot_pose_1.pose.orientation);

    // 计算目标方向和距离
    double diff_yaw = atan2((traj.poses[1].pose.position.y-robot_pose.pose.position.y ),( traj.poses[1].pose.position.x-robot_pose.pose.position.x));
    double diff_distance = GetEuclideanDistance(robot_pose,traj.poses[1]);

    // set it from -PI to PI
    if(diff_yaw > M_PI){
        diff_yaw -= 2*M_PI;
    } else if(diff_yaw < -M_PI){
        diff_yaw += 2*M_PI;
    }

    printf("阶段2医疗机器人 - diff_yaw: %f\n",diff_yaw);
    printf("阶段2医疗机器人 - diff_distance: %f\n",diff_distance);

    // 阶段2改进：使用自适应P值
    // 首先根据上一次的速度指令计算当前速度
    UpdateCurrentSpeed(prev_cmd_vel_);
    
    // 计算自适应P值
    double adaptive_p = CalculateAdaptiveP(current_speed_);

    // 使用自适应P值计算速度指令
    double vx_global = max_x_speed_*cos(diff_yaw)*adaptive_p;
    double vy_global = max_y_speed_*sin(diff_yaw)*adaptive_p;
    
    std::cout<<"yaw_: "<<yaw_<<std::endl;
    std::cout<<"vx_global: "<<vx_global<<"   vy_global: "<<vy_global<<std::endl;
    std::cout<<"自适应P值: "<<adaptive_p<<"   当前速度: "<<current_speed_<<std::endl;
    
    cmd_vel.linear.x = vx_global * cos(yaw_) + vy_global * sin(yaw_);
    cmd_vel.linear.y = - vx_global * sin(yaw_) + vy_global * cos(yaw_);
    
    // 医疗机器人在路径跟踪时不需要额外的角速度
    cmd_vel.angular.z = 0;
    
    // 保存当前速度指令用于下次计算
    prev_cmd_vel_ = cmd_vel;
}

void RobotCtrl::GlobalPathCallback(const nav_msgs::PathConstPtr & msg){
  if (!msg->poses.empty()){
      // 阶段3：使用阈值化重规划判断是否接受新路径
      if (ShouldAcceptNewPath(*msg)) {
          global_path_ = *msg;
          prune_index_ = 0;
          plan_ = true;
          goal_just_completed_ = false;  // 重置目标完成标志
          last_path_update_time_ = ros::Time::now();  // 更新路径接受时间
          ROS_INFO("阶段3医疗机器人 - 接受新的全局路径，包含 %lu 个路径点", msg->poses.size());
      } else {
          // 拒绝新路径，保持当前路径不变
          ROS_DEBUG("阶段3医疗机器人 - 拒绝新路径，偏差未超过阈值，保持当前路径");
      }
  }
}

void RobotCtrl::ImuCallback(const sensor_msgs::Imu &msg)
{
    // 如果需要使用IMU数据，可以在这里处理
}

// 服务处理函数 - 简化医疗机器人的状态控制
bool RobotCtrl::Follower_StateReq(roborts_msgs::PidPlannerStatus::Request& req,
          roborts_msgs::PidPlannerStatus::Response& resp){

    ROS_INFO("阶段2医疗机器人 - 请求数据: planner_state = %d, max_x_speed = %f, max_y_speed = %f, yaw_speed = %f"
            ,req.planner_state, req.max_x_speed, req.max_y_speed, req.yaw_speed);

    // 医疗机器人的状态检查：0=静止, 1=原地旋转, 2=路径跟踪
    if (req.planner_state < 0 || req.planner_state > 2)
    {
        ROS_ERROR("阶段2医疗机器人 - 无效的规划器状态: %d (应该是 0-2)", req.planner_state);
        resp.result = 0;    //失败时返回0
        return false;
    }

    planner_state_ = req.planner_state;
    
    // 更新速度参数
    if(req.max_x_speed > 0 && req.max_y_speed > 0)
    {
        max_x_speed_ = req.max_x_speed;
        max_y_speed_ = req.max_y_speed; 
        ROS_INFO("阶段2医疗机器人 - 速度已更新: x=%.2f, y=%.2f", max_x_speed_, max_y_speed_);
    }
    
    if(req.yaw_speed >= 0){
        set_yaw_speed_ = req.yaw_speed;
        ROS_INFO("阶段2医疗机器人 - 角速度已更新: %.2f", set_yaw_speed_);
    }

    resp.result = 1;   //成功时返回1
    
    // 打印当前状态
    std::string state_name;
    switch(planner_state_) {
        case 0: state_name = "静止"; break;
        case 1: state_name = "原地旋转"; break;
        case 2: state_name = "路径跟踪"; break;
        default: state_name = "未知"; break;
    }
    ROS_INFO("阶段2医疗机器人 - 状态切换至: %s", state_name.c_str());

    return true;
}

int main(int argc,char **argv)
{
  ros::init(argc, argv, "adaptive_pid_position_follow_stage2");
  ROS_INFO("启动阶段2：速度自适应PID医疗机器人控制器...");
  RobotCtrl robotctrl;
  ros::spin();
  return 0;
}