#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32.h>

class CmdVelMonitor {
private:
    ros::NodeHandle nh_;
    
    // 订阅话题
    ros::Subscriber cmd_vel_P_sub_;    // Python发布的速度命令
    ros::Subscriber cmd_vel_C_sub_;    // C++发布的速度命令
    ros::Subscriber arm_done_sub_;     // 机械臂完成信号
    ros::Subscriber target_done_sub_;  // 目标完成信号
    
    // 发布话题
    ros::Publisher cmd_vel_pub_;       // 最终的速度命令
    
    // 状态变量
    int arm_done_count_;
    int target_done_count_;
    bool use_python_control_;
    std::string current_controller_;
    
    // 最新接收到的速度命令
    geometry_msgs::Twist latest_cmd_vel_P_;
    geometry_msgs::Twist latest_cmd_vel_C_;
    
    // 定时器
    ros::Timer control_timer_;
    
public:
    CmdVelMonitor() : arm_done_count_(0), target_done_count_(0), use_python_control_(false), current_controller_("none") {
        // 初始化速度命令为零
        latest_cmd_vel_P_.linear.x = latest_cmd_vel_P_.linear.y = latest_cmd_vel_P_.angular.z = 0.0;
        latest_cmd_vel_C_.linear.x = latest_cmd_vel_C_.linear.y = latest_cmd_vel_C_.angular.z = 0.0;
        
        // 订阅话题
        cmd_vel_P_sub_ = nh_.subscribe("/cmd_vel_P", 1, &CmdVelMonitor::cmdVelPCallback, this);
        cmd_vel_C_sub_ = nh_.subscribe("/cmd_vel_C", 1, &CmdVelMonitor::cmdVelCCallback, this);
        arm_done_sub_ = nh_.subscribe("/arm_done", 1, &CmdVelMonitor::armDoneCallback, this);
        target_done_sub_ = nh_.subscribe("/target_done", 1, &CmdVelMonitor::targetDoneCallback, this);
        
        // 发布最终速度命令
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        
        // 创建高频定时器，确保及时响应
        control_timer_ = nh_.createTimer(ros::Duration(0.01), &CmdVelMonitor::controlTimerCallback, this);
        
        ROS_INFO("CmdVelMonitor初始化完成");
        ROS_INFO("监控逻辑:");
        ROS_INFO("  target_done <= 2: 使用C++控制");
        ROS_INFO("  target_done = 2, arm_done = 0: 使用Python控制");
        ROS_INFO("  target_done = 2, arm_done = 1: 使用C++控制");
        ROS_INFO("  target_done = 3, arm_done = 1: 使用Python控制");
        ROS_INFO("  target_done = 3, arm_done = 2: 使用C++控制");
    }
    
    void cmdVelPCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        latest_cmd_vel_P_ = *msg;
    }
    
    void cmdVelCCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        latest_cmd_vel_C_ = *msg;
    }
    
    void armDoneCallback(const std_msgs::Int32::ConstPtr& msg) {
        arm_done_count_ = msg->data;
        ROS_INFO("收到arm_done: %d", arm_done_count_);
    }
    
    void targetDoneCallback(const std_msgs::Int32::ConstPtr& msg) {
        target_done_count_ = msg->data;
        ROS_INFO("收到target_done: %d", target_done_count_);
    }
    
    void controlTimerCallback(const ros::TimerEvent&) {
        // 根据监控逻辑决定使用哪个控制器
        bool should_use_python = false;
        std::string new_controller;
        
        if (target_done_count_ <2) {
            should_use_python = false;
            new_controller = "C++";
        } else if (target_done_count_ == 2 && arm_done_count_ == 0) {
            should_use_python = true;
            new_controller = "Python";
        } else if (target_done_count_ == 2 && arm_done_count_ == 1) {
            should_use_python = false;
            new_controller = "C++";
        } else if (target_done_count_ == 3 && arm_done_count_ == 1) {
            should_use_python = true;
            new_controller = "Python";
        } else if (target_done_count_ == 3 && arm_done_count_ == 2) {
            should_use_python = false;
            new_controller = "C++";
        } else {
            // 默认使用C++控制
            should_use_python = false;
            new_controller = "C++";
        }
        
        // 检查是否需要切换控制器
        if (should_use_python != use_python_control_ || new_controller != current_controller_) {
            use_python_control_ = should_use_python;
            current_controller_ = new_controller;
            ROS_INFO("切换控制模式: %s (target_done=%d, arm_done=%d)", 
                     current_controller_.c_str(), target_done_count_, arm_done_count_);
        }
        
        // 发布对应的速度命令
        if (use_python_control_) {
            cmd_vel_pub_.publish(latest_cmd_vel_P_);
        } else {
            cmd_vel_pub_.publish(latest_cmd_vel_C_);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "cmd_vel_monitor");
    
    CmdVelMonitor monitor;
    
    ROS_INFO("CmdVelMonitor节点启动，开始监控速度命令...");
    
    ros::spin();
    
    return 0;
}

