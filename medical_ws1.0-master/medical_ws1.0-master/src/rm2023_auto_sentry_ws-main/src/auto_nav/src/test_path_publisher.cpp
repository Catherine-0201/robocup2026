#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

/**
 * 测试用全局路径发布器
 * 用于在没有move_base的情况下测试监控功能
 */

class TestPathPublisher {
public:
    TestPathPublisher() {
        ros::NodeHandle nh("~");
        
        // 发布器
        path_pub_ = nh.advertise<nav_msgs::Path>("/move_base/GlobalPlanner/plan", 10);
        
        // 定时器，每2秒发布一次稍有变化的路径
        timer_ = nh.createTimer(ros::Duration(2.0), &TestPathPublisher::publishPath, this);
        
        path_counter_ = 0;
        
        ROS_INFO("测试路径发布器已启动，将发布到 /move_base/GlobalPlanner/plan");
    }
    
private:
    void publishPath(const ros::TimerEvent& event) {
        nav_msgs::Path path;
        path.header.stamp = ros::Time::now();
        path.header.frame_id = "map";
        
        // 创建一个简单的直线路径，每次稍有变化来触发阈值检测
        for (int i = 0; i < 10; i++) {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "map";
            
            // 基础路径：从(0,0)到(5,0)的直线
            double base_x = i * 0.5;
            double base_y = 0.0;
            
            // 添加周期性扰动来测试阈值功能
            double disturbance = 0.1 * sin(path_counter_ * 0.5) * sin(i * 0.3);
            
            pose.pose.position.x = base_x + disturbance;
            pose.pose.position.y = base_y + disturbance * 0.5;
            pose.pose.position.z = 0.0;
            
            // 设置朝向
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
            
            path.poses.push_back(pose);
        }
        
        path_pub_.publish(path);
        
        ROS_INFO("发布测试路径 #%d，包含 %lu 个点，扰动: %.3f", 
                 path_counter_, path.poses.size(), 0.1 * sin(path_counter_ * 0.5));
        
        path_counter_++;
    }
    
    ros::Publisher path_pub_;
    ros::Timer timer_;
    int path_counter_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_path_publisher");
    
    TestPathPublisher publisher;
    
    ROS_INFO("测试路径发布器开始运行...");
    ros::spin();
    
    return 0;
}











