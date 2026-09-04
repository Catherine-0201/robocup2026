#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "std_msgs/Empty.h"
#include <signal.h>
#include <thread> //
#include <vector>
#include "pid_tracking/pid.h"

float angle_k[3], x_k[3], y_k[3];
float max_vel_x, max_vel_y, max_vel_w,acc;
float currant_state[3] = {0, 0, 0};
float init_angle = 0;
bool init_flag = 0;
PID angle_pid, x_pid, y_pid;
int goal_point_num,acc_time;
geometry_msgs::Twist twist_msg;
ros::Publisher cmd_vel_pub;
string odom_topic;
ros::Timer control_cmd_pub;
vector<pair<float,float>> goal_point;

void angle_loop(float target_angle, float vx, float vy);

/**
 * @brief 速度发布函数
 *
 * @param vx 前进为正方向
 * @param vy 左为正方向
 * @param vw 逆时针，左旋正方向
 */
void publish_vel(float vx, float vy, float vw)
{
	twist_msg.linear.x = vx;
	twist_msg.linear.y = vy;
	twist_msg.angular.z = vw;
	// ROS_INFO("vx:%.2f,vy:%.2f,vw:%.2f",vx,vy,vw);
	cmd_vel_pub.publish(twist_msg);
}

/**
 * @brief 点到点直线T形加减速运动（仅支持x,y单个方向）
 *
 * @param angle 姿态角度
 * @param target_point 目标点
 * @param dir 方向1为x轴，2为y轴
 * @param acc 每次加速值
 * @param time 延时时间，加速间隔时间
 */
void Tspeed_move(float angle, float target_point, uint8_t dir, float acc, uint8_t time)
{
	float error_max = 0, error_now = 0, speed = 0, init_pos = 0;
	float total_move = 0, acc_displacement = 0;
	uint8_t acc_flag = 0; // 没有加速到最大速度标志位
	int8_t sgn_flag = 0;
	while (!init_flag)
		;
	if (dir == 1)
	{
		// ros::spinOnce();
		target_point += currant_state[0];
		init_pos = currant_state[0];
		// ROS_INFO("target:%f,init:%f",target_point,init_pos);
		error_max = target_point - currant_state[0];
		total_move = abs(error_max);
		error_now = target_point - currant_state[0];
		sgn_flag = (error_max > 0) ? 1 : -1;
		while (speed <= max_vel_x) // 加速过程
		{
			// ros::spinOnce();
			if (abs(error_now) > (float)(total_move / 2))
			{
				speed += acc;
				angle_loop(init_angle, sgn_flag * speed, 0);
				ros::Duration(time / 1000.0).sleep();
			}
			else
			{
				acc_flag = 1;
				break;
			}
			error_now = target_point - currant_state[0];
		}
		if (acc_flag == 1) // 无匀速过程，仅加减速
		{
			while (speed > 0)
			{
				speed -= acc;
				angle_loop(init_angle, sgn_flag * speed, 0);
				ros::Duration(time / 1000.0).sleep();
			}
			publish_vel(0, 0, 0);
			return;
		}
		else // T形加减速
		{
			acc_displacement = abs((float)currant_state[0] - init_pos);
			// ROS_INFO("acc_displacement:%f,error_now:%f",acc_displacement,error_now);
			while (abs(error_now) > acc_displacement) // 匀速过程
			{
				// ros::spinOnce();
				angle_loop(init_angle, sgn_flag * speed, 0);
				ros::Duration(1 / 1000.0).sleep();
				error_now = target_point - currant_state[0];
				// ROS_INFO("error_now:%f",error_now);
			}
			while (speed > 0) // 减速过程
			{
				speed -= acc;
				angle_loop(init_angle, sgn_flag * speed, 0);
				ros::Duration(time / 1000.0).sleep();
			}
			publish_vel(0, 0, 0);
			return;
		}
	}
	else if (dir == 2)
	{
		target_point += currant_state[1];
		init_pos = currant_state[1];
		error_max = target_point - currant_state[1];
		total_move = abs(error_max);
		error_now = target_point - currant_state[1];
		sgn_flag = (error_max > 0) ? 1 : -1;
		while (speed <= max_vel_y) // 加速过程
		{
			if (abs(error_now) > (float)(total_move / 2))
			{
				speed += acc;
				angle_loop(init_angle, 0, sgn_flag * speed);
				ros::Duration(time / 1000.0).sleep();
			}
			else
			{
				acc_flag = 1;
				break;
			}
			error_now = target_point - currant_state[1];
		}
		if (acc_flag == 1) // 无匀速过程，仅加减速
		{
			while (speed > 0)
			{
				speed -= acc;
				angle_loop(init_angle, 0, sgn_flag * speed);
				ros::Duration(time / 1000.0).sleep();
			}
			publish_vel(0, 0, 0);
			return;
		}
		else // T形加减速
		{
			acc_displacement = abs((float)currant_state[1] - init_pos);
			while (abs(error_now) > acc_displacement) // 匀速阶段
			{
				angle_loop(init_angle, 0, sgn_flag * speed);
				ros::Duration(1 / 1000.0).sleep();
				error_now = target_point - currant_state[1];
			}
			while (speed > 0) // 减速阶段
			{
				speed -= acc;
				angle_loop(init_angle, 0, sgn_flag * speed);
				ros::Duration(time / 1000.0).sleep();
			}
			publish_vel(0, 0, 0);
			return;
		}
	}
}
/**
 * @brief 里程计回调函数
 *
 * @param msg
 */
void odomCallback(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
	if (init_flag == 0)
	{
		init_angle = tf2::getYaw(msg.pose.pose.orientation);
		cout << "init_angle: " << init_angle << endl;
		init_flag = true;
	}
	currant_state[0] = msg.pose.pose.position.x;
	currant_state[1] = msg.pose.pose.position.y;
	currant_state[2] = tf2::getYaw(msg.pose.pose.orientation);
	ROS_INFO("x:%f,y:%f,z:%f", currant_state[0], currant_state[1], currant_state[2]);
}
/**
 * @brief IMU回调函数，暂时不用，使用里程计的角度信息
 * 
 * @param msg 
 */
void imuCallback(const sensor_msgs::Imu &msg)
{
	currant_state[2] = tf2::getYaw(msg.orientation);
	// ROS_INFO("Yaw:%.4f",currant_state[2]);
}

/**
 * @brief 细调位置环
 *TODO
 * @param goal_x
 * @param goal_y
 */
void position_loop(float goal_x, float goal_y)
{

	float x_out = x_pid.calc(goal_x, currant_state[0]);
	twist_msg.linear.x = x_out;
	float y_out = y_pid.calc(goal_y, currant_state[1]);
	twist_msg.linear.y = y_out;
	cmd_vel_pub.publish(twist_msg);
}
/**
 * @brief 角度环
 *
 * @param target_angle
 * @param vx
 * @param vy
 */
void angle_loop(float target_angle, float vx, float vy)
{
	if (!init_flag)
		return;
	float result = 0;
	if (abs(currant_state[2] - target_angle) < 0.001)
	{
		result = 0;
	}
	else
	{
		float angle_pid_value = angle_pid.calc(target_angle, currant_state[2]);
		result = -angle_pid_value;
		//   ROS_INFO("init_angle:%.4f,angle:%.4f,w:%.4f",target_angle,currant_state[2],angle_pid_value);
	}
	publish_vel(vx, vy, result);
}

void MySigintHandler(int sig)
{
	publish_vel(0, 0, 0);
	ros::shutdown();
}

void task(float x,float y)
{
	if(x==0.0){
		Tspeed_move(0, y, 2, acc, acc_time);
		ros::Duration(1).sleep();
	}
	if(y==0.0){
		Tspeed_move(0, x, 1, acc, acc_time); // 在一个单独的线程里执行
		ros::Duration(1).sleep();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pid_tracking_node");

	signal(SIGINT, MySigintHandler); // 自定义退出函数
	ros::NodeHandle nh;
	//获取参数
	nh.param<float>("x_max_vel", max_vel_x, 0.5);//x方向最大速度
	nh.param<float>("y_max_vel", max_vel_y, 0.5);//y方向最大速度
	nh.param<float>("w_max_vel", max_vel_w, 0.5);//w方向最大速度
	nh.param<float>("angle_kp", angle_k[0], 0.0);
	nh.param<float>("angle_ki", angle_k[1], 0.0);
	nh.param<float>("angle_kd", angle_k[2], 0.0);
	nh.param<float>("x_kp", x_k[0], 0.0);
	nh.param<float>("x_ki", x_k[1], 0.0);
	nh.param<float>("x_kd", x_k[2], 0.0);
	nh.param<float>("y_kp", y_k[0], 0.0);
	nh.param<float>("y_ki", y_k[1], 0.0);
	nh.param<float>("y_kd", y_k[2], 0.0);
	nh.param<string>("odom_topic", odom_topic, "odom"); //里程计节点
	nh.param<float>("acc", acc, 0.02);//加速度
	nh.param<int>("time", acc_time, 100);//加速间隔时间
	//获取目标点
	nh.param<int>("goal_point_num", goal_point_num, 0);//目标点数目
	
	for(int i = 0;i < goal_point_num;i++){
		pair<float,float> temp;
		nh.param<float>("goal_point" + to_string(i) + "_x", temp.first, -1.0);
    	nh.param<float>("goal_point" + to_string(i) + "_y", temp.second, -1.0);
		goal_point.emplace_back(temp);
	}
	//PID初始化
	angle_pid.init(PID_POSITION, angle_k, max_vel_w, 0.1 * max_vel_w);
	x_pid.init(PID_POSITION, x_k, max_vel_x, 0.01 * max_vel_x);
	y_pid.init(PID_POSITION, y_k, max_vel_y, 0.01 * max_vel_y);

	cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	// predict_path_pub = nh.advertise<nav_msgs::Path>("/predict_path", 1);
	// motion_path_pub = nh.advertise<nav_msgs::Path>("/motion_path", 1);

	ros::Subscriber _odom_sub = nh.subscribe(odom_topic, 1, &odomCallback);
	// control_cmd_pub = nh.createTimer(ros::Duration(0.1), angle_loop);
	// ros::spin();
	// 创建多线程AsyncSpinner
	ros::AsyncSpinner spinner(2); // 使用两个线程
	spinner.start();

	// 加减速函数放在主线程中执行
	std::thread control_thread([](){
		// for(int i = 0;i<goal_point.size();i++){
		// 	task(goal_point[i].first,goal_point[i].second);
		// }
		while(1){
			angle_loop(init_angle,0,0);
		}
	});

	control_thread.join(); // 等待线程关闭

	return 0;
}
