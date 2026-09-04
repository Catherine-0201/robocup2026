#include "Bspline_follow.h"
#include <std_msgs/Int32.h>

// B-Spline Helper类定义
class BSplineHelper {
private:
    int k = 4;  // Cubic B-spline (degree 3)
    std::vector<double> knots;  // Use double for flexibility

    // Recursive B-spline basis function (Cox-de Boor)
    double Bspline(int index, int order, double u) {
        double coef1, coef2;
        if (order == 1) {
            if (index == 0) {
                if (knots[index] <= u && u <= knots[index + 1]) return 1.0;
            }
            if (knots[index] < u && u <= knots[index + 1]) return 1.0;
            return 0.0;
        } else {
            if (knots[index + order - 1] == knots[index]) {
                coef1 = (u == knots[index]) ? 1.0 : 0.0;
            } else {
                coef1 = (u - knots[index]) / (knots[index + order - 1] - knots[index]);
            }
            if (knots[index + order] == knots[index + 1]) {
                coef2 = (u == knots[index + order]) ? 1.0 : 0.0;
            } else {
                coef2 = (knots[index + order] - u) / (knots[index + order] - knots[index + 1]);
            }
            return (coef1 * Bspline(index, order - 1, u) + coef2 * Bspline(index + 1, order - 1, u));
        }
    }

    double bspline(int num, double u, const std::vector<double>& c) {
        double C = 0.0;
        for (int i = 0; i < num; ++i) {
            C += c[i] * Bspline(i, k, u);
        }
        return C;
    }

public:
    // Generate knots for uniform clamped B-spline
    void generateKnots(int num) {
        knots.clear();
        for (int i = 0; i <= num + k; ++i) {
            if (i <= k) {
                knots.push_back(0.0);
            } else if (i >= num) {
                knots.push_back(static_cast<double>(num - k));
            } else {
                knots.push_back(static_cast<double>(i - k));
            }
        }
    }

    Points B_Spline(const std::vector<double>& x_set, const std::vector<double>& y_set, double step = 0.1) {
        int num = x_set.size();
        generateKnots(num);

        Points points_;
        for (double i = 0.0; i <= static_cast<double>(num - k); i += step) {
            double x_ = bspline(num, i, x_set);
            double y_ = bspline(num, i, y_set);
            points_.points_x.push_back(x_);
            points_.points_y.push_back(y_);
        }
        return points_;
    }
};

RobotCtrl::RobotCtrl() {
    // 初始化 ROS 参数
    // launch中写在<node>内部的参数位于私有命名空间，必须通过"~"读取。
    ros::NodeHandle private_nh("~");
    private_nh.param("a_max", a_max_, 2.0);    // 线加速度上限
    private_nh.param("d_max", d_max_, 2.0);    // 线减速度上限
    private_nh.param("a_lat_max", a_lat_max_, 1.2); // 横向加速度上限
    private_nh.param("max_linear_speed", max_linear_speed_, 0.5); // 最大线速度限制
    private_nh.param("max_angular_speed", max_angular_speed_, 1.0); // 最大角速度限制
    private_nh.param("lookahead_idx", lookahead_idx_, 3); // 前视索引
    private_nh.param("goal_dist_tolerance", goal_dist_tolerance_, 0.2); // 目标距离容忍度
    private_nh.param("yaw_pid", yaw_pid_, 1.0); // yaw角度环PID参数
    private_nh.param("path_goal_match_tolerance", path_goal_match_tolerance_, 0.15);
    private_nh.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");

    // 初始化成员变量
    yaw_ = 0.0;
    initial_yaw_ = 0.0;  // 初始姿态，将在IMU回调中设置
    plan_ = false;
    has_global_path_ = false;

    // 初始化新变量
    target_done_ = 0;
    target_complete_ = 1;
    active_goal_received_ = false;
    active_goal_path_ready_ = false;
    former_goal_.pose.position.x = 0.0;
    former_goal_.pose.position.y = 0.0;
    former_goal_.pose.position.z = 0.0;

    // 初始化订阅和发布
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 10);
    local_path_pub_ = nh_.advertise<nav_msgs::Path>("/bspline_trajectory", 10);  // 发布B样条轨迹用于可视化
    target_done_pub_ = nh_.advertise<std_msgs::Int32>("/target_done", 10, true);  // 发布/target_done（锁存）
    move_base_goal_sub_ = nh_.subscribe("/move_base/goal", 10, &RobotCtrl::MoveBaseGoalCallback, this);
    move_base_simple_goal_sub_ = nh_.subscribe(
        "/move_base_simple/goal", 10, &RobotCtrl::MoveBaseSimpleGoalCallback, this);
    global_path_sub_ = nh_.subscribe("/move_base/GlobalPlanner/plan", 10, &RobotCtrl::GlobalPathCallback, this);
    imu_sub_ = nh_.subscribe("/handsfree/imu", 10, &RobotCtrl::ImuCallback, this);
    tf_listener_ = std::make_shared<tf::TransformListener>();
    
    // 初始发布target_done为0
    std_msgs::Int32 target_done_msg;
    target_done_msg.data = 0;
    target_done_pub_.publish(target_done_msg);
    ROS_INFO("初始发布target_done: 0");
    ROS_INFO("B样条速度输出话题: %s", cmd_vel_topic_.c_str());
}

RobotCtrl::~RobotCtrl() {
    // move_base客户端已移除
}

void RobotCtrl::GenTraj(const nav_msgs::Path& prune_path, nav_msgs::Path& local_path) {
    // ROS_INFO("GenTraj called with %zu input poses", prune_path.poses.size());
    BSplineHelper bspline_helper;

    local_path.header = prune_path.header;
    local_path.poses.clear();
    ref_vels_.clear();
    ref_times_.clear();

    if (prune_path.poses.empty()) {
        ROS_WARN("Cannot generate trajectory: global path is empty");
        return;
    }

    // 1) 提取控制点
    std::vector<double> x_set, y_set;
    x_set.reserve(prune_path.poses.size());
    y_set.reserve(prune_path.poses.size());
    for (const auto& pose : prune_path.poses) {
        x_set.push_back(pose.pose.position.x);
        y_set.push_back(pose.pose.position.y);
    }

    // 2) 使用 B-Spline 平滑路径
    Points points;
    if (x_set.size() < 4) {
        // A cubic B-spline needs at least four control points.  Densify a
        // short path instead of using only 1-3 samples: a two-point raw path
        // can otherwise remain at its zero-speed first sample whenever
        // move_base replans and resets the trajectory clock.
        points.points_x.push_back(x_set.front());
        points.points_y.push_back(y_set.front());
        for (size_t i = 1; i < x_set.size(); ++i) {
            const double dx = x_set[i] - x_set[i - 1];
            const double dy = y_set[i] - y_set[i - 1];
            const double distance = std::hypot(dx, dy);
            const size_t samples = std::max<size_t>(
                1, static_cast<size_t>(std::ceil(distance / 0.05)));
            for (size_t sample = 1; sample <= samples; ++sample) {
                const double ratio = static_cast<double>(sample) /
                                     static_cast<double>(samples);
                points.points_x.push_back(x_set[i - 1] + ratio * dx);
                points.points_y.push_back(y_set[i - 1] + ratio * dy);
            }
        }
        ROS_WARN("Global path has only %zu point(s); generated %zu linear samples",
                 x_set.size(), points.points_x.size());
    } else {
        points = bspline_helper.B_Spline(x_set, y_set, 0.1);
    }
    // ROS_INFO("B_Spline generated %zu points", points.points_x.size());

    if (points.points_x.empty() || points.points_x.size() != points.points_y.size()) {
        ROS_ERROR("Cannot generate trajectory: B-spline output is empty or inconsistent");
        return;
    }

    // 3) 计算弧长
    std::vector<double> s = ComputeArc(points);

    // 4) 计算路径曲率
    std::vector<double> kappa = ComputeCurvature(points);

    // 5) 计算速度限制（包括曲率限制）
    std::vector<double> vlimit = BuildVelocityLimit(points, s);

    // 6) 进行前向/后向扫描，计算速度
    ref_vels_ = TimeParameterize(points, s, vlimit);

    // 7) 计算时间戳
    ref_times_ = BuildTimeStamps(s, ref_vels_);

    // 8) 记录轨迹基准时间
    traj_base_time_ = ros::Time::now();

    // 填充可视化路径（不再将速度存入 orientation.x）
    local_path.header.frame_id = prune_path.header.frame_id;
    for (size_t i = 0; i < points.points_x.size(); ++i) {
        geometry_msgs::PoseStamped p;
        p.header.frame_id = local_path.header.frame_id;
        p.header.stamp = ros::Time::now();  // 当前时间戳
        p.pose.position.x = points.points_x[i];
        p.pose.position.y = points.points_y[i];
        p.pose.position.z = 0.0;
        p.pose.orientation.w = 1.0;  // 设置为单位四元数
        local_path.poses.push_back(p);
    }
    // ROS_INFO("local_path has %zu poses", local_path.poses.size());

    // ROS_INFO("Trajectory prepared: N=%zu, Total Time=%.2f s, Total Distance=%.2f m, Max Velocity=%.2f",
    //          ref_times_.size(),
    //          (ref_times_.empty() ? 0.0 : ref_times_.back()),
    //          (s.empty() ? 0.0 : s.back()),
    //          *std::max_element(ref_vels_.begin(), ref_vels_.end()));
    
    // 发布B样条轨迹用于RViz可视化
    if (!local_path.poses.empty()) {
        local_path_pub_.publish(local_path);
        // ROS_INFO("Published B-spline trajectory to /bspline_trajectory for visualization");
    } else {
        ROS_WARN("Skipping publish: empty trajectory - check if input path has at least 4 points for cubic B-spline");
    }
}

// 工具函数：计算弧长
std::vector<double> RobotCtrl::ComputeArc(const Points& pts) {
    std::vector<double> s(pts.points_x.size(), 0.0);
    for (size_t i = 1; i < pts.points_x.size(); ++i) {
        double dx = pts.points_x[i] - pts.points_x[i-1];
        double dy = pts.points_y[i] - pts.points_y[i-1];
        s[i] = s[i-1] + std::hypot(dx, dy);
    }
    return s;
}

// 工具函数：计算曲率（三点法）
std::vector<double> RobotCtrl::ComputeCurvature(const Points& pts) {
    size_t n = pts.points_x.size();
    std::vector<double> kappa(n, 0.0);
    if (n < 3) return kappa;

    for (size_t i = 1; i + 1 < n; ++i) {
        double x1 = pts.points_x[i-1], y1 = pts.points_y[i-1];
        double x2 = pts.points_x[i],   y2 = pts.points_y[i];
        double x3 = pts.points_x[i+1], y3 = pts.points_y[i+1];

        double a = std::hypot(x2-x1, y2-y1);
        double b = std::hypot(x3-x2, y3-y2);
        double c = std::hypot(x3-x1, y3-y1);
        double area2 = std::abs((x2-x1)*(y3-y1) - (y2-y1)*(x3-x1)); // 2*三角形面积
        double denom = a * b * c;

        if (denom > 1e-6) {
            kappa[i] = 2.0 * area2 / denom;
        } else {
            kappa[i] = 0.0;
        }
    }

    kappa[0] = kappa[1];
    if (n >= 3) kappa[n-1] = kappa[n-2];
    return kappa;
}

// 速度限制（包括曲率限速和最大线速度限制）
std::vector<double> RobotCtrl::BuildVelocityLimit(const Points& pts, const std::vector<double>& s) {
    size_t n = pts.points_x.size();
    std::vector<double> vlimit(n, max_linear_speed_);

    auto kappa = ComputeCurvature(pts);
    for (size_t i = 0; i < n; ++i) {
        // 曲率限速（考虑横向加速度限制）
        double v_curve = std::sqrt(std::max(0.0, a_lat_max_ / std::max(1e-6, std::abs(kappa[i]))));
        vlimit[i] = std::min(vlimit[i], v_curve);
        
        // 应用最大线速度限制
        vlimit[i] = std::min(vlimit[i], max_linear_speed_);
    }
    return vlimit;
}

// 前向/后向扫描，施加加减速度约束和速度限制
std::vector<double> RobotCtrl::TimeParameterize(const Points& pts, const std::vector<double>& s, const std::vector<double>& vlimit) {
    size_t n = s.size();
    if (n == 0 || vlimit.size() != n || pts.points_x.size() != n || pts.points_y.size() != n) {
        ROS_WARN("Cannot parameterize trajectory: path arrays are empty or inconsistent");
        return {};
    }
    std::vector<double> v(n, 0.0);
    v[0] = 0.0;                 // 起点速度

    // 前向扫描：加速度限制
    for (size_t i = 1; i < n; ++i) {
        double ds = s[i] - s[i-1];
        double v_prev = v[i-1];
        double v_max_acc = std::sqrt(std::max(0.0, v_prev * v_prev + 2.0 * a_max_ * ds));
        v[i] = std::min(v_max_acc, vlimit[i]); // 应用速度限制
    }

    // The robot must approach every waypoint with zero terminal velocity.
    // Without this, missing a tight distance threshold can leave the last
    // non-zero reference velocity active near a wall.
    v[n - 1] = 0.0;

    // 后向扫描：减速度限制
    for (size_t i = n-2; i < n; --i) {  // 防止 size_t 下溢
        double ds = s[i+1] - s[i];
        double v_next = v[i+1];
        double v_max_dec = std::sqrt(std::max(0.0, v_next * v_next + 2.0 * d_max_ * ds));
        v[i] = std::min(v[i], v_max_dec);
        v[i] = std::min(v[i], vlimit[i]); // 应用速度限制
        if (i == 0) break; // 防止下溢
    }

    return v;
}

// 由 v 和弧长得到时间戳
std::vector<double> RobotCtrl::BuildTimeStamps(const std::vector<double>& s, const std::vector<double>& v) {
    size_t n = s.size();
    if (n == 0 || v.size() != n) {
        ROS_WARN("Cannot build trajectory timestamps: arc and velocity arrays are empty or inconsistent");
        return {};
    }
    std::vector<double> t(n, 0.0);
    for (size_t i = 1; i < n; ++i) {
        double ds = s[i] - s[i-1];
        double v_mid = std::max(0.05, 0.5 * (v[i] + v[i-1])); // 防止除零
        double dt = ds / v_mid;
        t[i] = t[i-1] + dt;
    }
    return t;
}

// 轨迹跟踪
void RobotCtrl::FollowTraj(const geometry_msgs::PoseStamped& robot_pose,
                           const nav_msgs::Path& traj,
                           geometry_msgs::Twist& cmd_vel) {

    cmd_vel = geometry_msgs::Twist();

    // Arrival is a latched stop state.  move_base can continue replanning the
    // same action goal at planner_frequency, but the robot must remain stopped
    // until MoveBaseGoalCallback receives a genuinely new goal_id.
    if (active_goal_received_ && target_complete_ == 1) {
        return;
    }

    if (traj.poses.empty() || ref_times_.empty() || ref_vels_.empty() ||
        traj.poses.size() != ref_times_.size() ||
        traj.poses.size() != ref_vels_.size()) {
        ROS_WARN_THROTTLE(1.0, "Skipping trajectory tracking: trajectory data is empty or inconsistent");
        return;
    }

    // 获取当前时间和参考点
    double t_now = (ros::Time::now() - traj_base_time_).toSec();
    size_t k = std::lower_bound(ref_times_.begin(), ref_times_.end(), t_now) - ref_times_.begin();
    if (k >= ref_times_.size()) {
        k = ref_times_.size() - 1;
    }

    // 获取参考速度
    double v_ref = ref_vels_[k];  // 参考速度

    // 计算参考方向
    size_t km1 = (k > 0) ? k - 1 : std::min(k + 1, traj.poses.size() - 1);
    double tx = traj.poses[k].pose.position.x - traj.poses[km1].pose.position.x;
    double ty = traj.poses[k].pose.position.y - traj.poses[km1].pose.position.y;
    double heading_ref = std::atan2(ty, tx);
    // double e_yaw = NormalizeAngle(heading_ref - 0.0);  // 计算角度误差，机器人始终朝向正前方
    double e_yaw = NormalizeAngle(heading_ref - yaw_);
    // 前馈速度控制
    double vx_global = v_ref * std::cos(e_yaw);
    double vy_global = v_ref * std::sin(e_yaw);
    cmd_vel.linear.x = vx_global * std::cos(yaw_) + vy_global * std::sin(yaw_);
    cmd_vel.linear.y = -vx_global * std::sin(yaw_) + vy_global * std::cos(yaw_);

    // 角速度控制（保持初始姿态，应用最大角速度限制）
    // 与threenav.py第二阶段逻辑一致：恢复到初始姿态
    // 临时注释掉角度环控制部分
    
    double yaw_err = NormalizeAngle(initial_yaw_ - yaw_);
    double angular_vel = yaw_pid_ * yaw_err;  // 比例控制，使用参数化的Kp_yaw
    cmd_vel.angular.z = std::max(-max_angular_speed_, std::min(max_angular_speed_, angular_vel));  // 应用角速度限制
    
    // 临时设置角速度为0
    // cmd_vel.angular.z = 0.0;

    // 终点处理：仅基于目标距离容忍度判断，到达目标后停止机器人
    const auto& goal = traj.poses.back();
    double dist_goal = GetEuclideanDistance(robot_pose, goal);
    
    // 添加详细的调试信息
    static int debug_count = 0;
    debug_count++;
    if (debug_count % 20 == 1) {  // 每2秒输出一次（10Hz * 2s = 20）
        ROS_INFO("current (%.3f, %.3f), goal(%.3f, %.3f), distance%.3f m, tolerance %.3f m", 
                 robot_pose.pose.position.x, robot_pose.pose.position.y,
                 goal.pose.position.x, goal.pose.position.y,
                 dist_goal, goal_dist_tolerance_);
    }
    
    if (dist_goal < goal_dist_tolerance_) {
        ROS_INFO("=== reach goal ===");
        ROS_INFO("distance: %.3f m < tolerance: %.3f m", dist_goal, goal_dist_tolerance_);
        
        // 停止机器人
        cmd_vel.linear.x = cmd_vel.linear.y = cmd_vel.angular.z = 0.0;
        ROS_INFO("velocity command is zero");
        
        // 检查target_complete状态并发布/target_done
        if (active_goal_path_ready_ && target_complete_ == 0) {
            target_complete_ = 1;
            active_goal_path_ready_ = false;
            target_done_++;
            std_msgs::Int32 done_msg;
            done_msg.data = target_done_;
            target_done_pub_.publish(done_msg);
            ROS_INFO("target_complete set to 1,target_done=%d,/target_done", target_done_);
        }
        
        ROS_INFO("=== reach goal process completed ===");
    }
    // cmd_vel.linear.z=cmd_vel.linear.x;
    // cmd_vel.linear.x=cmd_vel.linear.y;
    // cmd_vel.linear.y=cmd_vel.linear.z;
}

void RobotCtrl::MoveBaseGoalCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg) {
    // move_base每接受一次新的action goal都会产生新的goal_id。使用goal_id而不是
    // GlobalPlanner路径末端的浮点数来区分目标，可避免重规划把同一目标重复计数。
    if (active_goal_received_ && msg->goal_id.id == active_goal_id_) {
        return;
    }

    active_goal_id_ = msg->goal_id.id;
    active_goal_ = msg->goal.target_pose;
    active_goal_received_ = true;
    active_goal_path_ready_ = false;
    target_complete_ = 0;
    plan_ = false;
    has_global_path_ = false;

    // 新目标与对应的新全局路径之间存在短暂时间差，必须清掉旧轨迹，否则机器人
    // 仍位于旧终点时会把新目标误判成已经完成。
    global_path_.poses.clear();
    local_path_.poses.clear();
    ref_vels_.clear();
    ref_times_.clear();

    geometry_msgs::Twist stop;
    cmd_vel_pub_.publish(stop);

    ROS_INFO("收到新导航目标: id=%s, x=%.3f, y=%.3f，等待对应全局路径",
             active_goal_id_.c_str(),
             active_goal_.pose.position.x,
             active_goal_.pose.position.y);
}

void RobotCtrl::MoveBaseSimpleGoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    // RViz 2D Nav Goal is published on /move_base_simple/goal.  It does not
    // reliably pass through /move_base/goal on every move_base setup, so treat
    // every simple-goal message as an explicit new navigation command.
    active_goal_id_ = "rviz_simple_goal";
    active_goal_ = *msg;
    active_goal_received_ = true;
    active_goal_path_ready_ = false;
    target_complete_ = 0;
    plan_ = false;
    has_global_path_ = false;

    global_path_.poses.clear();
    local_path_.poses.clear();
    ref_vels_.clear();
    ref_times_.clear();

    geometry_msgs::Twist stop;
    cmd_vel_pub_.publish(stop);

    ROS_INFO("Received new RViz navigation goal: x=%.3f, y=%.3f; waiting for matching global path",
             active_goal_.pose.position.x,
             active_goal_.pose.position.y);
}

// 辅助函数实现
double RobotCtrl::NormalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

double RobotCtrl::GetEuclideanDistance(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2) {
    double dx = pose1.pose.position.x - pose2.pose.position.x;
    double dy = pose1.pose.position.y - pose2.pose.position.y;
    return std::sqrt(dx * dx + dy * dy);
}

void RobotCtrl::GlobalPathCallback(const nav_msgs::Path::ConstPtr& msg) {
    if (!msg->poses.empty()) {
        // Ignore repeated plans for an action goal that this controller has
        // already completed.  A new goal_id resets target_complete_ to zero in
        // MoveBaseGoalCallback and unlocks the next navigation segment.
        if (active_goal_received_ && target_complete_ == 1) {
            ROS_INFO_THROTTLE(2.0,
                              "Current navigation goal is complete; ignoring repeated global plan");
            return;
        }

        const geometry_msgs::PoseStamped& path_goal = msg->poses.back();

        // 新action goal到达后，move_base可能还会短暂发布上一目标的旧路径。
        // 只接受终点与当前action目标匹配的路径，避免再次执行旧轨迹。
        if (active_goal_received_) {
            const double goal_error = GetEuclideanDistance(path_goal, active_goal_);
            if (goal_error > path_goal_match_tolerance_) {
                ROS_WARN_THROTTLE(1.0,
                                  "忽略旧全局路径: 路径终点与当前目标相差 %.3f m",
                                  goal_error);
                return;
            }
            active_goal_path_ready_ = true;
        } else {
            // 兼容只发布全局Path、不经过move_base action接口的测试工具。
            const double goal_change = GetEuclideanDistance(path_goal, former_goal_);
            if (goal_change > path_goal_match_tolerance_ || target_done_ == 0) {
                target_complete_ = 0;
            }
            active_goal_path_ready_ = true;
        }

        global_path_ = *msg;
        plan_ = true;
        has_global_path_ = true;
        former_goal_ = path_goal;
        
        ROS_INFO("Received global path with %zu waypoints", msg->poses.size());
    }
}

void RobotCtrl::ImuCallback(const sensor_msgs::Imu& msg) {
    // 从IMU数据中提取yaw角度
    tf::Quaternion q(
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    );
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    yaw_ = yaw;
    
    // 在第一次接收到IMU数据时设置初始姿态（与threenav.py逻辑一致）
    static bool first_imu_received = false;
    if (!first_imu_received) {
        initial_yaw_ = yaw;
        first_imu_received = true;
        ROS_INFO("initial IMU: yaw=%.3frad (%.1f°)", initial_yaw_, initial_yaw_ * 180.0 / M_PI);
    }
}

// 获取机器人当前位置
void RobotCtrl::GetRobotPose(geometry_msgs::PoseStamped& robot_pose) {
    robot_pose.header.frame_id = "map";
    robot_pose.header.stamp = ros::Time::now();
    
    // 从tf变换获取机器人真实位姿
    try {
        tf::StampedTransform transform;
        tf_listener_->lookupTransform("map", "base_link", ros::Time(0), transform);
        
        // 获取xy位置
        robot_pose.pose.position.x = transform.getOrigin().x();
        robot_pose.pose.position.y = transform.getOrigin().y();
        robot_pose.pose.position.z = transform.getOrigin().z();
        
        // 注意：yaw角度仍然从IMU获取，不从tf获取
        // 使用IMU的yaw角度（保持原有逻辑）
        tf::Quaternion q = tf::createQuaternionFromYaw(yaw_);
        robot_pose.pose.orientation.x = q.x();
        robot_pose.pose.orientation.y = q.y();
        robot_pose.pose.orientation.z = q.z();
        robot_pose.pose.orientation.w = q.w();
        
        // 调试信息：确认tf获取成功
        static int tf_success_count = 0;
        tf_success_count++;
        if (tf_success_count % 50 == 1) {  // 每5秒输出一次（10Hz * 5s = 50）
            ROS_INFO("TFget success - robotpose: x=%.3f, y=%.3f , yaw=%.3f° ", 
                     robot_pose.pose.position.x, robot_pose.pose.position.y, yaw_ * 180.0 / M_PI);
        }
        
    } catch (tf::TransformException &ex) {
        // tf获取失败时的处理
        ROS_WARN("TF变换获取失败: %s", ex.what());
        ROS_WARN("使用默认位姿 (0,0)，可能导致目标检测不准确");
        
        // 使用默认位姿
        robot_pose.pose.position.x = 0.0;
        robot_pose.pose.position.y = 0.0;
        robot_pose.pose.position.z = 0.0;
        
        // 使用IMU的yaw角度
        tf::Quaternion q = tf::createQuaternionFromYaw(yaw_);
        robot_pose.pose.orientation.x = q.x();
        robot_pose.pose.orientation.y = q.y();
        robot_pose.pose.orientation.z = q.z();
        robot_pose.pose.orientation.w = q.w();
    }
}

int main(int argc, char** argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "test_Bspline_follow");  // 节点名为 "test_Bspline_follow"
    ros::NodeHandle nh;  // 创建 ROS 节点句柄

    // 创建 RobotCtrl 对象
    RobotCtrl robotctrl;

    // 设置定时器（10Hz）
    ros::Rate loop_rate(10);

    while (ros::ok()) {
        // 1. 处理来自其他 ROS 节点的消息（例如全局路径）
        ros::spinOnce();  // 处理所有的回调函数，尤其是 GlobalPathCallback

        // 2. 如果接收到有效路径，生成轨迹
        if (robotctrl.hasPlan()) {
            // 路径有效时，生成轨迹
            nav_msgs::Path local_path;
            robotctrl.GenTraj(robotctrl.getGlobalPath(), local_path);
            robotctrl.setLocalPath(local_path);  // 存储生成的轨迹
            robotctrl.setPlan(false);  // 生成完轨迹后，设置标志为 false
        }

        // 3. 进行路径跟踪
        if (!robotctrl.getLocalPath().poses.empty()) {
            // 获取机器人当前位置
            geometry_msgs::PoseStamped robot_pose;
            robotctrl.getRobotPose(robot_pose);
            
            // 跟踪生成的轨迹
            geometry_msgs::Twist cmd_vel;
            robotctrl.FollowTraj(robot_pose, robotctrl.getLocalPath(), cmd_vel);
            robotctrl.publishCmdVel(cmd_vel);  // 发布速度指令
        }

        // 4. 控制循环频率
        loop_rate.sleep();  // 控制循环频率为 10Hz
    }

    return 0;
}
