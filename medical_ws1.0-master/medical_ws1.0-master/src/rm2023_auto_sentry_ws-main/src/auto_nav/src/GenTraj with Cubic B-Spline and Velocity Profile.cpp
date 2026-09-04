#include <vector>
#include <cmath>
#include <ros/ros.h>  // For ROS logging if needed

// B-Spline Structures and Functions (transplanted and adapted for k=4)
struct Points {
    std::vector<double> points_x;
    std::vector<double> points_y;
};

class BSplineHelper {  // Encapsulate to avoid globals in RobotCtrl
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

    Points B_Spline(const std::vector<double>& x_set, const std::vector<double>& y_set) {
        int num = x_set.size();
        generateKnots(num);

        Points points_;
        double step = 0.1;  // Adjustable sampling step for denser points
        for (double i = 0.0; i <= static_cast<double>(num - k); i += step) {
            double x_ = bspline(num, i, x_set);
            double y_ = bspline(num, i, y_set);
            points_.points_x.push_back(x_);
            points_.points_y.push_back(y_);
        }
        return points_;
    }
};

// Member function in RobotCtrl class
void RobotCtrl::GenTraj(const nav_msgs::Path& prune_path, nav_msgs::Path& local_path) {
    BSplineHelper bspline_helper;  // Local instance for encapsulation

    // Extract control points from prune_path
    std::vector<double> x_set, y_set;
    for (const auto& pose : prune_path.poses) {
        x_set.push_back(pose.pose.position.x);
        y_set.push_back(pose.pose.position.y);
    }

    // Generate smoothed points using cubic B-spline
    Points points = bspline_helper.B_Spline(x_set, y_set);

    // Fill local_path with smoothed poses
    local_path.header.frame_id = prune_path.header.frame_id;
    local_path.poses.clear();
    for (size_t i = 0; i < points.points_x.size(); ++i) {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = local_path.header.frame_id;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.pose.position.x = points.points_x[i];
        pose_stamped.pose.position.y = points.points_y[i];
        pose_stamped.pose.position.z = 0.0;
        pose_stamped.pose.orientation.w = 1.0;  // Default quaternion (no rotation)
        local_path.poses.push_back(pose_stamped);
    }

    // Generate reference velocity profile with acceleration limits
    GenerateRefVelocity(local_path, points);
}

// Helper function for velocity profile (add as private member in RobotCtrl)
void RobotCtrl::GenerateRefVelocity(nav_msgs::Path& local_path, const Points& points) {
    if (points.points_x.empty()) return;

    // Adjustable parameters (read from ROS params or hardcode)
    double max_vel = max_x_speed_;  // From class member, adjustable via ROS param
    double dt = 1.0 / plan_freq_;   // Planning period, adjustable via plan_freq_
    double sampling_step = 0.1;     // B-spline sampling step, adjustable for density

    // Velocity zones and corresponding acceleration limits (adjustable arrays)
    std::vector<double> v_zones = {0.5, 1.0};  // Speed thresholds (m/s), e.g., low/medium/high
    std::vector<double> a_max_zones = {2.0, 1.0};  // Max accel for each zone (m/s²), size = v_zones.size() + 1 implied
    a_max_zones.insert(a_max_zones.begin(), 3.0);  // Default high accel for v < v_zones[0]

    // Compute total arc length for time parameterization
    double total_arc = 0.0;
    for (size_t i = 1; i < points.points_x.size(); ++i) {
        double dx = points.points_x[i] - points.points_x[i - 1];
        double dy = points.points_y[i] - points.points_y[i - 1];
        total_arc += std::sqrt(dx * dx + dy * dy);
    }
    double total_time = total_arc / max_vel;  // Approximate total time

    // Generate velocity profile
    std::vector<double> ref_vels;
    double current_vel = 0.0;
    double current_acc = 0.0;
    size_t n_points = points.points_x.size();
    for (size_t i = 0; i < n_points; ++i) {
        // Parameterize time: t = (i / (n_points - 1)) * total_time
        double t = (static_cast<double>(i) / (n_points - 1)) * total_time;

        // Select a_max based on current_vel zone
        double a_max = a_max_zones[0];  // Default
        for (size_t z = 0; z < v_zones.size(); ++z) {
            if (current_vel > v_zones[z]) {
                a_max = a_max_zones[z + 1];
            } else {
                break;
            }
        }

        // Trapezoidal-like acceleration: ramp up/down with zone-specific limit
        if (t < total_time * 0.3) {  // Acceleration phase (adjustable fractions)
            current_acc = std::min(a_max, (max_vel - current_vel) / (total_time * 0.3));
        } else if (t > total_time * 0.7) {  // Deceleration phase
            current_acc = std::max(-a_max, (0.0 - current_vel) / (total_time * 0.3));
        } else {
            current_acc = 0.0;  // Cruise
        }

        current_vel += current_acc * dt;
        current_vel = std::max(0.0, std::min(max_vel, current_vel));

        ref_vels.push_back(current_vel);
    }

    // Store ref_vel in local_path.poses[i].pose.orientation.x for feedforward
    for (size_t i = 0; i < local_path.poses.size(); ++i) {
        local_path.poses[i].pose.orientation.x = ref_vels[i];
    }

    ROS_INFO("Generated velocity profile: max_vel=%.2f, zones=[%.1f/%.1f], a_max=[%.1f/%.1f/%.1f]",
             max_vel, v_zones[0], v_zones[1], a_max_zones[0], a_max_zones[1], a_max_zones[2]);
}