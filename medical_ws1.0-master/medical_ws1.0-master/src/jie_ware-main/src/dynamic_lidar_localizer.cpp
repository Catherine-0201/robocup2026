#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <opencv2/opencv.hpp>

#include <cmath>
#include <algorithm>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {
struct Pose2D { double x = 0.0, y = 0.0, yaw = 0.0; };
struct Quality { int valid = 0, static_inliers = 0; double score = 0.0; };
struct ScanPoint {
  cv::Point2f base;
  double ray_weight = 0.0;
};

double normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}
}

class DynamicLidarLocalizer {
 public:
  DynamicLidarLocalizer()
      : nh_(), pnh_("~"), tf_listener_(tf_buffer_) {
    pnh_.param<std::string>("base_frame", base_frame_, "base_footprint");
    pnh_.param<std::string>("odom_frame", odom_frame_, "odom");
    pnh_.param<std::string>("laser_frame", laser_frame_, "laser");
    pnh_.param<std::string>("laser_topic", laser_topic_, "scan");
    pnh_.param("static_distance_m", static_distance_m_, 0.18);
    pnh_.param("dynamic_distance_m", dynamic_distance_m_, 0.25);
    pnh_.param("uncertain_weight", uncertain_weight_, 0.25);
    pnh_.param("min_valid_points", min_valid_points_, 50);
    pnh_.param("min_static_inlier_ratio", min_static_inlier_ratio_, 0.35);
    pnh_.param("min_score_per_point", min_score_per_point_, 0.18);
    pnh_.param("raycast_foreground_margin_m", raycast_foreground_margin_m_, 0.12);
    pnh_.param("raycast_wall_tolerance_m", raycast_wall_tolerance_m_, 0.20);
    pnh_.param("raycast_step_m", raycast_step_m_, 0.025);
    pnh_.param("invert_laser_y", invert_laser_y_, false);

    map_sub_ = nh_.subscribe("map", 1, &DynamicLidarLocalizer::mapCallback, this);
    scan_sub_ = nh_.subscribe(laser_topic_, 1, &DynamicLidarLocalizer::scanCallback, this);
    initial_pose_sub_ = nh_.subscribe("initialpose", 1, &DynamicLidarLocalizer::initialPoseCallback, this);
    quality_pub_ = nh_.advertise<std_msgs::String>("dynamic_lidar_loc/quality", 10);
    status_pub_ = nh_.advertise<std_msgs::Bool>("dynamic_lidar_loc/status", 10);
    static_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan_static_for_loc", 1);
    dynamic_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan_dynamic_removed", 1);
  }

 private:
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    map_ = *msg;
    const size_t width = static_cast<size_t>(map_.info.width);
    const size_t height = static_cast<size_t>(map_.info.height);
    const size_t cell_count = width * height;
    if (width == 0 || height == 0 || map_.info.resolution <= 0.0 ||
        map_.data.size() != cell_count) {
      ROS_ERROR("dynamic_lidar_localizer: invalid map w=%zu h=%zu data=%zu resolution=%.6f",
                width, height, map_.data.size(), map_.info.resolution);
      map_ready_ = false;
      return;
    }

    // Two-pass chamfer distance field.  This deliberately avoids OpenCV's
    // Mat/distanceTransform path, which can abort on some Jetson OpenCV builds.
    const float inf = std::numeric_limits<float>::infinity();
    const float straight = map_.info.resolution;
    const float diagonal = straight * static_cast<float>(std::sqrt(2.0));
    distance_field_m_.assign(cell_count, inf);
    size_t occupied_count = 0;
    for (size_t i = 0; i < cell_count; ++i) {
      if (map_.data[i] >= 65) {
        distance_field_m_[i] = 0.0f;
        ++occupied_count;
      }
    }
    if (occupied_count == 0) {
      ROS_ERROR("dynamic_lidar_localizer: map contains no occupied cells");
      map_ready_ = false;
      return;
    }

    const auto relax = [this](size_t dst, size_t src, float cost) {
      if (std::isfinite(distance_field_m_[src]))
        distance_field_m_[dst] = std::min(distance_field_m_[dst], distance_field_m_[src] + cost);
    };
    for (size_t y = 0; y < height; ++y) {
      for (size_t x = 0; x < width; ++x) {
        const size_t i = y * width + x;
        if (x > 0) relax(i, i - 1, straight);
        if (y > 0) relax(i, i - width, straight);
        if (x > 0 && y > 0) relax(i, i - width - 1, diagonal);
        if (x + 1 < width && y > 0) relax(i, i - width + 1, diagonal);
      }
    }
    for (size_t y = height; y-- > 0;) {
      for (size_t x = width; x-- > 0;) {
        const size_t i = y * width + x;
        if (x + 1 < width) relax(i, i + 1, straight);
        if (y + 1 < height) relax(i, i + width, straight);
        if (x + 1 < width && y + 1 < height) relax(i, i + width + 1, diagonal);
        if (x > 0 && y + 1 < height) relax(i, i + width - 1, diagonal);
      }
    }
    map_ready_ = true;
    ROS_INFO("dynamic_lidar_localizer: map distance field ready (%zux%zu, %.3fm, occupied=%zu)",
             width, height, map_.info.resolution, occupied_count);
  }

  void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    pose_.x = msg->pose.pose.position.x;
    pose_.y = msg->pose.pose.position.y;
    pose_.yaw = yaw;
    pose_initialized_ = true;
    ROS_INFO("dynamic_lidar_localizer: initial pose received");
  }

  bool distanceAndWeight(double x, double y, double* weight, bool* is_static) const {
    *weight = 0.0;
    *is_static = false;
    const int gx = static_cast<int>(std::floor((x - map_.info.origin.position.x) / map_.info.resolution));
    const int gy = static_cast<int>(std::floor((y - map_.info.origin.position.y) / map_.info.resolution));
    if (gx < 0 || gy < 0 || gx >= static_cast<int>(map_.info.width) ||
        gy >= static_cast<int>(map_.info.height)) return false;
    const size_t index = static_cast<size_t>(gy) * map_.info.width + static_cast<size_t>(gx);
    if (index >= distance_field_m_.size()) return false;
    const double d = distance_field_m_[index];
    if (d <= static_distance_m_) {
      *weight = 1.0;
      *is_static = true;
    } else if (d < dynamic_distance_m_) {
      const double span = std::max(1e-6, dynamic_distance_m_ - static_distance_m_);
      *weight = uncertain_weight_ * (dynamic_distance_m_ - d) / span;
    }
    return true;
  }

  bool raycastExpectedRange(const Pose2D& pose, const cv::Point2f& laser_origin_base,
                            const cv::Point2f& endpoint_base, double max_range,
                            double* expected_range) const {
    const double bx = endpoint_base.x - laser_origin_base.x;
    const double by = endpoint_base.y - laser_origin_base.y;
    const double norm = std::hypot(bx, by);
    if (norm < 1e-6) return false;

    const double origin_x = pose.x + std::cos(pose.yaw) * laser_origin_base.x -
                            std::sin(pose.yaw) * laser_origin_base.y;
    const double origin_y = pose.y + std::sin(pose.yaw) * laser_origin_base.x +
                            std::cos(pose.yaw) * laser_origin_base.y;
    const double dir_x = (std::cos(pose.yaw) * bx - std::sin(pose.yaw) * by) / norm;
    const double dir_y = (std::sin(pose.yaw) * bx + std::cos(pose.yaw) * by) / norm;
    const double step = std::max(raycast_step_m_, 0.5 * map_.info.resolution);

    for (double range = step; range <= max_range; range += step) {
      const double x = origin_x + range * dir_x;
      const double y = origin_y + range * dir_y;
      const int gx = static_cast<int>(std::floor((x - map_.info.origin.position.x) / map_.info.resolution));
      const int gy = static_cast<int>(std::floor((y - map_.info.origin.position.y) / map_.info.resolution));
      if (gx < 0 || gy < 0 || gx >= static_cast<int>(map_.info.width) ||
          gy >= static_cast<int>(map_.info.height)) return false;
      if (map_.data[gy * map_.info.width + gx] >= 65) {
        *expected_range = range;
        return true;
      }
    }
    return false;
  }

  double rayConsistencyWeight(const Pose2D& pose, const cv::Point2f& laser_origin_base,
                              const cv::Point2f& endpoint_base, double measured_range,
                              double max_range) const {
    double expected_range = 0.0;
    if (!raycastExpectedRange(pose, laser_origin_base, endpoint_base, max_range,
                              &expected_range)) {
      // A finite hit where the static map predicts no wall is unsuitable for
      // localization (person, temporary object, or unmapped structure).
      return 0.0;
    }
    if (measured_range < expected_range - raycast_foreground_margin_m_) {
      // The return arrived before the mapped wall: foreground occlusion.
      return 0.0;
    }
    if (std::abs(measured_range - expected_range) <= raycast_wall_tolerance_m_)
      return 1.0;
    return uncertain_weight_;
  }

  Quality scorePose(const std::vector<ScanPoint>& points, const Pose2D& pose) const {
    Quality quality;
    for (const auto& point : points) {
      const double mx = pose.x + std::cos(pose.yaw) * point.base.x - std::sin(pose.yaw) * point.base.y;
      const double my = pose.y + std::sin(pose.yaw) * point.base.x + std::cos(pose.yaw) * point.base.y;
      double weight = 0.0;
      bool is_static = false;
      if (!distanceAndWeight(mx, my, &weight, &is_static)) continue;
      ++quality.valid;
      if (is_static && point.ray_weight >= 0.5) ++quality.static_inliers;
      // A likelihood-field score, with map-inconsistent endpoints receiving
      // zero (or a small transition weight) instead of pulling the pose.
      quality.score += weight * point.ray_weight;
    }
    return quality;
  }

  Pose2D refinePose(const std::vector<ScanPoint>& points, Quality* best_quality) const {
    Pose2D center = pose_;
    const double xy_steps[] = {0.05, 0.02, 0.005};
    const double yaw_steps[] = {2.0 * M_PI / 180.0, 0.75 * M_PI / 180.0, 0.25 * M_PI / 180.0};
    for (int level = 0; level < 3; ++level) {
      Quality level_best;
      double best_score = -1.0;
      Pose2D candidate_best = center;
      for (int dx = -2; dx <= 2; ++dx) {
        for (int dy = -2; dy <= 2; ++dy) {
          for (int dyaw = -2; dyaw <= 2; ++dyaw) {
            Pose2D candidate = center;
            candidate.x += dx * xy_steps[level];
            candidate.y += dy * xy_steps[level];
            candidate.yaw = normalizeAngle(candidate.yaw + dyaw * yaw_steps[level]);
            const Quality quality = scorePose(points, candidate);
            if (quality.score > best_score) {
              best_score = quality.score;
              candidate_best = candidate;
              level_best = quality;
            }
          }
        }
      }
      center = candidate_best;
      *best_quality = level_best;
    }
    return center;
  }

  void publishDebugScans(const sensor_msgs::LaserScan& input, const std::vector<ScanPoint>& points) {
    sensor_msgs::LaserScan statics = input;
    sensor_msgs::LaserScan dynamics = input;
    statics.ranges.assign(input.ranges.size(), std::numeric_limits<float>::infinity());
    dynamics.ranges.assign(input.ranges.size(), std::numeric_limits<float>::infinity());
    size_t point_index = 0;
    for (size_t i = 0; i < input.ranges.size() && point_index < points.size(); ++i) {
      const float r = input.ranges[i];
      if (!(r >= input.range_min && r <= input.range_max)) continue;
      const ScanPoint& point = points[point_index++];
      const double mx = pose_.x + std::cos(pose_.yaw) * point.base.x - std::sin(pose_.yaw) * point.base.y;
      const double my = pose_.y + std::sin(pose_.yaw) * point.base.x + std::cos(pose_.yaw) * point.base.y;
      double weight = 0.0;
      bool is_static = false;
      if (distanceAndWeight(mx, my, &weight, &is_static) && is_static && point.ray_weight >= 0.5)
        statics.ranges[i] = r;
      else dynamics.ranges[i] = r;
    }
    static_scan_pub_.publish(statics);
    dynamic_scan_pub_.publish(dynamics);
  }

  void publishTransform(const ros::Time& stamp) {
    try {
      const auto odom_to_base = tf_buffer_.lookupTransform(odom_frame_, base_frame_, stamp, ros::Duration(0.03));
      tf2::Transform map_to_base;
      map_to_base.setOrigin(tf2::Vector3(pose_.x, pose_.y, 0.0));
      tf2::Quaternion q; q.setRPY(0.0, 0.0, pose_.yaw); map_to_base.setRotation(q);
      tf2::Transform odom_to_base_tf;
      tf2::fromMsg(odom_to_base.transform, odom_to_base_tf);
      geometry_msgs::TransformStamped output;
      output.header.stamp = stamp;
      output.header.frame_id = "map";
      output.child_frame_id = odom_frame_;
      output.transform = tf2::toMsg(map_to_base * odom_to_base_tf.inverse());
      tf_broadcaster_.sendTransform(output);
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(1.0, "dynamic_lidar_localizer: odom TF unavailable: %s", ex.what());
    }
  }

  void publishQuality(const Quality& quality, bool accepted) {
    const double ratio = quality.valid == 0 ? 0.0 : static_cast<double>(quality.static_inliers) / quality.valid;
    std_msgs::String text;
    std::ostringstream stream;
    stream << "accepted=" << (accepted ? "true" : "false")
           << ";valid_points=" << quality.valid
           << ";static_inlier_ratio=" << ratio
           << ";score_per_point=" << (quality.valid == 0 ? 0.0 : quality.score / quality.valid);
    text.data = stream.str();
    quality_pub_.publish(text);
    std_msgs::Bool status; status.data = accepted; status_pub_.publish(status);
  }

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    if (!map_ready_ || !pose_initialized_) return;
    geometry_msgs::TransformStamped laser_to_base;
    try {
      laser_to_base = tf_buffer_.lookupTransform(base_frame_, laser_frame_, scan->header.stamp, ros::Duration(0.03));
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(1.0, "dynamic_lidar_localizer: laser TF unavailable: %s", ex.what());
      return;
    }
    geometry_msgs::PointStamped laser_origin, base_origin;
    laser_origin.header = scan->header;
    laser_origin.point.x = 0.0;
    laser_origin.point.y = 0.0;
    laser_origin.point.z = 0.0;
    tf2::doTransform(laser_origin, base_origin, laser_to_base);
    const cv::Point2f laser_origin_base(base_origin.point.x, base_origin.point.y);

    std::vector<ScanPoint> points;
    points.reserve(scan->ranges.size());
    double angle = scan->angle_min;
    for (float range : scan->ranges) {
      if (range >= scan->range_min && range <= scan->range_max) {
        geometry_msgs::PointStamped in, out;
        in.header = scan->header;
        in.point.x = range * std::cos(angle);
        in.point.y = (invert_laser_y_ ? -1.0 : 1.0) * range * std::sin(angle);
        tf2::doTransform(in, out, laser_to_base);
        ScanPoint point;
        point.base = cv::Point2f(out.point.x, out.point.y);
        point.ray_weight = rayConsistencyWeight(
            pose_, laser_origin_base, point.base, range, scan->range_max);
        points.push_back(point);
      }
      angle += scan->angle_increment;
    }
    Quality quality;
    const Pose2D candidate = refinePose(points, &quality);
    const double ratio = quality.valid == 0 ? 0.0 : static_cast<double>(quality.static_inliers) / quality.valid;
    const bool accepted = quality.valid >= min_valid_points_ && ratio >= min_static_inlier_ratio_ &&
        quality.score / std::max(1, quality.valid) >= min_score_per_point_;
    publishQuality(quality, accepted);
    if (!accepted) {
      ROS_WARN_THROTTLE(1.0, "dynamic_lidar_localizer: rejected scan (valid=%d ratio=%.2f)", quality.valid, ratio);
      // Keep map->odom fresh using the last trusted map pose.  Odom still
      // moves base_frame during a short dynamic-obstacle occlusion.
      publishDebugScans(*scan, points);
      publishTransform(scan->header.stamp);
      return;
    }
    pose_ = candidate;
    publishDebugScans(*scan, points);
    publishTransform(scan->header.stamp);
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber map_sub_, scan_sub_, initial_pose_sub_;
  ros::Publisher quality_pub_, status_pub_, static_scan_pub_, dynamic_scan_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  nav_msgs::OccupancyGrid map_;
  std::vector<float> distance_field_m_;
  Pose2D pose_;
  std::string base_frame_, odom_frame_, laser_frame_, laser_topic_;
  double static_distance_m_, dynamic_distance_m_, uncertain_weight_;
  double min_static_inlier_ratio_, min_score_per_point_;
  double raycast_foreground_margin_m_, raycast_wall_tolerance_m_, raycast_step_m_;
  int min_valid_points_;
  bool invert_laser_y_ = false, map_ready_ = false, pose_initialized_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "dynamic_lidar_localizer");
  DynamicLidarLocalizer node;
  ros::spin();
  return 0;
}
