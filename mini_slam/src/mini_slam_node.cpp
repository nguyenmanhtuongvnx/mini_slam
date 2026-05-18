#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mini_slam/scan_processor.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace mini_slam
{

class MiniSlamNode : public rclcpp::Node
{
public:
  MiniSlamNode()
  : Node("mini_slam_node")
  {
    max_scan_odom_time_delta_ = declare_parameter<double>(
      "max_scan_odom_time_delta", 0.1);
    max_odom_history_size_ = declare_parameter<int>(
      "max_odom_history_size", 200);

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan",
      rclcpp::SensorDataQoS(),
      std::bind(&MiniSlamNode::onScan, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      rclcpp::QoS(20),
      std::bind(&MiniSlamNode::onOdom, this, std::placeholders::_1));

    trajectory_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/mini_slam/trajectory",
      rclcpp::QoS(10));

    RCLCPP_INFO(
      get_logger(),
      "mini_slam_node started: subscribing /scan and /odom, publishing /mini_slam/trajectory");
    RCLCPP_INFO(
      get_logger(),
      "scan/odom timestamp sync enabled: max delta %.3f seconds, odom history %d messages",
      max_scan_odom_time_delta_,
      max_odom_history_size_);
  }

private:
  struct StampedOdomPose
  {
    Pose2D pose;
    builtin_interfaces::msg::Time stamp;
    std::string frame_id;
  };

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_history_.push_back(StampedOdomPose{
      scan_processor_.poseFromOdometry(*msg),
      msg->header.stamp,
      msg->header.frame_id.empty() ? "odom" : msg->header.frame_id});

    while (static_cast<int>(odom_history_.size()) > max_odom_history_size_) {
      odom_history_.pop_front();
    }
  }

  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const auto synced_odom = findNearestOdomPose(msg->header.stamp);
    if (!synced_odom) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Received /scan, but no time-synchronized /odom pose is available yet");
      return;
    }

    const double scan_odom_dt =
      std::abs(timeToSeconds(msg->header.stamp) - timeToSeconds(synced_odom->stamp));

    if (scan_odom_dt > max_scan_odom_time_delta_) {
      RCLCPP_WARN(
        get_logger(),
        "Dropping /scan: nearest /odom is %.3f seconds away, threshold is %.3f seconds",
        scan_odom_dt,
        max_scan_odom_time_delta_);
      return;
    }

    LocalizedScan localized_scan =
      scan_processor_.makeLocalizedScan(*msg, synced_odom->pose, synced_odom->stamp);

    localized_scans_.push_back(localized_scan);
    odom_frame_id_ = synced_odom->frame_id;
    appendTrajectoryPose(localized_scan);
    trajectory_pub_->publish(trajectory_);

    const Pose2D & pose = localized_scan.odom_pose;
    RCLCPP_INFO(
      get_logger(),
      "scan beams=%zu valid_points=%zu odom_pose=(x=%.3f, y=%.3f, theta=%.3f) scan_odom_dt=%.3f localized_scans=%zu",
      msg->ranges.size(),
      localized_scan.local_points.size(),
      pose.x,
      pose.y,
      pose.theta,
      scan_odom_dt,
      localized_scans_.size());
  }

  void appendTrajectoryPose(const LocalizedScan & localized_scan)
  {
    trajectory_.header.stamp = localized_scan.stamp;
    trajectory_.header.frame_id = odom_frame_id_;

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = trajectory_.header;
    pose_stamped.pose.position.x = localized_scan.estimated_pose.x;
    pose_stamped.pose.position.y = localized_scan.estimated_pose.y;
    pose_stamped.pose.position.z = 0.0;

    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, localized_scan.estimated_pose.theta);
    pose_stamped.pose.orientation = tf2::toMsg(orientation);

    trajectory_.poses.push_back(pose_stamped);
  }

  std::optional<StampedOdomPose> findNearestOdomPose(
    const builtin_interfaces::msg::Time & scan_stamp) const
  {
    if (odom_history_.empty()) {
      return std::nullopt;
    }

    const double scan_time = timeToSeconds(scan_stamp);
    const auto nearest = std::min_element(
      odom_history_.begin(),
      odom_history_.end(),
      [scan_time](const StampedOdomPose & lhs, const StampedOdomPose & rhs) {
        return std::abs(timeToSeconds(lhs.stamp) - scan_time) <
               std::abs(timeToSeconds(rhs.stamp) - scan_time);
      });

    return *nearest;
  }

  static double timeToSeconds(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<double>(stamp.sec) + 1e-9 * static_cast<double>(stamp.nanosec);
  }

  ScanProcessor scan_processor_;
  std::deque<StampedOdomPose> odom_history_;
  std::string odom_frame_id_{"odom"};
  std::vector<LocalizedScan> localized_scans_;
  nav_msgs::msg::Path trajectory_;
  double max_scan_odom_time_delta_{0.1};
  int max_odom_history_size_{200};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
};

}  // namespace mini_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mini_slam::MiniSlamNode>());
  rclcpp::shutdown();
  return 0;
}
