#include "mini_slam/scan_processor.hpp"

#include <cmath>

namespace mini_slam
{

std::vector<Point2D> ScanProcessor::convertScanToLocalPoints(
  const sensor_msgs::msg::LaserScan & scan) const
{
  std::vector<Point2D> points;
  points.reserve(scan.ranges.size());

  double angle = scan.angle_min;
  for (const float range : scan.ranges) {
    const bool valid_range =
      std::isfinite(range) &&
      range >= scan.range_min &&
      range <= scan.range_max;

    if (valid_range) {
      points.push_back(Point2D{
        static_cast<double>(range) * cos(angle),
        static_cast<double>(range) * sin(angle)});
    }

    angle += scan.angle_increment;
  }

  return points;
}

Pose2D ScanProcessor::poseFromOdometry(const nav_msgs::msg::Odometry & odom) const
{
  const auto & position = odom.pose.pose.position;
  const auto & q = odom.pose.pose.orientation;

  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);

  return Pose2D{
    position.x,
    position.y,
    atan2(siny_cosp, cosy_cosp)};
}

LocalizedScan ScanProcessor::makeLocalizedScan(
  const sensor_msgs::msg::LaserScan & scan,
  const Pose2D & odom_pose,
  const builtin_interfaces::msg::Time & odom_stamp) const
{
  LocalizedScan localized_scan;
  localized_scan.local_points = convertScanToLocalPoints(scan);
  localized_scan.odom_pose = odom_pose;
  localized_scan.estimated_pose = odom_pose;
  localized_scan.stamp = scan.header.stamp;
  localized_scan.odom_stamp = odom_stamp;
  localized_scan.frame_id = scan.header.frame_id;
  return localized_scan;
}

}  // namespace mini_slam
