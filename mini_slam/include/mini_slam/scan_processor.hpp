#pragma once

#include <vector>

#include "mini_slam/types.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace mini_slam
{

class ScanProcessor
{
public:
  std::vector<Point2D> convertScanToLocalPoints(
    const sensor_msgs::msg::LaserScan & scan) const;

  Pose2D poseFromOdometry(const nav_msgs::msg::Odometry & odom) const;

  LocalizedScan makeLocalizedScan(
    const sensor_msgs::msg::LaserScan & scan,
    const Pose2D & odom_pose,
    const builtin_interfaces::msg::Time & odom_stamp) const;
};

}  // namespace mini_slam
