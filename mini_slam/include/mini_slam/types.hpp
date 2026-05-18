#pragma once

#include <string>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"

namespace mini_slam
{

struct Point2D
{
  double x{0.0};
  double y{0.0};
};

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

struct LocalizedScan
{
  std::vector<Point2D> local_points;
  Pose2D odom_pose;
  Pose2D estimated_pose;
  builtin_interfaces::msg::Time stamp;
  builtin_interfaces::msg::Time odom_stamp;
  std::string frame_id;
};
  
}  // namespace mini_slam
