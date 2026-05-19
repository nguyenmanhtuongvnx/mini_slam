#pragma once

#include <string>
#include <vector>
#include <optional>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "mini_slam/types.hpp"


namespace mini_slam
{
struct GridIndex
{
  int x{0};
  int y{0};
};

class OccupancyGridMap
{
public:
    OccupancyGridMap(double resolution, int width, int height, double origin_x, 
        double origin_y, const std::string & frame_id);
    
    void updateEndpointsOnly(const LocalizedScan & scan);
    void updateWithRaycasting(const LocalizedScan & scan);

    std::optional<GridIndex> worldToGrid(double x, double y) const;
    nav_msgs::msg::OccupancyGrid toRosMsg(const rclcpp::Time& stamp) const;

private:
    double resolution_;
    int width_;
    int height_;
    double origin_x_;
    double origin_y_;
    std::vector<int8_t> data_;
    std::string frame_id_;
};
}