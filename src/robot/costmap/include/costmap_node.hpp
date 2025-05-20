#ifndef COSTMAP_NODE_HPP
#define COSTMAP_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
public:
    CostmapNode();

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

    robot::CostmapCore costmap_core_;
};

#endif 
