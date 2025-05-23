#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include "map_memory_core.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <geometry_msgs/msg/pose.hpp>

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    robot::MapMemoryCore map_memory_;
    void LocalMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void Odometry(const nav_msgs::msg::Odometry::SharedPtr msg);
    void publishGlobalMap();
    double quaternionToYaw(double x, double y, double z, double w);

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_map_subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr global_map_publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    std::shared_ptr<robot::MapMemoryCore> map_memory_;
    double previous_position_x, previous_position_y;
    double robot_position_x, robot_position_y, robot_orientation_angle
    double movement_threshold;
};

#endif 
