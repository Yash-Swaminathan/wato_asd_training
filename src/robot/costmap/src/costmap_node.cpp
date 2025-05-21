#include <chrono>
#include <memory>
#include "costmap_node.hpp"

CostmapNode::CostmapNode()
  : Node("costmap_node"),
    costmap_(this->get_logger(), this)
{
  // Sub to LiDAR scans
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar",
    10,
    std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1)
  );

  // Advertise occupancy grid
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/costmap",
    10
  );

  RCLCPP_INFO(this->get_logger(), "CostmapNode initialized and subscribing to /lidar");
}

void CostmapNode::laserCallback(
  const sensor_msgs::msg::LaserScan::SharedPtr& scan
) {
  // Reset the costmap
  costmap_.resetMap();

  // Mark obstacles from laser ranges
  double angle = scan->angle_min;
  for (auto range : scan->ranges) {
    if (range >= scan->range_min && range <= scan->range_max) {
      int x, y;
      costmap_.convertToGrid(range, angle, x, y);
      costmap_.markObstacle(x, y);
    }
    angle += scan->angle_increment;
  }

  // Inflate and publish
  costmap_.inflateObstacles();
  auto grid_msg = costmap_.getOccupancyGrid(this->now(), scan->header.frame_id);
  costmap_pub_->publish(grid_msg);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
