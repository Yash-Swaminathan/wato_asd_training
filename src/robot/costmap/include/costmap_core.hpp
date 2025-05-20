#ifndef COSTMAP_CORE_HPP
#define COSTMAP_CORE_HPP

#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace robot {

class CostmapCore {
public:
    explicit CostmapCore(const rclcpp::Logger& logger, rclcpp::Node* node);

    void initialize();

    void processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr& scan);

    nav_msgs::msg::OccupancyGrid getOccupancyGrid(
      const rclcpp::Time& timestamp,
      const std::string& frame_id) const;

private:
    void transformToChassisFrame(double& x, double& y);

    void convertToGrid(double range, double angle, int& x_grid, int& y_grid);

    void markObstacle(int x_grid, int y_grid);

    void inflateObstacles();

    void publishCostmap(const rclcpp::Time& timestamp, const std::string& frame_id);

private:
    rclcpp::Logger logger_;
    rclcpp::Node* node_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

    nav_msgs::msg::OccupancyGrid grid_;

    std::vector<std::vector<int>> costmap_;

    double resolution_{0.15};
    int width_{300};
    int height_{300};
    double origin_x_{-25.0};
    double origin_y_{-25.0};

    double inflation_radius_{8.5};
    int max_cost_{100};
};

} 

#endif 
