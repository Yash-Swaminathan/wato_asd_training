#ifndef COSTMAP_CORE_HPP
#define COSTMAP_CORE_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>

class CostmapCore {
public:
    CostmapCore();
    void initializeCostmap();
    void transformToChassisFrame(double &x, double &y);
    void convertToGrid(double range, double angle, int &x_grid, int &y_grid);
    void markObstacle(int x_grid, int y_grid);
    void inflateObstacles();
    void publishCostmap(rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_, const sensor_msgs::msg::LaserScan::SharedPtr& laser_scan_msg);

private:
    std::vector<std::vector<int>> costmap_; // 2D array of costmap
    int costmap_width_ = 300;                     
    int costmap_height_ = 300;                  
    int inflation_radius_ = 8.5;                    // radius of the inflation zone around obstacles
    double resolution_ = 0.15;                     
    double origin_x_ = -25.0;                     
    double origin_y_ = -25.0;                     
    
};
#endif