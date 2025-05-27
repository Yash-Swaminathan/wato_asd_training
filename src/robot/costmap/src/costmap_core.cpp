#include "costmap_core.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

CostmapCore::CostmapCore(){}

void CostmapCore::initializeCostmap() {
    costmap_.clear();
    costmap_.resize(costmap_height_, std::vector<int>(costmap_width_, 0));
}
void CostmapCore::transformToChassisFrame(double &x, double &y) {

    double robot_x = 0.0; 
    double robot_y = 0.0; 
    double robot_theta = 0.0; 


    x -= robot_x;
    y -= robot_y;

    double cos_theta = cos(robot_theta);
    double sin_theta = sin(robot_theta);

    double x_new = x * cos_theta + y * sin_theta;
    double y_new = -x * sin_theta + y * cos_theta;

    x = x_new;
    y = y_new;
}

// convert to grid coordinates, takes in the range and angle and returns x and y coordinates in the cartesian plane
void CostmapCore::convertToGrid(double range, double angle, int &x_grid, int &y_grid) {
    
    double x = range * cos(angle);
    double y = range * sin(angle);

    transformToChassisFrame(x, y);
    //subtract the origin; allows x and y polar coordinates to now be relative to the grid
    x_grid = (x - origin_x_ )/ resolution_;
    y_grid = (y - origin_y_ )/ resolution_;
}

void CostmapCore::markObstacle(int x_grid, int y_grid) {
    //check if the cell is empty or not in both the x and y direction
    //if not empty, mark as occupied
    if (x_grid >= 0 && x_grid < costmap_width_ && y_grid >= 0 && y_grid < costmap_height_) {
        costmap_[y_grid][x_grid] = 100;  // mark as occupied w value 100 in the costmap 2d array
    }
}

void CostmapCore::inflateObstacles() {
    int max_cost = 100;     
    // loop each cell
    for (int y = 0; y < costmap_height_; ++y) {
        for (int x = 0; x < costmap_width_; ++x) {
            if (costmap_[y][x] == max_cost) {
                for (int dy = -inflation_radius_; dy <= inflation_radius_; ++dy) {
                    for (int dx = -inflation_radius_; dx <= inflation_radius_; ++dx) {
                        // euclidean distance
                        double dist = std::sqrt(dx * dx + dy * dy);

                        if (dist <= inflation_radius_) {
                            int inflate_x = x + dx;
                            int inflate_y = y + dy;

                            // ensure the cell is within bounds
                            if (inflate_x >= 0 && inflate_x < costmap_width_ && inflate_y >= 0 && inflate_y < costmap_height_) {
                                // Calculate inflated cost
                                int inflated_cost = max_cost * (1.0 - (dist / inflation_radius_));

                                // Update the costmap with the inflated cost, clamping to the maximum value
                                costmap_[inflate_y][inflate_x] = std::max(costmap_[inflate_y][inflate_x], inflated_cost);
                            }
                        }
                    }
                }
            }
        }
    }
}


void CostmapCore::publishCostmap(
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_,
    const sensor_msgs::msg::LaserScan::SharedPtr& laser_scan_msg) {

    auto occupancy_grid_msg = nav_msgs::msg::OccupancyGrid();

    // Set header information
    occupancy_grid_msg.header.stamp = rclcpp::Clock().now();
    occupancy_grid_msg.header = laser_scan_msg->header; 

    occupancy_grid_msg.info.resolution = resolution_;
    occupancy_grid_msg.info.width = costmap_width_;
    occupancy_grid_msg.info.height = costmap_height_;

    occupancy_grid_msg.info.origin.position.x = origin_x_;
    occupancy_grid_msg.info.origin.position.y = origin_y_;

    occupancy_grid_msg.info.origin.orientation.x = 0.0;
    occupancy_grid_msg.info.origin.orientation.y = 0.0;
    occupancy_grid_msg.info.origin.orientation.z = 0.0;
    occupancy_grid_msg.info.origin.orientation.w = 1.0;

    occupancy_grid_msg.data.resize(costmap_width_ * costmap_height_);
    
    // Flatten the array to fill the costmap data
    for (int y = 0; y < costmap_height_; ++y) {
        for (int x = 0; x < costmap_width_; ++x) {
            occupancy_grid_msg.data[y * costmap_width_ + x] = costmap_[y][x];
        }
    }
    costmap_pub_->publish(occupancy_grid_msg);
}

