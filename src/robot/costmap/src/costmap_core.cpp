#include "costmap_core.hpp"

namespace robot {

CostmapCore::CostmapCore(const rclcpp::Logger& logger, rclcpp::Node* node)
  : logger_(logger)
{
  // Load parameters
  resolution_ = node->declare_parameter("resolution", 0.1);
  int width_m = node->declare_parameter("width", 10);
  int height_m = node->declare_parameter("height", 10);
  inflation_radius_ = node->declare_parameter("inflation_radius", 1.5);
  max_cost_ = node->declare_parameter("max_cost", 100);

  // Compute map dimensions
  width_   = static_cast<int>(width_m / resolution_);
  height_  = static_cast<int>(height_m / resolution_);
  origin_x_= -width_m / 2.0;
  origin_y_= -height_m / 2.0;

  // Initialize occupancy grid
  grid_.info.resolution = resolution_;
  grid_.info.width = width_;
  grid_.info.height = height_;
  grid_.info.origin.position.x = origin_x_;
  grid_.info.origin.position.y = origin_y_;
  grid_.info.origin.orientation.w = 1.0;
  grid_.data.assign(width_ * height_, 0);
}

void CostmapCore::resetMap()
{
  std::fill(grid_.data.begin(), grid_.data.end(), 0);
}

void CostmapCore::markCell(int x, int y)
{
  if (x < 0 || y < 0 || x >= width_ || y >= height_) return;
  grid_.data[y * width_ + x] = max_cost_;
}

void CostmapCore::processScan(const sensor_msgs::msg::LaserScan::SharedPtr& scan)
{
  resetMap();
  double angle = scan->angle_min;
  for (auto range : scan->ranges) {
    if (std::isfinite(range)) {
      double x = range * std::cos(angle);
      double y = range * std::sin(angle);
      int ix = static_cast<int>((x - origin_x_) / resolution_);
      int iy = static_cast<int>((y - origin_y_) / resolution_);
      markCell(ix, iy);
    }
    angle += scan->angle_increment;
  }
  inflateObstacles();
}

void CostmapCore::inflateObstacles()
{
  std::vector<int> temp = grid_.data;
  int rad_cells = static_cast<int>(inflation_radius_ / resolution_);

  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (grid_.data[y * width_ + x] == max_cost_) {
        for (int dy = -rad_cells; dy <= rad_cells; ++dy) {
          for (int dx = -rad_cells; dx <= rad_cells; ++dx) {
            int nx = x + dx;
            int ny = y + dy;
            if (nx < 0 || ny < 0 || nx >= width_ || ny >= height_) continue;
            double d = std::hypot(dx, dy) * resolution_;
            if (d <= inflation_radius_) {
              int cost = static_cast<int>(max_cost_ * (1.0 - d / inflation_radius_));
              int idx  = ny * width_ + nx;
              temp[idx] = std::max(temp[idx], cost);
            }
          }
        }
      }
    }
  }

  grid_.data.swap(temp);
}

nav_msgs::msg::OccupancyGrid CostmapCore::getGrid(rclcpp::Time stamp, const std::string& frame) const
{
  auto out = grid_;
  out.header.stamp    = stamp;
  out.header.frame_id = frame;
  return out;
}

} 
