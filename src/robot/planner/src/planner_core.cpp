#include "planner_core.hpp"

namespace robot {

PlannerCore::PlannerCore(const rclcpp::Logger& logger)
  : logger_(logger),
    map_(std::make_shared<nav_msgs::msg::OccupancyGrid>()),
    path_(std::make_shared<nav_msgs::msg::Path>()) {}

void PlannerCore::initializePlanner(double smoothing_factor, int iterations) {
  smoothing_factor_ = smoothing_factor;
  iterations_ = iterations;
}

bool PlannerCore::planPath(double start_x, double start_y,
                           double goal_x, double goal_y,
                           nav_msgs::msg::OccupancyGrid::SharedPtr map) {
  map_ = map;
  path_->header.frame_id = map_->header.frame_id;
  path_->header.stamp = rclcpp::Clock().now();
  path_->poses.clear();

  CellIndex start_idx, goal_idx;
  if (!poseToMap(start_x, start_y, start_idx) ||
      !poseToMap(goal_x, goal_y, goal_idx)) {
    RCLCPP_WARN(logger_, "Start or goal out of bounds");
    return false;
  }

  std::vector<CellIndex> path_cells;
  if (!doAStarSearch(start_idx, goal_idx, path_cells)) {
    RCLCPP_WARN(logger_, "A* failed to find path");
    return false;
  }

  for (auto& cell : path_cells) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = map_->header;
    double wx, wy;
    mapToPose(cell, wx, wy);
    ps.pose.position.x = wx;
    ps.pose.position.y = wy;
    ps.pose.orientation.w = 1.0;
    path_->poses.push_back(ps);
  }
  return true;
}

bool PlannerCore::doAStarSearch(const CellIndex& start_idx,
                                const CellIndex& goal_idx,
                                std::vector<CellIndex>& out_path) {
  int width = map_->info.width;
  int height = map_->info.height;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_map<CellIndex, double, CellIndexHash> g_cost;
  std::unordered_map<CellIndex, double, CellIndexHash> f_cost;

  auto getScore = [&](auto& m, const CellIndex& idx) {
    auto it = m.find(idx);
    return (it != m.end()) ? it->second : std::numeric_limits<double>::infinity();
  };
  auto setScore = [&](auto& m, const CellIndex& idx, double val) { m[idx] = val; };

  auto cellCost = [&](const CellIndex& idx) {
    int x = idx.x, y = idx.y;
    if (x < 0 || x >= width || y < 0 || y >= height) return 127;
    int8_t v = map_->data[y * width + x];
    return v < 0 ? 100 : static_cast<int>(v);
  };

  setScore(g_cost, start_idx, 0.0);
  setScore(f_cost, start_idx, euclidianHeuristic(start_idx, goal_idx));
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;
  open.push(AStarNode(start_idx, getScore(f_cost, start_idx)));

  while (!open.empty()) {
    CellIndex current = open.top().index;
    open.pop();
    if (current == goal_idx) {
      reconstructPath(came_from, current, out_path);
      return true;
    }
    double g_cur = getScore(g_cost, current);
    for (auto& nb : getNeighbors8(current)) {
      if (!inGridBounds(nb.x, nb.y)) continue;
      int cost = cellCost(nb);
      if (cost >= 100) continue;
      double tentative = g_cur + stepDistance(current, nb) + cost / 25.0;
      if (tentative < getScore(g_cost, nb)) {
        setScore(g_cost, nb, tentative);
        double f = tentative + euclidianHeuristic(nb, goal_idx);
        setScore(f_cost, nb, f);
        came_from[nb] = current;
        open.push(AStarNode(nb, f));
      }
    }
  }
  return false;
}

void PlannerCore::reconstructPath(
    const std::unordered_map<CellIndex, CellIndex, CellIndexHash>& came_from,
    const CellIndex& current,
    std::vector<CellIndex>& out_path) {
  out_path.clear();
  for (CellIndex c = current; came_from.count(c); c = came_from.at(c))
    out_path.push_back(c);
  std::reverse(out_path.begin(), out_path.end());
}

std::vector<CellIndex> PlannerCore::getNeighbors8(const CellIndex& c) {
  std::vector<CellIndex> v;
  for (int dx = -1; dx <= 1; ++dx)
    for (int dy = -1; dy <= 1; ++dy)
      if (dx || dy) v.emplace_back(c.x + dx, c.y + dy);
  return v;
}

double PlannerCore::euclidianHeuristic(const CellIndex& a,
                                       const CellIndex& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}

double PlannerCore::stepDistance(const CellIndex& a,
                                 const CellIndex& b) {
  int dx = std::abs(a.x - b.x), dy = std::abs(a.y - b.y);
  return (dx && dy) ? std::sqrt(2.0) : 1.0;
}

bool PlannerCore::poseToMap(double world_x, double world_y,
                            CellIndex& out_idx) {
  double ox = map_->info.origin.position.x;
  double oy = map_->info.origin.position.y;
  double res = map_->info.resolution;
  int ix = static_cast<int>(std::floor((world_x - ox) / res));
  int iy = static_cast<int>(std::floor((world_y - oy) / res));
  if (!inGridBounds(ix, iy)) return false;
  out_idx.x = ix;
  out_idx.y = iy;
  return true;
}

bool PlannerCore::inGridBounds(int idx_x, int idx_y) {
  return idx_x >= 0 && idx_x < static_cast<int>(map_->info.width) &&
         idx_y >= 0 && idx_y < static_cast<int>(map_->info.height);
}

void PlannerCore::mapToPose(const CellIndex& idx,
                            double& world_x, double& world_y) {
  double ox = map_->info.origin.position.x;
  double oy = map_->info.origin.position.y;
  double res = map_->info.resolution;
  world_x = ox + (idx.x + 0.5) * res;
  world_y = oy + (idx.y + 0.5) * res;
}

bool PlannerCore::lineOfSight(const CellIndex& start,
                              const CellIndex& end) { return true; }

bool PlannerCore::isCellFree(int x, int y) { return true; }

nav_msgs::msg::Path::SharedPtr PlannerCore::getPath() const { return path_; }

}  
