#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "planner_core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <mutex>

class PlannerNode : public rclcpp::Node {
public:
  PlannerNode();

private:
  void getParameters();
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg);
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
  void timerCallback();
  void publishPath();
  void resetGoal();

  robot::PlannerCore planner_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  std::mutex map_mutex_;
  geometry_msgs::msg::PoseStamped current_goal_;
  bool active_goal_{false};
  rclcpp::Time plan_start_time_;
  bool have_odom_{false};
  double odom_x_{0.0}, odom_y_{0.0};
  double smoothing_factor_{0.0};
  int iterations_{0};
  double goal_tolerance_{0.0}, plan_timeout_{0.0};
};

#endif  
