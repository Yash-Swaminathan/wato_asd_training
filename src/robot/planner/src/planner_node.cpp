#include "planner_node.hpp"

PlannerNode::PlannerNode()
  : Node("planner"), planner_(this->get_logger()) {
  getParameters();

  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, 10,
      std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, 10,
      std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 10,
      std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 10);

  timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&PlannerNode::timerCallback, this));

  planner_.initializePlanner(smoothing_factor_, iterations_);
}

void PlannerNode::getParameters() {
  declare_parameter<std::string>("map_topic", "/map");
  declare_parameter<std::string>("goal_topic", "/goal_point");
  declare_parameter<std::string>("odom_topic", "/odom/filtered");
  declare_parameter<std::string>("path_topic", "/path");
  declare_parameter<double>("smoothing_factor", 0.2);
  declare_parameter<int>("iterations", 20);
  declare_parameter<double>("goal_tolerance", 0.3);
  declare_parameter<double>("plan_timeout_seconds", 10.0);

  map_topic_ = get_parameter("map_topic").as_string();
  goal_topic_ = get_parameter("goal_topic").as_string();
  odom_topic_ = get_parameter("odom_topic").as_string();
  path_topic_ = get_parameter("path_topic").as_string();
  smoothing_factor_ = get_parameter("smoothing_factor").as_double();
  iterations_ = get_parameter("iterations").as_int();
  goal_tolerance_ = get_parameter("goal_tolerance").as_double();
  plan_timeout_ = get_parameter("plan_timeout_seconds").as_double();
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(map_mutex_);
  map_ = msg;
  if (active_goal_) publishPath();
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (!map_) return;
  current_goal_ = *msg;
  active_goal_ = true;
  plan_start_time_ = now();
  publishPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  odom_x_ = msg->pose.pose.position.x;
  odom_y_ = msg->pose.pose.position.y;
  have_odom_ = true;
}

void PlannerNode::timerCallback() {
  if (!active_goal_) return;
  double elapsed = (now() - plan_start_time_).seconds();
  if (elapsed > plan_timeout_) return resetGoal();
  double dx = current_goal_.pose.position.x - odom_x_;
  double dy = current_goal_.pose.position.y - odom_y_;
  if (std::hypot(dx, dy) < goal_tolerance_) resetGoal();
}

void PlannerNode::publishPath() {
  if (!have_odom_ || !map_) return;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    if (!planner_.planPath(odom_x_, odom_y_,
                           current_goal_.pose.position.x,
                           current_goal_.pose.position.y,
                           map_))
      return resetGoal();
  }
  auto path_msg = *planner_.getPath();
  path_msg.header.stamp = now();
  path_pub_->publish(path_msg);
}

void PlannerNode::resetGoal() {
  active_goal_ = false;
  nav_msgs::msg::Path empty;
  empty.header.stamp = now();
  empty.header.frame_id = map_ ? map_->header.frame_id : "map";
  path_pub_->publish(empty);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
