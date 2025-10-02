#include "controller.h"
using std::placeholders::_1;

Controller::Controller()
: Node("controller")
{
  // Subscribe to odometry from the correct topic
  sub1_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry", 10, std::bind(&Controller::odoCallback, this, _1));

  // Subscribe to goal commands
  sub2_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/drone/goal", 10, std::bind(&Controller::setGoal, this, _1));

  // Subscribe to clicked points from RViz
  sub3_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", 10, std::bind(&Controller::setGoalClicked, this, _1));

  goal_.time = 0.0; 
  goal_.distance = 0.0;
  
  RCLCPP_INFO(this->get_logger(), "Controller node initialized");
  RCLCPP_INFO(this->get_logger(), "Subscribing to odometry on: /odometry");
}

void Controller::setGoal(const geometry_msgs::msg::Point& msg)
{
  std::lock_guard<std::mutex> lk(goalMtx_);
  goal_.location = msg;
  goalSet_ = true;
  RCLCPP_INFO(this->get_logger(), "New goal set: (%.2f, %.2f, %.2f)", 
              msg.x, msg.y, msg.z);
}

void Controller::setGoalClicked(const geometry_msgs::msg::PointStamped& msg)
{
  std::lock_guard<std::mutex> lk(goalMtx_);
  goal_.location.x = msg.point.x;
  goal_.location.y = msg.point.y;
  goal_.location.z = 2.0;  // Default flying height
  goalSet_ = true;
  RCLCPP_INFO(this->get_logger(), "Goal set from clicked point: (%.2f, %.2f, %.2f)", 
              goal_.location.x, goal_.location.y, goal_.location.z);
}

bool Controller::setTolerance(double t) 
{ 
  tolerance_ = t; 
  RCLCPP_INFO(this->get_logger(), "Tolerance set to: %.2f", t);
  return true; 
}

double Controller::distanceToGoal(void) { return goal_.distance; }
double Controller::timeToGoal(void)      { return goal_.time;      }
double Controller::distanceTravelled(void) { return distance_travelled_; }
double Controller::timeInMotion(void)      { return time_travelled_;    }

GoalStats Controller::getGoalStats(void)
{
  std::lock_guard<std::mutex> lk(goalMtx_);
  return goal_;
}

bool Controller::goalReached()
{
  if (!goalSet_) return false;

  auto g = getGoalStats();
  auto p = getOdometry();

  const double dx = g.location.x - p.position.x;
  const double dy = g.location.y - p.position.y;
  const double dz = g.location.z - p.position.z;
  const double d  = std::sqrt(dx*dx + dy*dy + dz*dz);

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                       "Distance to goal: %.3f m", d);
  return (d < tolerance_);
}

geometry_msgs::msg::Pose Controller::getOdometry(void)
{
  std::lock_guard<std::mutex> lk(poseMtx_);
  return pose_;
}

void Controller::odoCallback(const nav_msgs::msg::Odometry& msg)
{
  std::lock_guard<std::mutex> lk(poseMtx_);
  pose_ = msg.pose.pose;
  
  // Debug: Log position occasionally
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                       "Current position: (%.2f, %.2f, %.2f)", 
                       pose_.position.x, pose_.position.y, pose_.position.z);
}