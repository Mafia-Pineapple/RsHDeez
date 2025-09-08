#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <mutex>

struct GoalStats {
  geometry_msgs::msg::Point location;
  double distance;
  double time;
};

class Controller : public rclcpp::Node
{
public:
  Controller();
  virtual ~Controller() = default;

  // Goal management
  void setGoal(const geometry_msgs::msg::Point& msg);
  void setGoalClicked(const geometry_msgs::msg::PointStamped& msg);
  bool setTolerance(double t);
  
  // Status queries
  double distanceToGoal(void);
  double timeToGoal(void);
  double distanceTravelled(void);
  double timeInMotion(void);
  GoalStats getGoalStats(void);
  bool goalReached();
  
  // Odometry
  geometry_msgs::msg::Pose getOdometry(void);
  void odoCallback(const nav_msgs::msg::Odometry& msg);

protected:
  // Subscriptions
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub1_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub2_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub3_;

  // State variables
  geometry_msgs::msg::Pose pose_;
  GoalStats goal_;
  bool goalSet_ = false;
  double tolerance_ = 0.5;
  double distance_travelled_ = 0.0;
  double time_travelled_ = 0.0;

  // Thread safety
  std::mutex goalMtx_;
  std::mutex poseMtx_;
};

#endif // CONTROLLER_H