#pragma once
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_srvs/srv/set_bool.hpp" // <-- add

#include "controllerinterface.h"

// Goal bundle (as in your code)
struct GoalStats {
  geometry_msgs::msg::Point location;
  double distance{0.0};
  double time{0.0};
};

class Controller : public ControllerInterface, public rclcpp::Node
{
public:
  Controller();

  // set goal (topic or clicked point)
  void setGoal(const geometry_msgs::msg::Point& msg);
  void setGoalClicked(const geometry_msgs::msg::PointStamped& msg);

  // planner query
  virtual bool checkOriginToDestination(geometry_msgs::msg::Pose origin,
                                        geometry_msgs::msg::Point goal,
                                        double& distance, double& time,
                                        geometry_msgs::msg::Pose& estimatedGoalPose) = 0;

  // tolerances/metrics
  bool   setTolerance(double tolerance);
  double distanceTravelled(void);
  double timeInMotion(void);
  double distanceToGoal(void);
  double timeToGoal(void);

  // odo
  geometry_msgs::msg::Pose getOdometry(void);
  void odoCallback(const nav_msgs::msg::Odometry& msg);

  // *** NEW: service callback contract (your TODO) ***
  virtual void control(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                       std::shared_ptr<std_srvs::srv::SetBool::Response> res) = 0;

protected:
  // impl-specific goal recalc
  virtual GoalStats calcNewGoal() = 0;

  bool goalReached();
  GoalStats getGoalStats(void);

protected:
  bool   goalSet_{false};
  double tolerance_{0.5};
  pfms::PlatformStatus status_{pfms::PlatformStatus::IDLE};

private:
  geometry_msgs::msg::Pose pose_;
  std::mutex poseMtx_;

  GoalStats goal_;
  std::mutex goalMtx_;

  double distance_travelled_{0.0};
  double time_travelled_{0.0};
  long unsigned int seq_{0};

  // Subscriptions (match your topics)  :contentReference[oaicite:17]{index=17}
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       sub1_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr     sub2_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub3_;
};

#endif // CONTROLLER_H
