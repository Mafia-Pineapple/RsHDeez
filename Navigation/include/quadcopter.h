#pragma once
#ifndef QUADCOPTER_H
#define QUADCOPTER_H

#include "controller.h"

// ROS msgs
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"

// UAV drone platform controller
class Quadcopter : public Controller
{
public:
  Quadcopter();
  ~Quadcopter();

  // Main control loop tick (invoked by timer)
  bool reachGoal(void) override;

  // Planner-style query
  bool checkOriginToDestination(geometry_msgs::msg::Pose origin,
                                geometry_msgs::msg::Point goal,
                                double& distance, double& time,
                                geometry_msgs::msg::Pose& estimatedGoalPose) override;

  // Service callback to arm/start/stop reaching goal
  void control(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
               std::shared_ptr<std_srvs::srv::SetBool::Response> res) override;

private:
  // Recompute target geometry and timing
  GoalStats calcNewGoal(void) override;

  // Low-level command helpers
  void sendCmd(double turn_l_r, double move_l_r, double move_u_d, double move_f_b);
  void sendTakeOff(void);
  void sendLanding(void);

  // ---- State ----
  double target_angle_ = 0.0;      // desired global heading to goal
  bool liftoff_ = false;
  bool landed_  = true;

  // Gains/limits
  const double TARGET_SPEED;              // m/s on XY
  const double TARGET_HEIGHT_TOLERANCE;   // m for Z control

  // IO
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pubCmdVel_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr      pubTakeOff_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr      pubLanding_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr      srvReachGoal_;
  rclcpp::TimerBase::SharedPtr                            timer_;
};

#endif // QUADCOPTER_H
