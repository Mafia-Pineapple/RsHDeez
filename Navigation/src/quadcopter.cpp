#include "quadcopter.h"
#include <cmath>
#include <chrono>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

Quadcopter::Quadcopter()
: liftoff_(false),
  TARGET_SPEED(1.0),
  TARGET_HEIGHT_TOLERANCE(0.2)
{
  tolerance_ = 0.5; // 0.5 m sphere tolerance

  // Publishers for Ignition Gazebo drone control
  pubCmdVel_  = this->create_publisher<geometry_msgs::msg::Twist>("/model/drone/cmd_vel", 10);
  pubTakeOff_ = this->create_publisher<std_msgs::msg::Empty>("/drone/takeoff", 10);
  pubLanding_ = this->create_publisher<std_msgs::msg::Empty>("/drone/land", 10);

  // Service to start/stop motion control
  srvReachGoal_ = this->create_service<std_srvs::srv::SetBool>(
      "/reach_goal",
      std::bind(&Quadcopter::control, this, std::placeholders::_1, std::placeholders::_2));

  // 20 Hz control loop for smooth flight
  timer_ = this->create_wall_timer(50ms, std::bind(&Quadcopter::reachGoal, this));

  RCLCPP_INFO(this->get_logger(), "Quadcopter node initialized");
  RCLCPP_INFO(this->get_logger(), "Call service '/reach_goal' with data=true to start");
  RCLCPP_INFO(this->get_logger(), "Publish to '/drone/goal' to set target");
}

Quadcopter::~Quadcopter() = default;

bool Quadcopter::checkOriginToDestination(geometry_msgs::msg::Pose origin,
                                          geometry_msgs::msg::Point goal,
                                          double& distance, double& time,
                                          geometry_msgs::msg::Pose& estimatedGoalPose)
{
  const double dx = goal.x - origin.position.x;
  const double dy = goal.y - origin.position.y;
  const double dz = goal.z - origin.position.z;
  distance = std::sqrt(dx*dx + dy*dy + dz*dz);
  time = distance / TARGET_SPEED;

  estimatedGoalPose.position = goal;
  estimatedGoalPose.orientation = origin.orientation; // keep heading
  return true;
}

GoalStats Quadcopter::calcNewGoal(void)
{
  auto pose = getOdometry();
  auto goalStats = getGoalStats();

  geometry_msgs::msg::Pose est;
  checkOriginToDestination(pose, goalStats.location, goalStats.distance, goalStats.time, est);

  const double dx = goalStats.location.x - pose.position.x;
  const double dy = goalStats.location.y - pose.position.y;
  target_angle_ = std::atan2(dy, dx);

  return goalStats;
}

void Quadcopter::sendCmd(double yaw_rate, double move_l_r, double move_u_d, double move_f_b)
{
  geometry_msgs::msg::Twist msg;
  // For Ignition Gazebo: x=forward/back, y=left/right, z=up/down
  msg.linear.x  = move_f_b;   // forward/backward
  msg.linear.y  = move_l_r;   // left/right  
  msg.linear.z  = move_u_d;   // up/down
  msg.angular.z = yaw_rate;   // yaw rotation
  pubCmdVel_->publish(msg);
}

void Quadcopter::sendTakeOff(void)
{
  std_msgs::msg::Empty e;
  pubTakeOff_->publish(e);
  liftoff_ = true;  
  landed_ = false;
  RCLCPP_INFO(this->get_logger(), "Takeoff command sent");
}

void Quadcopter::sendLanding(void)
{
  std_msgs::msg::Empty e;
  pubLanding_->publish(e);
  liftoff_ = false; 
  landed_ = true;
  RCLCPP_INFO(this->get_logger(), "Landing command sent");
}

void Quadcopter::control(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                         std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  if (req->data) {
    status_ = pfms::PlatformStatus::TAKEOFF;
    res->success = true;
    res->message = "Starting autonomous flight control";
    RCLCPP_INFO(this->get_logger(), "Control enabled - starting takeoff sequence");
  } else {
    status_ = pfms::PlatformStatus::LANDING;
    res->success = true;
    res->message = "Stopping control and initiating landing";
    RCLCPP_INFO(this->get_logger(), "Control disabled - initiating landing");
  }
}

bool Quadcopter::reachGoal(void)
{
  switch (status_) {
    case pfms::PlatformStatus::IDLE:
      // Send zero velocities when idle
      sendCmd(0, 0, 0, 0);
      return false;

    case pfms::PlatformStatus::TAKEOFF:
      sendTakeOff();
      // Wait a bit for takeoff, then check if we have a goal
      static int takeoff_counter = 0;
      takeoff_counter++;
      if (takeoff_counter > 40) { // 2 seconds at 20Hz
        takeoff_counter = 0;
        status_ = goalSet_ ? pfms::PlatformStatus::RUNNING : pfms::PlatformStatus::IDLE;
        if (!goalSet_) {
          RCLCPP_WARN(this->get_logger(), "No goal set - remaining idle");
        }
      }
      return true;

    case pfms::PlatformStatus::LANDING:
      sendLanding();
      sendCmd(0, 0, 0, 0); // Stop all motion
      status_ = pfms::PlatformStatus::IDLE;
      return true;

    case pfms::PlatformStatus::RUNNING:
      break;
  }

  if (!goalSet_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                         "No goal set - hovering in place");
    sendCmd(0, 0, 0, 0);
    return false;
  }

  // Main control logic
  auto goalStats = calcNewGoal();
  auto pose = getOdometry();

  // Check if goal is reached
  if (goalReached()) {
    sendCmd(0, 0, 0, 0); // Stop
    RCLCPP_INFO(this->get_logger(), "Goal reached! Landing...");
    status_ = pfms::PlatformStatus::LANDING;
    return true;
  }

  // Compute control commands
  const double dx = goalStats.location.x - pose.position.x;
  const double dy = goalStats.location.y - pose.position.y;
  const double dz = goalStats.location.z - pose.position.z;

  // Simple proportional control
  const double kp_xy = 0.8;  // Horizontal gain
  const double kp_z = 0.5;   // Vertical gain
  const double max_vel = 2.0; // Max velocity

  double vx = kp_xy * dx;
  double vy = kp_xy * dy; 
  double vz = kp_z * dz;

  // Clamp velocities
  vx = std::max(-max_vel, std::min(max_vel, vx));
  vy = std::max(-max_vel, std::min(max_vel, vy));
  vz = std::max(-max_vel, std::min(max_vel, vz));

  // Send command (no yaw control for now)
  sendCmd(0.0, vy, vz, vx);

  return false;
}