#include "quadcopter.h"
#include <cmath>
#include <chrono>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

Quadcopter::Quadcopter()
: liftoff_(false),
  TARGET_SPEED(1.0),
  TARGET_HEIGHT_TOLERANCE(0.2),
  goalSet_(false)
{
  tolerance_ = 0.5; // 0.5 m sphere tolerance

  // Publishers for drone control
  pubCmdVel_  = this->create_publisher<geometry_msgs::msg::Twist>("/model/drone/cmd_vel", 10);
  pubTakeOff_ = this->create_publisher<std_msgs::msg::Empty>("/drone/takeoff", 10);
  pubLanding_ = this->create_publisher<std_msgs::msg::Empty>("/drone/land", 10);

  // Subscriber for goal position (using different topic to avoid conflict)
  subGoal_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/drone/goal_stamped", 10,
      std::bind(&Quadcopter::goalCallback, this, std::placeholders::_1));

  // Service to start/stop motion control
  srvReachGoal_ = this->create_service<std_srvs::srv::SetBool>(
      "/reach_goal",
      std::bind(&Quadcopter::control, this, std::placeholders::_1, std::placeholders::_2));

  // 20 Hz control loop for smooth flight
  timer_ = this->create_wall_timer(50ms, std::bind(&Quadcopter::reachGoal, this));

  RCLCPP_INFO(this->get_logger(), "Quadcopter node initialized");
  RCLCPP_INFO(this->get_logger(), "Publish to '/drone/goal_stamped' to set target position");
  RCLCPP_INFO(this->get_logger(), "Call service '/reach_goal' with data=true to start flying to goal");
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

void Quadcopter::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  goalPosition_ = msg->point;
  goalSet_ = true;
  RCLCPP_INFO(this->get_logger(), "New goal received: (%.2f, %.2f, %.2f)", 
              goalPosition_.x, goalPosition_.y, goalPosition_.z);
}

bool Quadcopter::goalReached(void)
{
  if (!goalSet_) return false;
  
  auto pose = getOdometry();
  const double dx = goalPosition_.x - pose.position.x;
  const double dy = goalPosition_.y - pose.position.y;
  const double dz = goalPosition_.z - pose.position.z;
  const double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
  
  return distance < tolerance_;
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
  
  // Publish velocity command
  pubCmdVel_->publish(msg);
  
  // Debug output
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                       "Publishing cmd: linear.z=%.2f to /model/drone/cmd_vel", 
                       msg.linear.z);
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
    if (goalSet_) {
      status_ = pfms::PlatformStatus::RUNNING;
      res->success = true;
      res->message = "Flying to goal position";
      RCLCPP_INFO(this->get_logger(), "Control enabled - flying to goal");
    } else {
      res->success = false;
      res->message = "No goal set. Publish to /drone/goal first";
      RCLCPP_WARN(this->get_logger(), "No goal set - publish to /drone/goal first");
    }
  } else {
    status_ = pfms::PlatformStatus::LANDING;
    res->success = true;
    res->message = "Stopping and landing";
    RCLCPP_INFO(this->get_logger(), "Control disabled - landing");
  }
}

bool Quadcopter::reachGoal(void)
{
  // Debug: Always log what state we're in
  static int debug_counter = 0;
  debug_counter++;
  if (debug_counter % 20 == 0) { // Every second at 20Hz
    RCLCPP_INFO(this->get_logger(), "reachGoal() called - Status: %d, goalSet_: %s", 
                static_cast<int>(status_), goalSet_ ? "true" : "false");
  }

  switch (status_) {
    case pfms::PlatformStatus::IDLE:
      // Send zero velocities when idle
      sendCmd(0, 0, 0, 0);
      return false;

    case pfms::PlatformStatus::TAKEOFF:
      // Not used in this simplified version
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "In TAKEOFF state");
      break;

    case pfms::PlatformStatus::LANDING:
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "In LANDING state");
      sendLanding();
      sendCmd(0, 0, 0, 0); // Stop all motion
      status_ = pfms::PlatformStatus::IDLE;
      return true;

    case pfms::PlatformStatus::RUNNING:
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "In RUNNING state");
      
      if (!goalSet_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                             "No goal set - hovering in place");
        sendCmd(0, 0, 0, 0);
        return false;
      }

      // Check if goal is reached
      if (goalReached()) {
        sendCmd(0, 0, 0, 0); // Stop
        RCLCPP_INFO(this->get_logger(), "Goal reached! Hovering at target position");
        goalSet_ = false; // Reset goal so we can accept a new one
        status_ = pfms::PlatformStatus::IDLE;
        return true;
      }

      // Get current position
      auto pose = getOdometry();
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                           "Current pose: (%.2f, %.2f, %.2f)", 
                           pose.position.x, pose.position.y, pose.position.z);
      
      // Compute control commands using proportional control
      const double dx = goalPosition_.x - pose.position.x;
      const double dy = goalPosition_.y - pose.position.y;
      const double dz = goalPosition_.z - pose.position.z;
      
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                           "Goal: (%.2f, %.2f, %.2f), Error: (%.2f, %.2f, %.2f)", 
                           goalPosition_.x, goalPosition_.y, goalPosition_.z, dx, dy, dz);
      
      const double kp_xy = 0.8;  // Horizontal gain
      const double kp_z = 0.5;   // Vertical gain
      const double max_vel = 2.0; // Max velocity
      
      double vx = kp_xy * dx;  // forward/backward
      double vy = kp_xy * dy;  // left/right
      double vz = kp_z * dz;   // up/down
      
      // Clamp velocities to max
      vx = std::max(-max_vel, std::min(max_vel, vx));
      vy = std::max(-max_vel, std::min(max_vel, vy));
      vz = std::max(-max_vel, std::min(max_vel, vz));
      
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                           "Computed velocities: vx=%.2f, vy=%.2f, vz=%.2f", vx, vy, vz);
      
      // Send velocity command (yaw=0, left/right=vy, up/down=vz, forward/back=vx)
      sendCmd(0.0, vy, vz, vx);
      
      // Log progress occasionally
      double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                           "Flying to goal - distance remaining: %.2f m", distance);
      return true;
  }

  return false;
}