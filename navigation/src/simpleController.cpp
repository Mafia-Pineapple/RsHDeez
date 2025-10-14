#include "quadcopter.h"
#include <cmath>
#include <chrono>
#include <random>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

Quadcopter::Quadcopter()
: liftoff_(false),
  TARGET_SPEED(1.0),
  TARGET_HEIGHT_TOLERANCE(0.2),
  goalSet_(false),
  wandering_(false),
  target_agl_(1.5)
{
  tolerance_ = 0.5; // 0.5 m sphere tolerance

  // Publishers for drone control
  pubCmdVel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  pubTakeOff_ = this->create_publisher<std_msgs::msg::Empty>("/drone/takeoff", 10);
  pubLanding_ = this->create_publisher<std_msgs::msg::Empty>("/drone/land", 10);

  // Subscriber for goal position
  subGoal_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/drone/goal_stamped", 10,
      std::bind(&Quadcopter::goalCallback, this, std::placeholders::_1));

  // Subscriber for AGL
  subAgl_ = this->create_subscription<std_msgs::msg::Float64>(
      "/drone/agl_distance", 10,
      std::bind(&Quadcopter::aglCallback, this, std::placeholders::_1));

  // Service to start/stop motion control
  srvReachGoal_ = this->create_service<std_srvs::srv::SetBool>(
      "/reach_goal",
      std::bind(&Quadcopter::control, this, std::placeholders::_1, std::placeholders::_2));

  // Service to enable/disable wandering mode
  srvWander_ = this->create_service<std_srvs::srv::SetBool>(
      "/wander_mode",
      std::bind(&Quadcopter::wanderControl, this, std::placeholders::_1, std::placeholders::_2));

  // 20 Hz control loop for smooth flight
  timer_ = this->create_wall_timer(50ms, std::bind(&Quadcopter::reachGoal, this));

  // Random number generator for wandering
  random_engine_.seed(std::chrono::system_clock::now().time_since_epoch().count());

  RCLCPP_INFO(this->get_logger(), "Quadcopter node initialized");
  RCLCPP_INFO(this->get_logger(), "Services:");
  RCLCPP_INFO(this->get_logger(), "  - /reach_goal (data=true: fly to goal)");
  RCLCPP_INFO(this->get_logger(), "  - /wander_mode (data=true: wander aimlessly)");
  RCLCPP_INFO(this->get_logger(), "Topics:");
  RCLCPP_INFO(this->get_logger(), "  - /drone/goal_stamped (set target position)");
  RCLCPP_INFO(this->get_logger(), "  - /drone/agl_distance (altitude above ground)");
}

Quadcopter::~Quadcopter() = default;

void Quadcopter::aglCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  current_agl_ = msg->data;
  agl_received_ = true;
}

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
  estimatedGoalPose.orientation = origin.orientation;
  return true;
}

void Quadcopter::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  goalPosition_ = msg->point;
  goalSet_ = true;
  wandering_ = false; // Stop wandering if goal is set
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
    if (goalSet_) {
      status_ = pfms::PlatformStatus::RUNNING;
      wandering_ = false;
      res->success = true;
      res->message = "Flying to goal position";
      RCLCPP_INFO(this->get_logger(), "Control enabled - flying to goal");
    } else {
      res->success = false;
      res->message = "No goal set. Publish to /drone/goal_stamped first";
      RCLCPP_WARN(this->get_logger(), "No goal set - publish to /drone/goal_stamped first");
    }
  } else {
    status_ = pfms::PlatformStatus::LANDING;
    wandering_ = false;
    res->success = true;
    res->message = "Stopping and landing";
    RCLCPP_INFO(this->get_logger(), "Control disabled - landing");
  }
}

void Quadcopter::wanderControl(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  if (req->data) {
    wandering_ = true;
    goalSet_ = false;
    status_ = pfms::PlatformStatus::RUNNING;
    wander_direction_change_time_ = this->now();
    generateRandomDirection();
    res->success = true;
    res->message = "Wandering mode enabled - maintaining 1.5m AGL";
    RCLCPP_INFO(this->get_logger(), "Wandering mode enabled!");
  } else {
    wandering_ = false;
    res->success = true;
    res->message = "Wandering mode disabled";
    RCLCPP_INFO(this->get_logger(), "Wandering mode disabled");
  }
}

void Quadcopter::generateRandomDirection()
{
  // Random forward velocity (0.3 to 1.2 m/s)
  std::uniform_real_distribution<double> speed_dist(0.3, 1.2);
  wander_speed_ = speed_dist(random_engine_);
  
  // Random yaw rate (-0.3 to 0.3 rad/s for gentle turning)
  std::uniform_real_distribution<double> yaw_dist(-0.3, 0.3);
  wander_yaw_rate_ = yaw_dist(random_engine_);
  
  // Random duration (3 to 8 seconds before changing direction)
  std::uniform_real_distribution<double> time_dist(3.0, 8.0);
  wander_duration_ = time_dist(random_engine_);
  
  RCLCPP_INFO(this->get_logger(), 
              "New wander direction: speed=%.2f m/s, yaw_rate=%.2f rad/s, duration=%.1fs",
              wander_speed_, wander_yaw_rate_, wander_duration_);
}

double Quadcopter::maintainAltitude()
{
  if (!agl_received_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "No AGL data received yet");
    return 0.0;
  }
  
  // Proportional control for altitude
  const double agl_error = target_agl_ - current_agl_;
  const double kp_alt = 0.8;  // Altitude gain
  double vz = kp_alt * agl_error;
  
  // Clamp vertical velocity
  const double max_vz = 1.0;
  vz = std::max(-max_vz, std::min(max_vz, vz));
  
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                       "AGL: %.2fm, Target: %.2fm, Error: %.2fm, vz: %.2fm/s",
                       current_agl_, target_agl_, agl_error, vz);
  
  return vz;
}

bool Quadcopter::reachGoal(void)
{
  // Debug counter
  static int debug_counter = 0;
  debug_counter++;
  if (debug_counter % 20 == 0) {
    RCLCPP_DEBUG(this->get_logger(), "reachGoal() - Status: %d, goalSet: %s, wandering: %s", 
                static_cast<int>(status_), goalSet_ ? "true" : "false",
                wandering_ ? "true" : "false");
  }

  switch (status_) {
    case pfms::PlatformStatus::IDLE:
      sendCmd(0, 0, 0, 0);
      return false;

    case pfms::PlatformStatus::TAKEOFF:
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "In TAKEOFF state");
      break;

    case pfms::PlatformStatus::LANDING:
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "In LANDING state");
      sendLanding();
      sendCmd(0, 0, 0, 0);
      status_ = pfms::PlatformStatus::IDLE;
      return true;

    case pfms::PlatformStatus::RUNNING:
      if (wandering_) {
        // WANDERING MODE
        auto pose = getOdometry();
        
        // Check if it's time to change direction
        auto elapsed = (this->now() - wander_direction_change_time_).seconds();
        if (elapsed > wander_duration_) {
          generateRandomDirection();
          wander_direction_change_time_ = this->now();
        }
        
        // Maintain altitude at target AGL
        double vz = maintainAltitude();
        
        // Move in the random direction
        sendCmd(wander_yaw_rate_, 0.0, vz, wander_speed_);
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Wandering: speed=%.2f, yaw=%.2f, alt=%.2fm/%.2fm, pos=(%.1f, %.1f)",
                             wander_speed_, wander_yaw_rate_, current_agl_, target_agl_,
                             pose.position.x, pose.position.y);
        return true;
      }
      else if (goalSet_) {
        // GOAL-SEEKING MODE (existing behavior)
        if (goalReached()) {
          sendCmd(0, 0, 0, 0);
          RCLCPP_INFO(this->get_logger(), "Goal reached! Hovering at target position");
          goalSet_ = false;
          status_ = pfms::PlatformStatus::IDLE;
          return true;
        }

        auto pose = getOdometry();
        
        const double dx = goalPosition_.x - pose.position.x;
        const double dy = goalPosition_.y - pose.position.y;
        const double dz = goalPosition_.z - pose.position.z;
        
        const double kp_xy = 0.8;
        const double kp_z = 0.5;
        const double max_vel = 2.0;
        
        double vx = kp_xy * dx;
        double vy = kp_xy * dy;
        double vz = kp_z * dz;
        
        vx = std::max(-max_vel, std::min(max_vel, vx));
        vy = std::max(-max_vel, std::min(max_vel, vy));
        vz = std::max(-max_vel, std::min(max_vel, vz));
        
        sendCmd(0.0, vy, vz, vx);
        
        double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                             "Flying to goal - distance remaining: %.2f m", distance);
        return true;
      }
      else {
        // No goal and not wandering - hover
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                             "No goal or wander mode - hovering");
        sendCmd(0, 0, 0, 0);
        return false;
      }
  }

  return false;
}