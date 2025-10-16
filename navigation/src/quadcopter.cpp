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
  goalSet_(false),
  wandering_(false),
  pattern_mode_(false),
  target_agl_(23)
{
  tolerance_ = 0.5; // 0.5 m sphere tolerance

  // Declare pattern parameters
  this->declare_parameter("pattern_type", "spiral");
  this->declare_parameter("pattern_leg_length", 25.0);
  this->declare_parameter("pattern_lane_spacing", 8.0);
  this->declare_parameter("pattern_spiral_yaw_rate", 0.25);
  this->declare_parameter("pattern_spiral_speed", 1.2);
  this->declare_parameter("pattern_spiral_inflate", 0.05);
  this->declare_parameter("pattern_box_side", 20.0);
  
  // Declare collision avoidance parameters
  this->declare_parameter("obs_stop_dist", 1.2);
  this->declare_parameter("obs_slow_dist", 2.5);
  this->declare_parameter("lidar_front_fov_deg", 15.0);
  this->declare_parameter("lidar_side_min_deg", 30.0);
  this->declare_parameter("lidar_side_max_deg", 90.0);
  
  // Get parameters
  pattern_type_ = this->get_parameter("pattern_type").as_string();
  pattern_leg_length_ = this->get_parameter("pattern_leg_length").as_double();
  pattern_lane_spacing_ = this->get_parameter("pattern_lane_spacing").as_double();
  pattern_spiral_yaw_rate_ = this->get_parameter("pattern_spiral_yaw_rate").as_double();
  pattern_spiral_speed_ = this->get_parameter("pattern_spiral_speed").as_double();
  pattern_spiral_inflate_ = this->get_parameter("pattern_spiral_inflate").as_double();
  pattern_box_side_ = this->get_parameter("pattern_box_side").as_double();
  
  obs_stop_dist_ = this->get_parameter("obs_stop_dist").as_double();
  obs_slow_dist_ = this->get_parameter("obs_slow_dist").as_double();
  lidar_front_fov_deg_ = this->get_parameter("lidar_front_fov_deg").as_double();
  lidar_side_min_deg_ = this->get_parameter("lidar_side_min_deg").as_double();
  lidar_side_max_deg_ = this->get_parameter("lidar_side_max_deg").as_double();

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

  // Subscriber for LiDAR
  subLidar_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(),
      std::bind(&Quadcopter::lidarCallback, this, std::placeholders::_1));

  // Service to start/stop motion control
  srvReachGoal_ = this->create_service<std_srvs::srv::SetBool>(
      "/reach_goal",
      std::bind(&Quadcopter::control, this, std::placeholders::_1, std::placeholders::_2));

  // Service to enable/disable wandering mode
  srvWander_ = this->create_service<std_srvs::srv::SetBool>(
      "/wander_mode",
      std::bind(&Quadcopter::wanderControl, this, std::placeholders::_1, std::placeholders::_2));

  // Service to enable/disable pattern mode
  srvPattern_ = this->create_service<std_srvs::srv::SetBool>(
      "/pattern_mode",
      std::bind(&Quadcopter::patternControl, this, std::placeholders::_1, std::placeholders::_2));

  // 20 Hz control loop for smooth flight
  timer_ = this->create_wall_timer(50ms, std::bind(&Quadcopter::reachGoal, this));

  // Random number generator for wandering
  random_engine_.seed(std::chrono::system_clock::now().time_since_epoch().count());

  RCLCPP_INFO(this->get_logger(), "Quadcopter node initialized");
  RCLCPP_INFO(this->get_logger(), "Services:");
  RCLCPP_INFO(this->get_logger(), "  - /reach_goal (data=true: fly to goal)");
  RCLCPP_INFO(this->get_logger(), "  - /wander_mode (data=true: wander aimlessly)");
  RCLCPP_INFO(this->get_logger(), "  - /pattern_mode (data=true: pattern search)");
  RCLCPP_INFO(this->get_logger(), "Topics:");
  RCLCPP_INFO(this->get_logger(), "  - /drone/goal_stamped (set target position)");
  RCLCPP_INFO(this->get_logger(), "  - /drone/agl_distance (altitude above ground)");
  RCLCPP_INFO(this->get_logger(), "  - /scan (LiDAR for collision avoidance)");
  RCLCPP_INFO(this->get_logger(), "Pattern type: %s", pattern_type_.c_str());
  RCLCPP_INFO(this->get_logger(), "Collision avoidance: stop=%.1fm, slow=%.1fm", 
              obs_stop_dist_, obs_slow_dist_);
}

Quadcopter::~Quadcopter() = default;

void Quadcopter::aglCallback(const std_msgs::msg::Float64::SharedPtr msg)
{
  current_agl_ = msg->data;
  agl_received_ = true;
}

void Quadcopter::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (msg->ranges.empty()) return;
  
  // Helper to clamp index
  auto clampIndex = [&](int i) -> int {
    return std::max(0, std::min((int)msg->ranges.size() - 1, i));
  };
  
  // Convert angle to index
  auto angleToIndex = [&](double angle) -> int {
    return (int)std::round((angle - msg->angle_min) / msg->angle_increment);
  };
  
  // FRONT SECTOR: Check directly ahead
  const double front_fov = lidar_front_fov_deg_ * M_PI / 180.0;
  int i_min_front = clampIndex(angleToIndex(-front_fov));
  int i_max_front = clampIndex(angleToIndex(front_fov));
  if (i_max_front < i_min_front) std::swap(i_min_front, i_max_front);
  
  // LEFT SECTOR: Check left side
  const double side_min = lidar_side_min_deg_ * M_PI / 180.0;
  const double side_max = lidar_side_max_deg_ * M_PI / 180.0;
  int i_min_left = clampIndex(angleToIndex(side_min));
  int i_max_left = clampIndex(angleToIndex(side_max));
  if (i_max_left < i_min_left) std::swap(i_min_left, i_max_left);
  
  // RIGHT SECTOR: Check right side
  int i_min_right = clampIndex(angleToIndex(-side_max));
  int i_max_right = clampIndex(angleToIndex(-side_min));
  if (i_max_right < i_min_right) std::swap(i_min_right, i_max_right);
  
  // Find minimum distance in each sector
  auto findMin = [&](int start, int end) -> float {
    float min_dist = std::numeric_limits<float>::infinity();
    for (int i = start; i <= end; ++i) {
      const float r = msg->ranges[i];
      if (std::isfinite(r) && r >= msg->range_min && r <= msg->range_max) {
        min_dist = std::min(min_dist, r);
      }
    }
    return min_dist;
  };
  
  min_ahead_.store(findMin(i_min_front, i_max_front), std::memory_order_relaxed);
  min_left_.store(findMin(i_min_left, i_max_left), std::memory_order_relaxed);
  min_right_.store(findMin(i_min_right, i_max_right), std::memory_order_relaxed);
  
  // Debug output
  RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                        "LiDAR: ahead=%.2fm, left=%.2fm, right=%.2fm",
                        min_ahead_.load(), min_left_.load(), min_right_.load());
}

void Quadcopter::applyCollisionAvoidance(double& vx, double& yaw_rate)
{
  const float ahead = getMinAhead();
  const float left = getMinLeft();
  const float right = getMinRight();
  
  if (!std::isfinite(ahead)) return;  // No valid readings
  
  // STOP: Obstacle very close ahead
  if (ahead < obs_stop_dist_) {
    vx = 0.0;
    
    // Try to turn away from obstacle
    // Turn toward the side with more space
    if (std::isfinite(left) && std::isfinite(right)) {
      if (left > right) {
        yaw_rate += 0.3;  // Turn left
      } else {
        yaw_rate -= 0.3;  // Turn right
      }
    } else if (std::isfinite(left)) {
      yaw_rate += 0.3;  // Turn left
    } else if (std::isfinite(right)) {
      yaw_rate -= 0.3;  // Turn right
    }
    
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "STOP! Obstacle at %.2fm", ahead);
    return;
  }
  
  // SLOW DOWN: Obstacle moderately close
  if (ahead < obs_slow_dist_) {
    const double alpha = (ahead - obs_stop_dist_) / (obs_slow_dist_ - obs_stop_dist_);
    vx *= std::max(0.0, std::min(1.0, alpha));  // Reduce speed proportionally
    
    // Gentle steering away
    if (std::isfinite(left) && std::isfinite(right)) {
      if (left > right) {
        yaw_rate += 0.1;  // Gentle turn left
      } else {
        yaw_rate -= 0.1;  // Gentle turn right
      }
    }
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Slowing down, obstacle at %.2fm", ahead);
  }
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
  wandering_ = false;
  pattern_mode_ = false;
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
      pattern_mode_ = false;
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
    pattern_mode_ = false;
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
    pattern_mode_ = false;
    status_ = pfms::PlatformStatus::RUNNING;
    wander_direction_change_time_ = this->now();
    generateRandomDirection();
    res->success = true;
    res->message = "Wandering mode enabled with collision avoidance";
    RCLCPP_INFO(this->get_logger(), "Wandering mode enabled!");
  } else {
    wandering_ = false;
    res->success = true;
    res->message = "Wandering mode disabled";
    RCLCPP_INFO(this->get_logger(), "Wandering mode disabled");
  }
}

void Quadcopter::patternControl(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                                 std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  if (req->data) {
    // Get current pattern type from parameter
    pattern_type_ = this->get_parameter("pattern_type").as_string();
    
    pattern_mode_ = true;
    wandering_ = false;
    goalSet_ = false;
    status_ = pfms::PlatformStatus::RUNNING;
    patternReset();
    res->success = true;
    res->message = "Pattern mode enabled: " + pattern_type_ + " with collision avoidance";
    RCLCPP_INFO(this->get_logger(), "Pattern search mode enabled! Type: %s", pattern_type_.c_str());
  } else {
    pattern_mode_ = false;
    res->success = true;
    res->message = "Pattern mode disabled";
    RCLCPP_INFO(this->get_logger(), "Pattern mode disabled");
  }
}

void Quadcopter::patternReset()
{
  auto pose = getOdometry();
  pattern_origin_.x = pose.position.x;
  pattern_origin_.y = pose.position.y;
  pattern_origin_.z = pose.position.z;
  pattern_start_time_ = this->now();
  
  // Reset pattern-specific state
  pattern_lane_ = 0;
  pattern_heading_east_ = true;
  pattern_spiral_radius_ = 0.0;
  pattern_spiral_angle_ = 0.0;
  pattern_box_side_idx_ = 0;
  
  RCLCPP_INFO(this->get_logger(), "Pattern reset at origin: (%.2f, %.2f, %.2f)", 
              pattern_origin_.x, pattern_origin_.y, pattern_origin_.z);
}

void Quadcopter::generatePatternWaypoint()
{
  // This function updates the desired heading and speed based on pattern type
}

void Quadcopter::generateRandomDirection()
{
  std::uniform_real_distribution<double> speed_dist(0.3, 1.2);
  wander_speed_ = speed_dist(random_engine_);
  
  std::uniform_real_distribution<double> yaw_dist(-0.3, 0.3);
  wander_yaw_rate_ = yaw_dist(random_engine_);
  
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
  
  const double agl_error = target_agl_ - current_agl_;
  const double kp_alt = 0.8;
  double vz = kp_alt * agl_error;
  
  const double max_vz = 1.0;
  vz = std::max(-max_vz, std::min(max_vz, vz));
  
  return vz;
}

bool Quadcopter::reachGoal(void)
{
  switch (status_) {
    case pfms::PlatformStatus::IDLE:
      sendCmd(0, 0, 0, 0);
      return false;

    case pfms::PlatformStatus::TAKEOFF:
      break;

    case pfms::PlatformStatus::LANDING:
      sendLanding();
      sendCmd(0, 0, 0, 0);
      status_ = pfms::PlatformStatus::IDLE;
      return true;

    case pfms::PlatformStatus::RUNNING:
      if (pattern_mode_) {
        // ========== PATTERN SEARCH MODE WITH COLLISION AVOIDANCE ==========
        auto pose = getOdometry();
        double vz = maintainAltitude();
        double yaw_rate = 0.0;
        double vx = 0.0;
        
        if (pattern_type_ == "spiral") {
          // SPIRAL PATTERN - Archimedean spiral with increasing radius
          // Update spiral parameters
          const double dt = 0.05;  // 20Hz control loop
          pattern_spiral_angle_ += pattern_spiral_yaw_rate_ * dt;
          pattern_spiral_radius_ = pattern_spiral_inflate_ * pattern_spiral_angle_;
          
          // Calculate desired position on spiral
          double target_x = pattern_origin_.x + pattern_spiral_radius_ * std::cos(pattern_spiral_angle_);
          double target_y = pattern_origin_.y + pattern_spiral_radius_ * std::sin(pattern_spiral_angle_);
          
          // Get current position
          double dx = target_x - pose.position.x;
          double dy = target_y - pose.position.y;
          
          // Calculate desired heading toward next spiral point
          double desired_yaw = std::atan2(dy, dx);
          
          // Get current yaw
          tf2::Quaternion q(pose.orientation.x, pose.orientation.y,
                           pose.orientation.z, pose.orientation.w);
          double roll, pitch, yaw;
          tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
          
          // Yaw error
          double yaw_error = desired_yaw - yaw;
          while (yaw_error > M_PI) yaw_error -= 2.0*M_PI;
          while (yaw_error < -M_PI) yaw_error += 2.0*M_PI;
          
          yaw_rate = 1.2 * yaw_error;
          yaw_rate = std::max(-0.6, std::min(0.6, yaw_rate));
          
          vx = pattern_spiral_speed_;
          
          // Apply collision avoidance
          applyCollisionAvoidance(vx, yaw_rate);
          
          sendCmd(yaw_rate, 0.0, vz, vx);
          
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                               "SPIRAL: r=%.2fm, angle=%.2frad, target=(%.1f,%.1f), ahead=%.2fm",
                               pattern_spiral_radius_, pattern_spiral_angle_, 
                               target_x, target_y, getMinAhead());
                               
        } else if (pattern_type_ == "lawnmower") {
          // LAWNMOWER PATTERN
          double dx = pose.position.x - pattern_origin_.x;
          double dy = pose.position.y - (pattern_origin_.y + pattern_lane_ * pattern_lane_spacing_);
          double along = pattern_heading_east_ ? dx : -dx;
          
          // Check if reached end of leg
          if (along >= pattern_leg_length_) {
            pattern_heading_east_ = !pattern_heading_east_;
            pattern_lane_++;
            pattern_origin_.y += pattern_lane_spacing_;
            RCLCPP_INFO(this->get_logger(), "Lawnmower: switching to lane %d, heading %s",
                       pattern_lane_, pattern_heading_east_ ? "east" : "west");
          }
          
          // Move forward in current direction
          double target_yaw = pattern_heading_east_ ? 0.0 : M_PI;
          
          // Get current yaw from pose
          tf2::Quaternion q(pose.orientation.x, pose.orientation.y, 
                           pose.orientation.z, pose.orientation.w);
          double roll, pitch, yaw;
          tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
          
          double yaw_error = target_yaw - yaw;
          while (yaw_error > M_PI) yaw_error -= 2.0*M_PI;
          while (yaw_error < -M_PI) yaw_error += 2.0*M_PI;
          
          yaw_rate = 1.2 * yaw_error;
          yaw_rate = std::max(-0.6, std::min(0.6, yaw_rate));
          
          vx = 1.2;  // cruise speed
          
          // Apply collision avoidance
          applyCollisionAvoidance(vx, yaw_rate);
          
          sendCmd(yaw_rate, 0.0, vz, vx);
          
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                               "LAWNMOWER: lane=%d, along=%.2fm/%.2fm, ahead=%.2fm",
                               pattern_lane_, along, pattern_leg_length_, getMinAhead());
                               
        } else if (pattern_type_ == "box") {
          // BOX PATTERN
          double dx = pose.position.x - pattern_origin_.x;
          double dy = pose.position.y - pattern_origin_.y;
          double L = pattern_box_side_;
          
          int side = pattern_box_side_idx_ % 4;
          double target_yaw;
          bool advance = false;
          
          switch (side) {
            case 0: // +X (east)
              target_yaw = 0.0;
              if (dx >= L) advance = true;
              break;
            case 1: // +Y (north)
              target_yaw = M_PI/2.0;
              if (dy >= L) advance = true;
              break;
            case 2: // -X (west)
              target_yaw = M_PI;
              if (dx <= -L) advance = true;
              break;
            case 3: // -Y (south)
              target_yaw = -M_PI/2.0;
              if (dy <= -L) advance = true;
              break;
          }
          
          if (advance) {
            pattern_box_side_idx_++;
            RCLCPP_INFO(this->get_logger(), "Box: advancing to side %d", pattern_box_side_idx_ % 4);
          }
          
          // Get current yaw
          tf2::Quaternion q(pose.orientation.x, pose.orientation.y,
                           pose.orientation.z, pose.orientation.w);
          double roll, pitch, yaw;
          tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
          
          double yaw_error = target_yaw - yaw;
          while (yaw_error > M_PI) yaw_error -= 2.0*M_PI;
          while (yaw_error < -M_PI) yaw_error += 2.0*M_PI;
          
          yaw_rate = 1.2 * yaw_error;
          yaw_rate = std::max(-0.6, std::min(0.6, yaw_rate));
          
          vx = 1.2;  // cruise speed
          
          // Apply collision avoidance
          applyCollisionAvoidance(vx, yaw_rate);
          
          sendCmd(yaw_rate, 0.0, vz, vx);
          
          RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                               "BOX: side=%d, pos=(%.2f,%.2f), ahead=%.2fm", 
                               side, dx, dy, getMinAhead());
        }
        
        return true;
        
      } else if (wandering_) {
        // ========== WANDER MODE WITH COLLISION AVOIDANCE ==========
        auto pose = getOdometry();
        
        auto elapsed = (this->now() - wander_direction_change_time_).seconds();
        if (elapsed > wander_duration_) {
          generateRandomDirection();
          wander_direction_change_time_ = this->now();
        }
        
        double vz = maintainAltitude();
        double yaw_rate = wander_yaw_rate_;
        double vx = wander_speed_;
        
        // Apply collision avoidance
        applyCollisionAvoidance(vx, yaw_rate);
        
        sendCmd(yaw_rate, 0.0, vz, vx);
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Wandering: speed=%.2f, yaw=%.2f, ahead=%.2fm",
                             vx, yaw_rate, getMinAhead());
        return true;
        
      } else if (goalSet_) {
        // ========== GOAL-SEEKING MODE WITH COLLISION AVOIDANCE ==========
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
        
        // Apply collision avoidance to forward speed
        double yaw_rate = 0.0;
        applyCollisionAvoidance(vx, yaw_rate);
        
        sendCmd(yaw_rate, vy, vz, vx);
        
        double distance = std::sqrt(dx*dx + dy*dy + dz*dz);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                             "Goal: dist=%.2fm, ahead=%.2fm", distance, getMinAhead());
        return true;
        
      } else {
        // No mode active - hover
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                             "No goal, wander, or pattern mode - hovering");
        sendCmd(0, 0, 0, 0);
        return false;
      }
  }

  return false;
}