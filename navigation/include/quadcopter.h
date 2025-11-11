#ifndef QUADCOPTER_H
#define QUADCOPTER_H

#include "controller.h"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <chrono>
#include <random>
#include <atomic>



namespace pfms {
  enum class PlatformStatus {
    IDLE,
    TAKEOFF,
    RUNNING,
    LANDING
  };
}

class Quadcopter : public rclcpp::Node
{
public:
  Quadcopter();
  virtual ~Quadcopter();

  // Core control functions
  bool reachGoal(void);
  GoalStats calcNewGoal(void);
  bool checkOriginToDestination(geometry_msgs::msg::Pose origin,
                                geometry_msgs::msg::Point goal,
                                double& distance, double& time,
                                geometry_msgs::msg::Pose& estimatedGoalPose);

  // Goal management
  void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  bool goalReached(void);

  // Command sending
  void sendCmd(double yaw_rate, double move_l_r, double move_u_d, double move_f_b);
  void sendTakeOff(void);
  void sendLanding(void);

  // Service callbacks
  void control(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
               std::shared_ptr<std_srvs::srv::SetBool::Response> res);
  void wanderControl(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                     std::shared_ptr<std_srvs::srv::SetBool::Response> res);
  void patternControl(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                      std::shared_ptr<std_srvs::srv::SetBool::Response> res);

  // Wander mode
  void generateRandomDirection();
  double maintainAltitude();
  
  // Callbacks
  void aglCallback(const std_msgs::msg::Float64::SharedPtr msg);
  void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);  // IMU callback

  // Pattern mode functions
  void patternReset();
  void generatePatternWaypoint();
  
  // Collision avoidance
  void applyCollisionAvoidance(double& vx, double& yaw_rate);
  
  // NEW: Attitude stabilization
  void applyAttitudeStabilization(double& move_l_r, double& move_f_b);
  bool needsEmergencyStabilization();
  
  // Inline accessors for LiDAR data
  float getMinAhead() const { return min_ahead_.load(std::memory_order_relaxed); }
  float getMinLeft() const { return min_left_.load(std::memory_order_relaxed); }
  float getMinRight() const { return min_right_.load(std::memory_order_relaxed); }
  geometry_msgs::msg::Pose getOdometry();
  geometry_msgs::msg::PoseStamped getCurrentPose() const;
  void setGoal(const geometry_msgs::msg::Point &goal);
  void stopMovement();
  void resumeSearch();

 


private:
  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pubCmdVel_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pubTakeOff_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pubLanding_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subGoal_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subAgl_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subLidar_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu_;  // IMU subscriber

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subOdom_;
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);


  // Services
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srvReachGoal_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srvWander_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srvPattern_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // State variables
  pfms::PlatformStatus status_ = pfms::PlatformStatus::IDLE;
  bool liftoff_;
  bool landed_ = false;
  bool goalSet_ = false;
  bool wandering_ = false;
  bool pattern_mode_ = false;
  
  geometry_msgs::msg::Point goalPosition_;
   geometry_msgs::msg::PoseStamped current_pose_;
  double target_angle_ = 0.0;
  double tolerance_;

  // AGL tracking
  double current_agl_ = 0.0;
  double target_agl_ = 1.5;
  bool agl_received_ = false;

  
  double current_roll_ = 0.0;
  double current_pitch_ = 0.0;
  double current_yaw_ = 0.0;
  double roll_rate_ = 0.0;   // Angular velocity around x-axis
  double pitch_rate_ = 0.0;  // Angular velocity around y-axis
  double yaw_rate_ = 0.0;    // Angular velocity around z-axis
  bool imu_received_ = false;
  rclcpp::Time last_imu_time_;
  
  // Attitude stabilization PID parameters
  double roll_kp_ = 2.0;      // Proportional gain for roll
  double roll_kd_ = 0.5;      // Derivative gain for roll
  double pitch_kp_ = 2.0;     // Proportional gain for pitch
  double pitch_kd_ = 0.5;     // Derivative gain for pitch
  
  //Emergency stabilization thresholds
  double max_safe_roll_ = 0.35;   // ~20 degrees max roll before emergency
  double max_safe_pitch_ = 0.35;  // ~20 degrees max pitch before emergency
  double emergency_roll_ = 0.52;  // ~30 degrees - CRITICAL recovery needed
  double emergency_pitch_ = 0.52; // ~30 degrees - CRITICAL recovery needed
  
  // Stabilization state
  bool in_emergency_stabilization_ = false;
  rclcpp::Time emergency_stabilization_start_;

  // LiDAR collision avoidance
  std::atomic<float> min_ahead_{std::numeric_limits<float>::infinity()};
  std::atomic<float> min_left_{std::numeric_limits<float>::infinity()};
  std::atomic<float> min_right_{std::numeric_limits<float>::infinity()};
  double obs_stop_dist_;
  double obs_slow_dist_;
  double lidar_front_fov_deg_;
  double lidar_side_min_deg_;
  double lidar_side_max_deg_;

  // Wander mode state
  std::mt19937 random_engine_;
  double wander_speed_ = 0.0;
  double wander_yaw_rate_ = 0.0;
  double wander_duration_ = 0.0;
  rclcpp::Time wander_direction_change_time_;

  // Pattern mode state
  std::string pattern_type_ = "spiral";  // Default: spiral
  geometry_msgs::msg::Point pattern_origin_;
  geometry_msgs::msg::Point pattern_current_waypoint_;
  rclcpp::Time pattern_start_time_;
  
  // Lawnmower pattern state
  int pattern_lane_ = 0;
  bool pattern_heading_east_ = true;
  double pattern_leg_length_ = 25.0;
  double pattern_lane_spacing_ = 8.0;
  
  // Spiral pattern state
  double pattern_spiral_radius_ = 0.0;
  double pattern_spiral_angle_ = 0.0;
  double pattern_spiral_yaw_rate_ = 0.25;  // rad/s
  double pattern_spiral_speed_ = 1.2;      // m/s
  double pattern_spiral_inflate_ = 0.05;   // radius growth per second
  
  // Box pattern state
  int pattern_box_side_idx_ = 0;
  double pattern_box_side_ = 20.0;

  // Constants
  const double TARGET_SPEED;
  const double TARGET_HEIGHT_TOLERANCE;
};

#endif // QUADCOPTER_H