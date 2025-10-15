#pragma once

#include <atomic>
#include <limits>
#include <algorithm>
#include <cmath>
#include <random>
#include <optional>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/set_bool.hpp>

class Controller : public rclcpp::Node {
public:
  explicit Controller(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // For debugging/telemetry if another class wants to read it
  float getMinAhead() const { return min_ahead_.load(std::memory_order_relaxed); }

private:
  // ===== Parameters =====
  double loop_hz_{20.0};

  // AGL hold
  double agl_target_{1.5};    // [m] target height above ground
  double agl_kp_{0.8};        // P gain for vertical speed
  double vz_limit_{0.8};      // [m/s] vertical speed clamp

  // LiDAR gating (forward arc)
  double obs_stop_dist_{1.2};     // [m] stop below this
  double obs_slow_dist_{2.5};     // [m] start slowing below this
  double lidar_front_fov_deg_{15.0}; // half-FOV (±deg) ahead

  // Goal mode (reach-goal)
  double cruise_speed_{1.2};      // [m/s]
  double min_speed_{0.1};         // [m/s]
  double yaw_kp_{1.2};            // yaw P
  double yaw_rate_limit_{0.6};    // [rad/s]

  // Wander mode
  double wander_speed_min_{0.3};
  double wander_speed_max_{1.2};
  double wander_yaw_rate_max_{0.3};
  double wander_reseed_min_{3.0};
  double wander_reseed_max_{8.0};

  // ===== State =====
  std::atomic<bool> wander_mode_{false};
  std::atomic<bool> reach_goal_mode_{false};

  std::atomic<float>  min_ahead_{std::numeric_limits<float>::infinity()};
  std::atomic<double> agl_meas_{std::numeric_limits<double>::quiet_NaN()};

  std::mutex odom_mtx_;
  double x_{0.0}, y_{0.0}, z_{0.0}, yaw_{0.0};
  bool have_odom_{false};

  std::mutex goal_mtx_;
  std::optional<geometry_msgs::msg::Point> goal_world_;

  std::mt19937 rng_;
  double wander_target_speed_{0.6};
  double wander_target_yaw_rate_{0.0};
  rclcpp::Time next_wander_reseed_;

  // ===== ROS I/O =====
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_agl_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_goal_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_wander_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_reach_goal_;

  rclcpp::TimerBase::SharedPtr timer_;

  // ===== Helpers =====
  void controlLoop();
  void reseedWander();

  static double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(hi, v));
  }
};
