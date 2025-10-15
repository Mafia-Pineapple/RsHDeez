#include "controller.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using sensor_msgs::msg::LaserScan;

Controller::Controller(const rclcpp::NodeOptions & options)
: rclcpp::Node("controller", options),
  rng_(std::random_device{}())
{
  // ===== Parameters =====
  loop_hz_             = this->declare_parameter<double>("loop_hz", 20.0);
  agl_target_          = this->declare_parameter<double>("agl_target", 1.5);
  agl_kp_              = this->declare_parameter<double>("agl_kp", 0.8);
  vz_limit_            = this->declare_parameter<double>("vz_limit", 0.8);

  obs_stop_dist_       = this->declare_parameter<double>("obs_stop_dist", 1.2);
  obs_slow_dist_       = this->declare_parameter<double>("obs_slow_dist", 2.5);
  lidar_front_fov_deg_ = this->declare_parameter<double>("lidar_front_fov_deg", 15.0);

  cruise_speed_        = this->declare_parameter<double>("cruise_speed", 1.2);
  min_speed_           = this->declare_parameter<double>("min_speed", 0.1);
  yaw_kp_              = this->declare_parameter<double>("yaw_kp", 1.2);
  yaw_rate_limit_      = this->declare_parameter<double>("yaw_rate_limit", 0.6);

  wander_speed_min_    = this->declare_parameter<double>("wander_speed_min", 0.3);
  wander_speed_max_    = this->declare_parameter<double>("wander_speed_max", 1.2);
  wander_yaw_rate_max_ = this->declare_parameter<double>("wander_yaw_rate_max", 0.3);
  wander_reseed_min_   = this->declare_parameter<double>("wander_reseed_min", 3.0);
  wander_reseed_max_   = this->declare_parameter<double>("wander_reseed_max", 8.0);

  // ===== Publisher =====
  pub_cmd_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 20);

  // ===== Subscriptions =====
  sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry", 20,
    [this](const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      std::lock_guard<std::mutex> lk(odom_mtx_);
      x_ = msg->pose.pose.position.x;
      y_ = msg->pose.pose.position.y;
      z_ = msg->pose.pose.position.z;

      const auto & q = msg->pose.pose.orientation;
      tf2::Quaternion quat(q.x, q.y, q.z, q.w);
      double roll, pitch, yaw;
      tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
      yaw_ = yaw;
      have_odom_ = true;
    });

  sub_agl_ = this->create_subscription<std_msgs::msg::Float64>(
    "/drone/agl_distance", rclcpp::SensorDataQoS(),
    [this](const std_msgs::msg::Float64::SharedPtr msg)
    {
      agl_meas_.store(msg->data, std::memory_order_relaxed);
    });

  sub_scan_ = this->create_subscription<LaserScan>(
    "/scan", rclcpp::SensorDataQoS(),
    [this](const LaserScan::SharedPtr msg)
    {
      const double fov = lidar_front_fov_deg_ * M_PI / 180.0;
      int i_min = static_cast<int>(std::ceil((-fov - msg->angle_min) / msg->angle_increment));
      int i_max = static_cast<int>(std::floor(( fov - msg->angle_min) / msg->angle_increment));
      i_min = std::max(0, std::min((int)msg->ranges.size() - 1, i_min));
      i_max = std::max(0, std::min((int)msg->ranges.size() - 1, i_max));
      if (i_max < i_min) std::swap(i_min, i_max);

      float m = std::numeric_limits<float>::infinity();
      for (int i = i_min; i <= i_max; ++i) {
        const float r = msg->ranges[i];
        if (std::isfinite(r) && r >= msg->range_min) {
          m = std::min(m, r);
        }
      }
      min_ahead_.store(m, std::memory_order_relaxed);
    });

  sub_goal_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/drone/goal_stamped", 10,
    [this](const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
      std::lock_guard<std::mutex> g(goal_mtx_);
      goal_world_ = msg->point;
      RCLCPP_INFO(this->get_logger(),
                  "New goal: (%.2f, %.2f, %.2f) frame='%s'",
                  msg->point.x, msg->point.y, msg->point.z, msg->header.frame_id.c_str());
    });

  // ===== Services =====
  srv_wander_ = this->create_service<std_srvs::srv::SetBool>(
    "/wander_mode",
    [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
           std::shared_ptr<std_srvs::srv::SetBool::Response> res)
    {
      wander_mode_.store(req->data);
      if (req->data) {
        reach_goal_mode_.store(false);
        reseedWander();
      }
      res->success = true;
      res->message = req->data ? "Wander mode enabled" : "Wander mode disabled";
    });

  srv_reach_goal_ = this->create_service<std_srvs::srv::SetBool>(
    "/reach_goal",
    [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
           std::shared_ptr<std_srvs::srv::SetBool::Response> res)
    {
      reach_goal_mode_.store(req->data);
      if (req->data) {
        wander_mode_.store(false);
      }
      res->success = true;
      res->message = req->data ? "Reach-goal mode enabled" : "Reach-goal mode disabled";
    });

  // ===== Timer / control loop =====
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds((int)std::round(1000.0 / loop_hz_)),
    std::bind(&Controller::controlLoop, this));

  // Initial reseed time for wander
  next_wander_reseed_ = this->get_clock()->now();
}

void Controller::reseedWander()
{
  std::uniform_real_distribution<double> spd(wander_speed_min_, wander_speed_max_);
  std::uniform_real_distribution<double> yaw(-wander_yaw_rate_max_, wander_yaw_rate_max_);
  std::uniform_real_distribution<double> tsec(wander_reseed_min_, wander_reseed_max_);

  wander_target_speed_ = spd(rng_);
  wander_target_yaw_rate_ = yaw(rng_);
  const double dt = tsec(rng_);
  next_wander_reseed_ = this->get_clock()->now() + rclcpp::Duration::from_seconds(dt);
}

void Controller::controlLoop()
{
  geometry_msgs::msg::Twist cmd;

  // === Planar (x/y yaw) ===
  const bool reach_goal = reach_goal_mode_.load();
  const bool wander     = wander_mode_.load();

  if (reach_goal) {
    // Need odom + goal
    geometry_msgs::msg::Point goal;
    {
      std::lock_guard<std::mutex> g(goal_mtx_);
      if (!goal_world_.has_value()) {
        // No goal yet → hold still laterally
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        // AGL handled below; publish at end
        goto AGL_HOLD_AND_PUBLISH;
      }
      goal = *goal_world_;
    }

    double x, y, yaw;
    {
      std::lock_guard<std::mutex> lk(odom_mtx_);
      if (!have_odom_) {
        // No odom yet → hold still
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        goto AGL_HOLD_AND_PUBLISH;
      }
      x = x_; y = y_; yaw = yaw_;
    }

    const double dx = goal.x - x;
    const double dy = goal.y - y;
    const double dist = std::hypot(dx, dy);

    const double desired_yaw = std::atan2(dy, dx);
    double yaw_err = desired_yaw - yaw;
    while (yaw_err >  M_PI) yaw_err -= 2.0*M_PI;
    while (yaw_err < -M_PI) yaw_err += 2.0*M_PI;

    cmd.angular.z = clamp(yaw_kp_ * yaw_err, -yaw_rate_limit_, yaw_rate_limit_);

    double v = cruise_speed_;
    if (dist < 2.5) v = std::max(min_speed_, cruise_speed_ * (dist / 2.5));
    if (std::abs(yaw_err) > M_PI/3.0) v = 0.0; // don't drive when not facing
    cmd.linear.x = v;

  } else if (wander) {
    // Random walk
    const rclcpp::Time now = this->get_clock()->now();
    if (now >= next_wander_reseed_) reseedWander();
    cmd.linear.x  = wander_target_speed_;
    cmd.angular.z = wander_target_yaw_rate_;
  } else {
    // Idle
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
  }

  // === LiDAR obstacle gating on forward speed ===
  {
    const float ahead = getMinAhead();
    if (std::isfinite(ahead)) {
      if (ahead < obs_stop_dist_) {
        cmd.linear.x = 0.0;
        // Optional: if (std::abs(cmd.angular.z) < 0.2) cmd.angular.z = 0.3;
      } else if (ahead < obs_slow_dist_) {
        const double alpha = (ahead - obs_stop_dist_) / (obs_slow_dist_ - obs_stop_dist_);
        cmd.linear.x = std::max(0.0, cmd.linear.x * clamp(alpha, 0.0, 1.0));
      }
    }
  }

AGL_HOLD_AND_PUBLISH:
  // === Altitude control (AGL) → linear.z ===
  {
    const double agl = agl_meas_.load(std::memory_order_relaxed);
    if (std::isfinite(agl)) {
      const double e  = agl_target_ - agl; // +ve => climb
      const double vz = clamp(agl_kp_ * e, -vz_limit_, vz_limit_);
      cmd.linear.z = vz;
    } else {
      cmd.linear.z = 0.0; // no AGL reading
    }
  }

  pub_cmd_->publish(cmd);
}
