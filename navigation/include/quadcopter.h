#ifndef QUADCOPTER_H
#define QUADCOPTER_H

#include "controller.h"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <chrono>

namespace pfms {
  enum class PlatformStatus {
    IDLE,
    TAKEOFF,
    RUNNING,
    LANDING
  };
}

class Quadcopter : public Controller
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

  // Service callback
  void control(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
               std::shared_ptr<std_srvs::srv::SetBool::Response> res);

private:
  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pubCmdVel_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pubTakeOff_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pubLanding_;

  // Subscriber for goal
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subGoal_;

  // Service
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srvReachGoal_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // State variables
  pfms::PlatformStatus status_ = pfms::PlatformStatus::IDLE;
  bool liftoff_;
  bool landed_ = false;
  bool goalSet_ = false;
  geometry_msgs::msg::Point goalPosition_;
  double target_angle_ = 0.0;
  double tolerance_;

  // Constants
  const double TARGET_SPEED;
  const double TARGET_HEIGHT_TOLERANCE;
};

#endif // QUADCOPTER_H