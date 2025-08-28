#include "quadcopter.h"
#include <cmath>
#include <chrono>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

Quadcopter::Quadcopter()
: liftoff_(false),
  TARGET_SPEED(0.4),
  TARGET_HEIGHT_TOLERANCE(0.2)
{
  // tolerances
  tolerance_ = 0.5; // 0.5 m sphere

  // Publishers (match your existing topics)
  //   drone/cmd_vel, drone/takeoff, drone/land  (Twist + Empty)
  pubCmdVel_  = this->create_publisher<geometry_msgs::msg::Twist>("drone/cmd_vel", 3);   // :contentReference[oaicite:8]{index=8}
  pubTakeOff_ = this->create_publisher<std_msgs::msg::Empty>("drone/takeoff", 3);        // :contentReference[oaicite:9]{index=9}
  pubLanding_ = this->create_publisher<std_msgs::msg::Empty>("drone/land", 3);           // :contentReference[oaicite:10]{index=10}

  // Service to start/stop motion to current goal (true=start/arm, false=land/stop)
  srvReachGoal_ = this->create_service<std_srvs::srv::SetBool>(
      "/reach_goal",
      std::bind(&Quadcopter::control, this, std::placeholders::_1, std::placeholders::_2));

  // 10 Hz control loop
  timer_ = this->create_wall_timer(100ms, std::bind(&Quadcopter::reachGoal, this)); // :contentReference[oaicite:11]{index=11}
}

Quadcopter::~Quadcopter() = default;

bool Quadcopter::checkOriginToDestination(geometry_msgs::msg::Pose origin,
                                          geometry_msgs::msg::Point goal,
                                          double& distance, double& time,
                                          geometry_msgs::msg::Pose& estimatedGoalPose)
{
  const double dx = goal.x - origin.position.x;
  const double dy = goal.y - origin.position.y;
  distance = std::hypot(dx, dy);
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
  // NOTE: your example maps fb->x, lr->y, ud->z, yaw->angular.z :contentReference[oaicite:12]{index=12}
  msg.linear.x  = move_f_b;
  msg.linear.y  = move_l_r;
  msg.linear.z  = move_u_d;
  msg.angular.z = yaw_rate;
  pubCmdVel_->publish(msg);
}

void Quadcopter::sendTakeOff(void)
{
  std_msgs::msg::Empty e;
  pubTakeOff_->publish(e);
  liftoff_ = true;  landed_ = false;
}

void Quadcopter::sendLanding(void)
{
  std_msgs::msg::Empty e;
  pubLanding_->publish(e);
  liftoff_ = false; landed_ = true;
}

void Quadcopter::control(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                         std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  // true  -> TAKEOFF/RUNNING; false -> LANDING/IDLE
  if (req->data) {
    status_ = pfms::PlatformStatus::TAKEOFF;
    res->success = true;
    res->message = "Starting reach-goal control (TAKEOFF → RUNNING)";
  } else {
    status_ = pfms::PlatformStatus::LANDING;
    res->success = true;
    res->message = "Stopping control (LANDING → IDLE)";
  }
}

// 10 Hz tick
bool Quadcopter::reachGoal(void)
{
  // Small state machine exactly like your example (IDLE/TAKEOFF/RUNNING/LANDING) :contentReference[oaicite:13]{index=13}
  switch (status_) {
    case pfms::PlatformStatus::IDLE:
      return false;

    case pfms::PlatformStatus::TAKEOFF:
      sendTakeOff();
      status_ = goalSet_ ? pfms::PlatformStatus::RUNNING : pfms::PlatformStatus::IDLE;
      return true;

    case pfms::PlatformStatus::LANDING:
      sendLanding();
      status_ = pfms::PlatformStatus::IDLE;
      return true;

    case pfms::PlatformStatus::RUNNING:
      break;
  }

  if (!goalSet_) return false;

  // (1) Recompute path geometry
  auto goalStats = calcNewGoal();
  auto pose = getOdometry();

  // (2) Compute relative heading (world yaw - required target yaw) and body-frame XY
  const double theta = tf2::getYaw(pose.orientation) - target_angle_;     // :contentReference[oaicite:14]{index=14}
  const double vx = TARGET_SPEED * std::cos(theta);
  const double vy = TARGET_SPEED * std::sin(theta);

  // (3) Simple P control on Z (with clipping)
  const double err_z = goalStats.location.z - pose.position.z;            // :contentReference[oaicite:15]{index=15}
  double vz = 0.5 * err_z;
  if (vz >  1.0) vz =  1.0;
  if (vz < -1.0) vz = -1.0;

  // (4) Command body velocities (+yaw rate = 0 here)
  sendCmd(0.0, -vy, vz, vx);  // same mapping/order as your code :contentReference[oaicite:16]{index=16}

  // (5) Check done → land automatically
  const bool done = goalReached();
  if (done) {
    sendCmd(0, 0, 0, 0);
    status_ = pfms::PlatformStatus::LANDING;
  }
  return done;
}
