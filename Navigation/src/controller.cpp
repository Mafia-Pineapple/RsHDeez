#include "controller.h"
using std::placeholders::_1;

Controller::Controller()
: Node("controller")
{
  // Subs created in base (safe; callbacks just write local state)
  sub1_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/drone/gt_odom", 10, std::bind(&Controller::odoCallback, this, _1));  // :contentReference[oaicite:18]{index=18}

  sub2_ = this->create_subscription<geometry_msgs::msg::Point>(
      "/drone/goal", 10, std::bind(&Controller::setGoal, this, _1));         // :contentReference[oaicite:19]{index=19}

  sub3_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", 10, std::bind(&Controller::setGoalClicked, this, _1)); // :contentReference[oaicite:20]{index=20}

  goal_.time = 0.0; goal_.distance = 0.0;
}

void Controller::setGoal(const geometry_msgs::msg::Point& msg)
{
  std::lock_guard<std::mutex> lk(goalMtx_);
  goal_.location = msg;
  goalSet_ = true;
}

void Controller::setGoalClicked(const geometry_msgs::msg::PointStamped& msg)
{
  std::lock_guard<std::mutex> lk(goalMtx_);
  goal_.location.x = msg.point.x;
  goal_.location.y = msg.point.y;
  goal_.location.z = 2.0;
  goalSet_ = true;
}

bool Controller::setTolerance(double t) { tolerance_ = t; return true; }
double Controller::distanceToGoal(void) { return goal_.distance; }
double Controller::timeToGoal(void)      { return goal_.time;      }
double Controller::distanceTravelled(void) { return distance_travelled_; }
double Controller::timeInMotion(void)      { return time_travelled_;    }

GoalStats Controller::getGoalStats(void)
{
  std::lock_guard<std::mutex> lk(goalMtx_);
  return goal_;
}

bool Controller::goalReached()
{
  if (!goalSet_) return false;

  auto g = getGoalStats();
  auto p = getOdometry();

  const double dx = g.location.x - p.position.x;
  const double dy = g.location.y - p.position.y;
  const double dz = g.location.z - p.position.z;
  const double d  = std::sqrt(dx*dx + dy*dy + dz*dz);

  RCLCPP_INFO(this->get_logger(), "distance: %.3f", d);
  return (d < tolerance_);
}

geometry_msgs::msg::Pose Controller::getOdometry(void)
{
  std::lock_guard<std::mutex> lk(poseMtx_);
  return pose_;
}

void Controller::odoCallback(const nav_msgs::msg::Odometry& msg)
{
  std::lock_guard<std::mutex> lk(poseMtx_);
  pose_ = msg.pose.pose;
  // (Optionally accumulate distance/time here if desired)
}
