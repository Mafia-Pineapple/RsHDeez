#pragma once
#ifndef CONTROLLERINTERFACE_H
#define CONTROLLERINTERFACE_H

#include <memory>

// ROS messages used in the interface
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace pfms {
  // High-level controller state used across the project
  enum class PlatformStatus {
    IDLE = 0,
    TAKEOFF,
    RUNNING,
    LANDING
  };
}

// Abstract controller API that concrete platforms (e.g., Quadcopter) implement
class ControllerInterface {
public:
  virtual ~ControllerInterface() = default;

  // Main control tick (called periodically). Should return true if the goal
  // was reached on this tick; otherwise false.
  virtual bool reachGoal(void) = 0;

  // Planner-style query to estimate distance/time from origin -> goal and an
  // estimated goal pose on arrival.
  virtual bool checkOriginToDestination(geometry_msgs::msg::Pose origin,
                                        geometry_msgs::msg::Point goal,
                                        double& distance, double& time,
                                        geometry_msgs::msg::Pose& estimatedGoalPose) = 0;

  // Start/stop control (e.g., arm/land) via std_srvs/SetBool:
  //   data=true  -> start/arm (TAKEOFF → RUNNING)
  //   data=false -> stop/land (→ LANDING/IDLE)
  virtual void control(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                       std::shared_ptr<std_srvs::srv::SetBool::Response> res) = 0;
};

#endif // CONTROLLERINTERFACE_H
