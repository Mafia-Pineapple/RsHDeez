#ifndef DECISION_MAKING_HPP
#define DECISION_MAKING_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <memory>

#include "quadcopter.h"  // include full class, not just forward declaration

class DecisionMaking : public rclcpp::Node
{
public:
    explicit DecisionMaking(std::shared_ptr<Quadcopter> drone);

private:
    void thermalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void reachGoal();
    void finishInspection();
    double distanceToGoal(const geometry_msgs::msg::Point &goal);

    std::shared_ptr<Quadcopter> drone_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr thermal_sub_;
    rclcpp::TimerBase::SharedPtr inspect_timer_;

    enum class MissionState { SEARCHING, DETECTION, INSPECTING, RETURNING };
    MissionState mission_state_ = MissionState::SEARCHING;
    bool detection_in_progress_ = false;

    geometry_msgs::msg::PoseStamped last_search_pose_;
    geometry_msgs::msg::PoseStamped detection_pose_;
};

#endif  // DECISION_MAKING_HPP
