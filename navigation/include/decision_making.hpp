#ifndef DECISION_MAKING_HPP
#define DECISION_MAKING_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>

// Forward declaration
class Quadcopter;

class DecisionMaking : public rclcpp::Node
{
public:
    explicit DecisionMaking(std::shared_ptr<Quadcopter> drone);

    enum class MissionState
    {
        SEARCHING,
        DETECTION,
        INSPECTING,
        RETURNING
    };

private:
    // Configuration parameters
    static constexpr double GOAL_REACH_THRESHOLD = 1.0;  // meters
    static constexpr int INSPECTION_DURATION_SEC = 3;
    static constexpr int GOAL_TIMEOUT_SEC = 30;  // Add timeout for safety

    // Core members
    std::shared_ptr<Quadcopter> drone_;
    MissionState mission_state_{MissionState::SEARCHING};
    
    // Detection tracking
    bool detection_in_progress_{false};
    geometry_msgs::msg::PoseStamped detection_pose_;
    geometry_msgs::msg::PoseStamped last_search_pose_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr inspect_timer_;
    rclcpp::TimerBase::SharedPtr goal_timeout_timer_;
    rclcpp::Time goal_start_time_;
    
    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr thermal_sub_;

    // Callbacks
    void thermalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void reachGoal();
    void finishInspection();
    void handleGoalTimeout();
    
    // Utility functions
    double distanceToGoal(const geometry_msgs::msg::Point &goal);
    bool setGoalSafely(const geometry_msgs::msg::Point &goal);
    void transitionToState(MissionState new_state);
};

#endif // DECISION_MAKING_HPP