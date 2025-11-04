#ifndef DECISION_MAKING_HPP
#define DECISION_MAKING_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <memory>

// Forward declaration to avoid circular dependency
class Quadcopter;

class DecisionMaking : public rclcpp::Node
{
public:
    explicit DecisionMaking(std::shared_ptr<Quadcopter> quadcopter);
    ~DecisionMaking() = default;

private:
    // Callback for thermal camera detections
    void thermalDetectionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    
    // State machine update (runs every 100ms)
    void updateStateMachine();
    
    // Helper to calculate distance to a point
    double distanceToPoint(const geometry_msgs::msg::Point& target);

    // Reference to quadcopter controller
    std::shared_ptr<Quadcopter> quadcopter_;
    
    // ROS subscribers and timers
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr thermal_sub_;
    rclcpp::TimerBase::SharedPtr state_machine_timer_;
    
    // State machine
    enum class State {
        WANDERING,      // Normal patrol/search mode
        FLYING_TO_DETECTION,  // Flying to thermal signature
        INVESTIGATING   // At thermal signature location
    };
    State current_state_;
    
    // Detection tracking
    bool has_active_detection_;
    geometry_msgs::msg::Point detection_target_;
    geometry_msgs::msg::Point wander_resume_point_;
    
    // Investigation timer
    rclcpp::Time investigation_start_time_;
    const double INVESTIGATION_DURATION = 3.0;  // seconds
    const double DETECTION_TOLERANCE = 1.5;     // meters
};

#endif  // DECISION_MAKING_HPP