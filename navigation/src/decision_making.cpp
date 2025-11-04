#include "decision_making.hpp"
#include "quadcopter.h"
#include <cmath>

DecisionMaking::DecisionMaking(std::shared_ptr<Quadcopter> quadcopter)
    : Node("decision_making"),
      quadcopter_(quadcopter),
      current_state_(State::WANDERING),
      has_active_detection_(false)
{
    RCLCPP_INFO(this->get_logger(), "Decision Making node started");
    
    // Subscribe to thermal camera detections
    thermal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/thermal/detections", 10,
        std::bind(&DecisionMaking::thermalDetectionCallback, this, std::placeholders::_1)
    );
    
    // State machine timer - runs at 10Hz
    state_machine_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&DecisionMaking::updateStateMachine, this)
    );
    
    RCLCPP_INFO(this->get_logger(), "Listening for thermal detections on /thermal/detections");
    RCLCPP_INFO(this->get_logger(), "State: WANDERING");
}

void DecisionMaking::thermalDetectionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    // Only accept new detections when in WANDERING mode
    if (current_state_ != State::WANDERING) {
        RCLCPP_INFO(this->get_logger(), "Ignoring detection - already investigating");
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), 
                "⚠️  THERMAL DETECTION at (%.2f, %.2f, %.2f)!", 
                msg->point.x, msg->point.y, msg->point.z);
    
    // Save current position to return to later
    auto current_pose_stamped = quadcopter_->getCurrentPose();
    wander_resume_point_ = current_pose_stamped.pose.position;
    
    RCLCPP_INFO(this->get_logger(), 
                "Saved wander position: (%.2f, %.2f, %.2f)",
                wander_resume_point_.x, wander_resume_point_.y, wander_resume_point_.z);
    
    // Set the detection target
    detection_target_ = msg->point;
    has_active_detection_ = true;
    
    // Command quadcopter to fly to detection
    quadcopter_->setGoal(detection_target_);
    
    // Change state
    current_state_ = State::FLYING_TO_DETECTION;
    RCLCPP_INFO(this->get_logger(), "State: FLYING_TO_DETECTION");
}

void DecisionMaking::updateStateMachine()
{
    switch (current_state_)
    {
        case State::WANDERING:
            // Normal patrol mode - nothing special to do
            // The quadcopter handles its own wandering logic
            break;
            
        case State::FLYING_TO_DETECTION:
        {
            // Check if we've reached the detection point
            double distance = distanceToPoint(detection_target_);
            
            if (distance < DETECTION_TOLERANCE) {
                RCLCPP_INFO(this->get_logger(), 
                            "✓ Reached detection point (%.2f m away)", distance);
                RCLCPP_INFO(this->get_logger(), "State: INVESTIGATING");
                
                current_state_ = State::INVESTIGATING;
                investigation_start_time_ = this->now();
            } else {
                // Still flying - log progress occasionally
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                   "Flying to detection... %.2f m remaining", distance);
            }
            break;
        }
            
        case State::INVESTIGATING:
        {
            // Wait at the detection point for INVESTIGATION_DURATION seconds
            double elapsed = (this->now() - investigation_start_time_).seconds();
            
            if (elapsed >= INVESTIGATION_DURATION) {
                RCLCPP_INFO(this->get_logger(), 
                            "✓ Investigation complete after %.1f seconds", elapsed);
                RCLCPP_INFO(this->get_logger(), 
                            "Returning to wander position (%.2f, %.2f, %.2f)",
                            wander_resume_point_.x, wander_resume_point_.y, wander_resume_point_.z);
                
                // Return to where we left off
                quadcopter_->setGoal(wander_resume_point_);
                
                // Reset detection tracking
                has_active_detection_ = false;
                
                // Return to wandering
                current_state_ = State::WANDERING;
                RCLCPP_INFO(this->get_logger(), "State: WANDERING");
            } else {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                   "Investigating... %.1f/%.1f seconds", 
                                   elapsed, INVESTIGATION_DURATION);
            }
            break;
        }
    }
}

double DecisionMaking::distanceToPoint(const geometry_msgs::msg::Point& target)
{
    auto current_pose_stamped = quadcopter_->getCurrentPose();
    const auto& pos = current_pose_stamped.pose.position;
    
    double dx = target.x - pos.x;
    double dy = target.y - pos.y;
    double dz = target.z - pos.z;
    
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}