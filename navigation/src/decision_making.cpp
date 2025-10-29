#include "decision_making.hpp"
#include "quadcopter.h"
#include <cmath>

DecisionMaking::DecisionMaking(std::shared_ptr<Quadcopter> drone)
    : Node("decision_making_node"), drone_(drone)
{
    if (!drone_) {
        RCLCPP_ERROR(this->get_logger(), "Drone pointer is null!");
        throw std::invalid_argument("Invalid drone pointer");
    }

    RCLCPP_INFO(this->get_logger(), "DecisionMaking node initialized.");
    
    thermal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/thermal/detections", 10,
        std::bind(&DecisionMaking::thermalCallback, this, std::placeholders::_1)
    );
    
    this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&DecisionMaking::reachGoal, this)
    );
}

void DecisionMaking::thermalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    // Ignore detections if already processing or not searching
    if (detection_in_progress_ || mission_state_ != MissionState::SEARCHING) {
        RCLCPP_DEBUG(this->get_logger(), 
                     "Ignoring thermal detection (state=%d, in_progress=%d)",
                     static_cast<int>(mission_state_), detection_in_progress_);
        return;
    }

    // Validate detection data
    if (!std::isfinite(msg->point.x) || 
        !std::isfinite(msg->point.y) || 
        !std::isfinite(msg->point.z)) {
        RCLCPP_WARN(this->get_logger(), "Invalid thermal detection coordinates");
        return;
    }

    // Store current position and stop movement
    last_search_pose_ = drone_->getCurrentPose();
    drone_->stopMovement();
    
    // Prepare detection pose
    detection_pose_.pose.position = msg->point;
    detection_pose_.pose.orientation = last_search_pose_.pose.orientation;
    detection_pose_.header.frame_id = "map";
    detection_pose_.header.stamp = this->now();
    
    RCLCPP_INFO(this->get_logger(),
                "Thermal detection! Moving to target (x=%.2f, y=%.2f, z=%.2f)",
                msg->point.x, msg->point.y, msg->point.z);
    
    // Transition to detection state
    if (setGoalSafely(detection_pose_.pose.position)) {
        transitionToState(MissionState::DETECTION);
        detection_in_progress_ = true;
        goal_start_time_ = this->now();
        
        // Start timeout timer
        goal_timeout_timer_ = this->create_wall_timer(
            std::chrono::seconds(GOAL_TIMEOUT_SEC),
            std::bind(&DecisionMaking::handleGoalTimeout, this)
        );
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to set detection goal");
        drone_->resumeSearch();
    }
}

void DecisionMaking::reachGoal()
{
    switch (mission_state_)
    {
        case MissionState::SEARCHING:
            // Normal search mode - no goal to reach
            break;
            
        case MissionState::DETECTION:
        {
            double distance = distanceToGoal(detection_pose_.pose.position);
            if (distance < GOAL_REACH_THRESHOLD) {
                // Cancel timeout timer
                if (goal_timeout_timer_) {
                    goal_timeout_timer_->cancel();
                    goal_timeout_timer_.reset();
                }
                
                RCLCPP_INFO(this->get_logger(), 
                           "Reached detection target (%.2f m). Starting inspection...",
                           distance);
                
                drone_->stopMovement();
                transitionToState(MissionState::INSPECTING);
                
                // Start inspection timer
                inspect_timer_ = this->create_wall_timer(
                    std::chrono::seconds(INSPECTION_DURATION_SEC),
                    [this]() { this->finishInspection(); }
                );
            }
            break;
        }
        
        case MissionState::INSPECTING:
            // Waiting for inspection timer
            break;
            
        case MissionState::RETURNING:
        {
            double distance = distanceToGoal(last_search_pose_.pose.position);
            if (distance < GOAL_REACH_THRESHOLD) {
                // Cancel timeout timer
                if (goal_timeout_timer_) {
                    goal_timeout_timer_->cancel();
                    goal_timeout_timer_.reset();
                }
                
                RCLCPP_INFO(this->get_logger(), 
                           "Returned to search location (%.2f m). Resuming search.",
                           distance);
                
                transitionToState(MissionState::SEARCHING);
                detection_in_progress_ = false;
                drone_->resumeSearch();
            }
            break;
        }
    }
}

void DecisionMaking::finishInspection()
{
    // Clean up inspection timer
    if (inspect_timer_) {
        inspect_timer_->cancel();
        inspect_timer_.reset();
    }
    
    RCLCPP_INFO(this->get_logger(), "Inspection complete. Capturing image...");
    
    auto image_pose = drone_->getCurrentPose();
    RCLCPP_INFO(this->get_logger(),
                "Image captured at (x=%.2f, y=%.2f, z=%.2f)",
                image_pose.pose.position.x,
                image_pose.pose.position.y,
                image_pose.pose.position.z);
    
    // Return to last search position
    if (setGoalSafely(last_search_pose_.pose.position)) {
        transitionToState(MissionState::RETURNING);
        goal_start_time_ = this->now();
        
        // Start timeout timer for return journey
        goal_timeout_timer_ = this->create_wall_timer(
            std::chrono::seconds(GOAL_TIMEOUT_SEC),
            std::bind(&DecisionMaking::handleGoalTimeout, this)
        );
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to set return goal. Resuming search.");
        transitionToState(MissionState::SEARCHING);
        detection_in_progress_ = false;
        drone_->resumeSearch();
    }
}

void DecisionMaking::handleGoalTimeout()
{
    RCLCPP_WARN(this->get_logger(), 
                "Goal timeout in state %d. Aborting and resuming search.",
                static_cast<int>(mission_state_));
    
    // Clean up timers
    if (goal_timeout_timer_) {
        goal_timeout_timer_->cancel();
        goal_timeout_timer_.reset();
    }
    if (inspect_timer_) {
        inspect_timer_->cancel();
        inspect_timer_.reset();
    }
    
    // Reset to search state
    transitionToState(MissionState::SEARCHING);
    detection_in_progress_ = false;
    drone_->resumeSearch();
}

double DecisionMaking::distanceToGoal(const geometry_msgs::msg::Point &goal)
{
    const auto &pos = drone_->getCurrentPose().pose.position;
    double dx = goal.x - pos.x;
    double dy = goal.y - pos.y;
    double dz = goal.z - pos.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

bool DecisionMaking::setGoalSafely(const geometry_msgs::msg::Point &goal)
{
    try {
        drone_->setGoal(goal);
        return true;
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set goal: %s", e.what());
        return false;
    }
}

void DecisionMaking::transitionToState(MissionState new_state)
{
    if (mission_state_ != new_state) {
        RCLCPP_DEBUG(this->get_logger(), "State transition: %d -> %d",
                    static_cast<int>(mission_state_),
                    static_cast<int>(new_state));
        mission_state_ = new_state;
    }
}