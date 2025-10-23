#include "decision_making.hpp"
#include "quadcopter.h"
#include <cmath>

DecisionMaking::DecisionMaking(std::shared_ptr<Quadcopter> drone)
: Node("decision_making_node"), drone_(drone)
{
    RCLCPP_INFO(this->get_logger(), "DecisionMaking node initialized.");

    thermal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/thermal/detections", 10,
        std::bind(&DecisionMaking::thermalCallback, this, std::placeholders::_1)
    );

    this->create_wall_timer(std::chrono::milliseconds(100),
                            std::bind(&DecisionMaking::reachGoal, this));
}

void DecisionMaking::thermalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    if (detection_in_progress_ || mission_state_ != MissionState::SEARCHING)
        return;

    last_search_pose_ = drone_->getCurrentPose();
    drone_->stopMovement();

    detection_in_progress_ = true;
    mission_state_ = MissionState::DETECTION;

    detection_pose_.pose.position = msg->point;
    detection_pose_.pose.orientation = last_search_pose_.pose.orientation;
    detection_pose_.header.frame_id = "map";

    RCLCPP_INFO(this->get_logger(),
                "Moving to detection target (x=%.2f, y=%.2f, z=%.2f)...",
                msg->point.x, msg->point.y, msg->point.z);

    drone_->setGoal(detection_pose_.pose.position);
}

void DecisionMaking::reachGoal()
{
    switch (mission_state_)
    {
        case MissionState::SEARCHING:
            break;

        case MissionState::DETECTION:
            if (distanceToGoal(detection_pose_.pose.position) < 1.0)
            {
                RCLCPP_INFO(this->get_logger(), "Reached detection target. Starting inspection...");
                mission_state_ = MissionState::INSPECTING;
                drone_->stopMovement();

                inspect_timer_ = this->create_wall_timer(
                    std::chrono::seconds(3),
                    std::bind(&DecisionMaking::finishInspection, this)
                );
            }
            break;

        case MissionState::INSPECTING:
            break;

        case MissionState::RETURNING:
            if (distanceToGoal(last_search_pose_.pose.position) < 1.0)
            {
                RCLCPP_INFO(this->get_logger(), "Returned to search location. Resuming search.");
                mission_state_ = MissionState::SEARCHING;
                detection_in_progress_ = false;
                drone_->resumeSearch();
            }
            break;
    }
}

void DecisionMaking::finishInspection()
{
    inspect_timer_->cancel();
    RCLCPP_INFO(this->get_logger(), "Inspection complete. Capturing image...");

    auto image_pose = drone_->getCurrentPose();
    RCLCPP_INFO(this->get_logger(),
                "Image captured at approx (x=%.2f, y=%.2f, z=%.2f)",
                image_pose.pose.position.x,
                image_pose.pose.position.y,
                image_pose.pose.position.z);

    drone_->setGoal(last_search_pose_.pose.position);
    mission_state_ = MissionState::RETURNING;
}

double DecisionMaking::distanceToGoal(const geometry_msgs::msg::Point &goal)
{
    const auto &pos = drone_->getCurrentPose().pose.position;
    double dx = goal.x - pos.x;
    double dy = goal.y - pos.y;
    double dz = goal.z - pos.z;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}
