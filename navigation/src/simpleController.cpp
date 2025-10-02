#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <chrono>

using namespace std::chrono_literals;

class SimpleDroneController : public rclcpp::Node
{
public:
    SimpleDroneController() : Node("simple_drone_controller"), phase_(Phase::TAKEOFF)
    {
        // Publisher to drone cmd_vel topic - FIXED: Changed from /model/drone to /model/scout
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/model/scout/cmd_vel", 10);
        
        // Subscribe to odometry to track position - FIXED: Changed from /model/drone to /model/scout
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/model/scout/odometry", 10,
            std::bind(&SimpleDroneController::odomCallback, this, std::placeholders::_1));
        
        // Control timer at 20Hz
        timer_ = this->create_wall_timer(50ms, std::bind(&SimpleDroneController::controlLoop, this));
        
        // Initialize variables
        start_time_ = this->now();
        takeoff_height_ = 3.0;  // Target takeoff height in meters
        forward_distance_ = 5.0; // Target forward distance in meters
        
        RCLCPP_INFO(this->get_logger(), "Simple Drone Controller started");
        RCLCPP_INFO(this->get_logger(), "Phase 1: Taking off to %.1f meters", takeoff_height_);
    }

private:
    enum class Phase {
        TAKEOFF,
        HOVER_STABILIZE,
        MOVE_FORWARD,
        HOVER_END,
        COMPLETE
    };
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pose_ = msg->pose.pose;
        
        // Track initial position on first callback
        if (!initial_pose_set_) {
            initial_pose_ = current_pose_;
            initial_pose_set_ = true;
            RCLCPP_INFO(this->get_logger(), "Initial position recorded: (%.2f, %.2f, %.2f)", 
                       initial_pose_.position.x, initial_pose_.position.y, initial_pose_.position.z);
        }
    }
    
    void controlLoop()
    {
        if (!initial_pose_set_) {
            // Wait for first odometry message
            return;
        }
        
        geometry_msgs::msg::Twist cmd;
        
        switch (phase_) {
            case Phase::TAKEOFF:
                handleTakeoff(cmd);
                break;
                
            case Phase::HOVER_STABILIZE:
                handleHoverStabilize(cmd);
                break;
                
            case Phase::MOVE_FORWARD:
                handleMoveForward(cmd);
                break;
                
            case Phase::HOVER_END:
                handleHoverEnd(cmd);
                break;
                
            case Phase::COMPLETE:
                // Mission complete, send zero velocities
                cmd.linear.x = 0.0;
                cmd.linear.y = 0.0;
                cmd.linear.z = 0.0;
                cmd.angular.z = 0.0;
                break;
        }
        
        cmd_vel_pub_->publish(cmd);
    }
    
    void handleTakeoff(geometry_msgs::msg::Twist& cmd)
    {
        double current_height = current_pose_.position.z - initial_pose_.position.z;
        double height_error = takeoff_height_ - current_height;
        
        if (std::abs(height_error) < 0.2) {
            // Close enough to target height, move to hover stabilize
            phase_ = Phase::HOVER_STABILIZE;
            phase_start_time_ = this->now();
            RCLCPP_INFO(this->get_logger(), "Takeoff complete! Current height: %.2f m", current_height);
            RCLCPP_INFO(this->get_logger(), "Phase 2: Stabilizing hover for 2 seconds");
            
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.linear.z = 0.0;
            cmd.angular.z = 0.0;
        } else {
            // Continue climbing with proportional control
            double climb_velocity = std::max(0.2, std::min(1.5, height_error * 0.8));
            
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.linear.z = climb_velocity;
            cmd.angular.z = 0.0;
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "Taking off... Height: %.2f/%.2f m", current_height, takeoff_height_);
        }
    }
    
    void handleHoverStabilize(geometry_msgs::msg::Twist& cmd)
    {
        auto elapsed = this->now() - phase_start_time_;
        
        if (elapsed.seconds() > 2.0) {
            // Stabilization complete, start moving forward
            phase_ = Phase::MOVE_FORWARD;
            forward_start_pose_ = current_pose_;
            RCLCPP_INFO(this->get_logger(), "Stabilization complete!");
            RCLCPP_INFO(this->get_logger(), "Phase 3: Moving forward %.1f meters", forward_distance_);
        }
        
        // Maintain altitude with slight upward velocity to counteract gravity
        double current_height = current_pose_.position.z - initial_pose_.position.z;
        double height_error = takeoff_height_ - current_height;
        double altitude_correction = height_error * 0.5;
        
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.linear.z = altitude_correction;
        cmd.angular.z = 0.0;
    }
    
    void handleMoveForward(geometry_msgs::msg::Twist& cmd)
    {
        double dx = current_pose_.position.x - forward_start_pose_.position.x;
        double distance_travelled = std::abs(dx);
        double distance_remaining = forward_distance_ - distance_travelled;
        
        if (distance_remaining <= 0.2) {
            // Close enough to target distance, start final hover
            phase_ = Phase::HOVER_END;
            phase_start_time_ = this->now();
            RCLCPP_INFO(this->get_logger(), "Forward movement complete! Distance: %.2f m", distance_travelled);
            RCLCPP_INFO(this->get_logger(), "Phase 4: Final hover for 3 seconds");
            
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.linear.z = 0.0;
            cmd.angular.z = 0.0;
        } else {
            // Continue moving forward with altitude maintenance
            double forward_velocity = std::max(0.3, std::min(1.0, distance_remaining * 0.5));
            
            // Maintain altitude
            double current_height = current_pose_.position.z - initial_pose_.position.z;
            double height_error = takeoff_height_ - current_height;
            double altitude_correction = height_error * 0.5;
            
            cmd.linear.x = forward_velocity;  // Forward (positive X in body frame)
            cmd.linear.y = 0.0;               // No sideways movement
            cmd.linear.z = altitude_correction; // Altitude correction
            cmd.angular.z = 0.0;              // No yaw rotation
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "Moving forward... Distance: %.2f/%.2f m", distance_travelled, forward_distance_);
        }
    }
    
    void handleHoverEnd(geometry_msgs::msg::Twist& cmd)
    {
        auto elapsed = this->now() - phase_start_time_;
        
        if (elapsed.seconds() > 3.0) {
            // Mission complete
            phase_ = Phase::COMPLETE;
            RCLCPP_INFO(this->get_logger(), "Mission complete! Drone will now hover in place.");
        }
        
        // Maintain altitude during final hover
        double current_height = current_pose_.position.z - initial_pose_.position.z;
        double height_error = takeoff_height_ - current_height;
        double altitude_correction = height_error * 0.5;
        
        cmd.linear.x = 0.0;
        cmd.linear.y = 0.0;
        cmd.linear.z = altitude_correction;
        cmd.angular.z = 0.0;
    }
    
    // ROS2 components
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // State variables
    Phase phase_;
    rclcpp::Time start_time_;
    rclcpp::Time phase_start_time_;
    
    // Pose tracking
    geometry_msgs::msg::Pose current_pose_;
    geometry_msgs::msg::Pose initial_pose_;
    geometry_msgs::msg::Pose forward_start_pose_;
    bool initial_pose_set_ = false;
    
    // Flight parameters
    double takeoff_height_;
    double forward_distance_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<SimpleDroneController>();
    
    RCLCPP_INFO(node->get_logger(), "Starting drone flight sequence...");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}