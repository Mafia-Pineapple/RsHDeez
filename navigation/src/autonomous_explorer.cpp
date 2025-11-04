#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>
#include <cmath>
#include <random>

class AutonomousExplorer : public rclcpp::Node {
public:
    AutonomousExplorer() : Node("autonomous_explorer") {
        // Parameters
        this->declare_parameter("exploration_radius", 30.0);
        this->declare_parameter("min_goal_distance", 3.0);
        this->declare_parameter("max_goal_distance", 8.0);
        this->declare_parameter("goal_timeout", 30.0);
        this->declare_parameter("exploration_time", 180.0);
        this->declare_parameter("flight_height", 3.0);  // Lower default height
        this->declare_parameter("survey_pattern", std::string("spiral"));
        
        exploration_radius_ = this->get_parameter("exploration_radius").as_double();
        min_goal_distance_ = this->get_parameter("min_goal_distance").as_double();
        max_goal_distance_ = this->get_parameter("max_goal_distance").as_double();
        goal_timeout_ = this->get_parameter("goal_timeout").as_double();
        exploration_time_ = this->get_parameter("exploration_time").as_double();
        flight_height_ = this->get_parameter("flight_height").as_double();
        survey_pattern_ = this->get_parameter("survey_pattern").as_string();
        
        // Publishers
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/drone/goal_stamped", 10);
        
        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry", 10,
            std::bind(&AutonomousExplorer::odomCallback, this, std::placeholders::_1));
        
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&AutonomousExplorer::scanCallback, this, std::placeholders::_1));
        
        // Service client
        reach_goal_client_ = this->create_client<std_srvs::srv::SetBool>("/reach_goal");
        
        // Wait for odometry to initialize
        RCLCPP_INFO(this->get_logger(), "Waiting for odometry to capture start position...");
        
        // Timer for initialization (runs once)
        init_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&AutonomousExplorer::checkInitialization, this));
        
        RCLCPP_INFO(this->get_logger(), "Autonomous Explorer initialized");
    }

private:
    void checkInitialization() {
        if (!has_odom_) {
            return;
        }
        
        if (!initialized_) {
            // Capture starting position
            start_position_ = current_position_;
            start_time_ = this->now();
            initialized_ = true;
            
            RCLCPP_INFO(this->get_logger(), 
                       "Start position captured: (%.2f, %.2f, %.2f)",
                       start_position_.x, start_position_.y, start_position_.z);
            RCLCPP_INFO(this->get_logger(), "Survey pattern: %s", survey_pattern_.c_str());
            RCLCPP_INFO(this->get_logger(), "Exploration time: %.1f seconds", exploration_time_);
            RCLCPP_INFO(this->get_logger(), "Flight height: %.1f meters", flight_height_);
            RCLCPP_INFO(this->get_logger(), "Starting exploration in 3 seconds...");
            
            // Cancel init timer
            init_timer_->cancel();
            
            // Start exploration timer after short delay
            std::this_thread::sleep_for(std::chrono::seconds(3));
            
            timer_ = this->create_wall_timer(
                std::chrono::seconds(5),
                std::bind(&AutonomousExplorer::explorationLoop, this));
        }
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_position_ = msg->pose.pose.position;
        has_odom_ = true;
    }
    
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (!msg->ranges.empty()) {
            size_t front_start = msg->ranges.size() / 2 - 15;
            size_t front_end = msg->ranges.size() / 2 + 15;
            
            obstacle_ahead_ = false;
            for (size_t i = front_start; i < front_end && i < msg->ranges.size(); ++i) {
                if (std::isfinite(msg->ranges[i]) && msg->ranges[i] < 2.0) {
                    obstacle_ahead_ = true;
                    break;
                }
            }
        }
    }
    
    void explorationLoop() {
        if (!initialized_) {
            return;
        }
        
        // Check if exploration time is up
        auto elapsed = (this->now() - start_time_).seconds();
        if (elapsed > exploration_time_) {
            if (!exploration_complete_) {
                RCLCPP_INFO(this->get_logger(), 
                           "Exploration complete! Mapped for %.1f seconds", elapsed);
                RCLCPP_INFO(this->get_logger(), "Total goals reached: %d", goals_reached_);
                returnHome();
                exploration_complete_ = true;
                timer_->cancel();
            }
            return;
        }
        
        // Check if we've reached current goal
        if (current_goal_set_) {
            double dist = distanceToGoal();
            auto goal_elapsed = (this->now() - goal_start_time_).seconds();
            
            if (dist < 1.5) {  // Goal tolerance
                RCLCPP_INFO(this->get_logger(), "Goal reached! Distance: %.2fm", dist);
                current_goal_set_ = false;
                goals_reached_++;
                
                // Pause briefly at each goal for better scanning
                std::this_thread::sleep_for(std::chrono::seconds(2));
                
            } else if (goal_elapsed > goal_timeout_) {
                RCLCPP_WARN(this->get_logger(), 
                           "Goal timeout after %.1fs. Moving to next goal.", goal_elapsed);
                current_goal_set_ = false;
                
            } else if (obstacle_ahead_ && dist > 3.0) {
                RCLCPP_WARN(this->get_logger(), "Obstacle detected! Replanning...");
                current_goal_set_ = false;
            }
        }
        
        // Generate new goal if needed
        if (!current_goal_set_) {
            geometry_msgs::msg::Point next_goal;
            
            if (survey_pattern_ == "spiral") {
                next_goal = generateSpiralGoal();
            } else if (survey_pattern_ == "grid") {
                next_goal = generateGridGoal();
            } else {
                next_goal = generateSpiralGoal();
            }
            
            sendGoal(next_goal);
            
            double progress = (elapsed / exploration_time_) * 100.0;
            RCLCPP_INFO(this->get_logger(), 
                       "Progress: %.1f%% | Goals: %d | Time: %.0f/%.0fs",
                       progress, goals_reached_, elapsed, exploration_time_);
        }
    }
    
    geometry_msgs::msg::Point generateSpiralGoal() {
        // Spiral outward from starting position
        spiral_angle_ += 0.6;  // ~35 degrees per goal
        spiral_radius_ += 1.0; // 1 meter expansion per goal
        
        // Reset spiral if we've gone too far
        if (spiral_radius_ > exploration_radius_) {
            spiral_radius_ = min_goal_distance_;
            spiral_angle_ += M_PI / 4;  // Offset angle on reset
        }
        
        geometry_msgs::msg::Point goal;
        
        // Generate goal relative to START position
        goal.x = start_position_.x + spiral_radius_ * std::cos(spiral_angle_);
        goal.y = start_position_.y + spiral_radius_ * std::sin(spiral_angle_);
        
        // Use relative height (start height + flight offset)
        goal.z = start_position_.z + flight_height_;
        
        return goal;
    }
    
    geometry_msgs::msg::Point generateGridGoal() {
        const double grid_spacing = 6.0;
        
        if (grid_forward_) {
            grid_x_ += grid_spacing;
            if (grid_x_ > exploration_radius_) {
                grid_x_ = -exploration_radius_;
                grid_y_ += grid_spacing;
                grid_forward_ = false;
            }
        } else {
            grid_x_ -= grid_spacing;
            if (grid_x_ < -exploration_radius_) {
                grid_x_ = exploration_radius_;
                grid_y_ += grid_spacing;
                grid_forward_ = true;
            }
        }
        
        if (grid_y_ > exploration_radius_) {
            grid_y_ = -exploration_radius_;
        }
        
        geometry_msgs::msg::Point goal;
        goal.x = start_position_.x + grid_x_;
        goal.y = start_position_.y + grid_y_;
        goal.z = start_position_.z + flight_height_;
        
        return goal;
    }
    
    void sendGoal(const geometry_msgs::msg::Point& goal) {
        geometry_msgs::msg::PointStamped goal_msg;
        goal_msg.header.stamp = this->now();
        goal_msg.header.frame_id = "map";
        goal_msg.point = goal;
        
        goal_pub_->publish(goal_msg);
        
        current_goal_ = goal;
        current_goal_set_ = true;
        goal_start_time_ = this->now();
        
        // Enable autonomous flight
        if (!flight_enabled_) {
            auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
            request->data = true;
            
            if (reach_goal_client_->wait_for_service(std::chrono::seconds(1))) {
                reach_goal_client_->async_send_request(request);
                flight_enabled_ = true;
            }
        }
        
        double dist = std::sqrt(
            std::pow(goal.x - current_position_.x, 2) +
            std::pow(goal.y - current_position_.y, 2)
        );
        
        RCLCPP_INFO(this->get_logger(), 
                   "→ Goal: (%.1f, %.1f, %.1f) | Dist: %.1fm | From start: (%.1f, %.1f)",
                   goal.x, goal.y, goal.z, dist, 
                   goal.x - start_position_.x, goal.y - start_position_.y);
    }
    
    void returnHome() {
        geometry_msgs::msg::Point home = start_position_;
        home.z = start_position_.z + flight_height_;
        
        current_goal_set_ = false;  // Reset goal flag
        sendGoal(home);
        
        RCLCPP_INFO(this->get_logger(), "🏠 Returning home: (%.2f, %.2f, %.2f)",
                   home.x, home.y, home.z);
    }
    
    double distanceToGoal() {
        return std::sqrt(
            std::pow(current_goal_.x - current_position_.x, 2) +
            std::pow(current_goal_.y - current_position_.y, 2) +
            std::pow(current_goal_.z - current_position_.z, 2)
        );
    }
    
    // ROS components
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr reach_goal_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr init_timer_;
    
    // State
    geometry_msgs::msg::Point current_position_;
    geometry_msgs::msg::Point start_position_;
    geometry_msgs::msg::Point current_goal_;
    
    bool has_odom_ = false;
    bool initialized_ = false;
    bool current_goal_set_ = false;
    bool flight_enabled_ = false;
    bool exploration_complete_ = false;
    bool obstacle_ahead_ = false;
    bool grid_forward_ = true;
    
    int goals_reached_ = 0;
    
    rclcpp::Time start_time_;
    rclcpp::Time goal_start_time_;
    
    // Parameters
    double exploration_radius_;
    double min_goal_distance_;
    double max_goal_distance_;
    double goal_timeout_;
    double exploration_time_;
    double flight_height_;
    std::string survey_pattern_;
    
    // Pattern state
    double spiral_angle_ = 0.0;
    double spiral_radius_ = 3.0;
    double grid_x_ = 0.0;
    double grid_y_ = 0.0;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutonomousExplorer>());
    rclcpp::shutdown();
    return 0;
}