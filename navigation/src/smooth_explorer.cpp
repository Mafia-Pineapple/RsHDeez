#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>
#include <vector>
#include <algorithm>

class SmoothExplorer : public rclcpp::Node {
public:
    SmoothExplorer() : Node("smooth_explorer") {
        // Parameters
        this->declare_parameter("exploration_radius", 25.0);
        this->declare_parameter("exploration_time", 180.0);
        this->declare_parameter("cruise_speed", 0.8);
        this->declare_parameter("cruise_height", 5.0);
        this->declare_parameter("survey_pattern", std::string("spiral"));
        this->declare_parameter("angular_speed", 0.3);
        this->declare_parameter("obstacle_distance", 3.0);      // Stop distance
        this->declare_parameter("safe_distance", 5.0);          // Slow down distance
        this->declare_parameter("avoidance_gain", 1.5);         // How aggressively to avoid
        
        exploration_radius_ = this->get_parameter("exploration_radius").as_double();
        exploration_time_ = this->get_parameter("exploration_time").as_double();
        cruise_speed_ = this->get_parameter("cruise_speed").as_double();
        cruise_height_ = this->get_parameter("cruise_height").as_double();
        survey_pattern_ = this->get_parameter("survey_pattern").as_string();
        angular_speed_ = this->get_parameter("angular_speed").as_double();
        obstacle_distance_ = this->get_parameter("obstacle_distance").as_double();
        safe_distance_ = this->get_parameter("safe_distance").as_double();
        avoidance_gain_ = this->get_parameter("avoidance_gain").as_double();
        
        // Publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry", 10,
            std::bind(&SmoothExplorer::odomCallback, this, std::placeholders::_1));
        
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&SmoothExplorer::scanCallback, this, std::placeholders::_1));
        
        // Control loop at 20Hz for smooth motion
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&SmoothExplorer::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "Smooth Explorer with Collision Avoidance");
        RCLCPP_INFO(this->get_logger(), "Obstacle stop distance: %.1fm", obstacle_distance_);
        RCLCPP_INFO(this->get_logger(), "Safe distance: %.1fm", safe_distance_);
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_position_ = msg->pose.pose.position;
        
        // Extract yaw from quaternion
        auto q = msg->pose.pose.orientation;
        current_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        
        if (!has_odom_) {
            start_position_ = current_position_;
            start_time_ = this->now();
            has_odom_ = true;
            
            RCLCPP_INFO(this->get_logger(), 
                       "Start: (%.2f, %.2f, %.2f) | Pattern: %s | Speed: %.2f m/s",
                       start_position_.x, start_position_.y, start_position_.z,
                       survey_pattern_.c_str(), cruise_speed_);
        }
    }
    
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (msg->ranges.empty()) return;
        
        size_t n = msg->ranges.size();
        
        // Divide 360° into 8 sectors for detailed obstacle awareness
        sectors_.clear();
        sectors_.resize(8, std::numeric_limits<double>::infinity());
        
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            if (!std::isfinite(msg->ranges[i])) continue;
            
            // Determine which sector this ray belongs to (0=front, 1=front-left, etc.)
            int sector = (i * 8) / n;
            
            // Keep the minimum distance in each sector
            sectors_[sector] = std::min(sectors_[sector], static_cast<double>(msg->ranges[i]));
        }
        
        // Key directions
        front_distance_ = sectors_[0];                          // 0° (front)
        front_left_distance_ = sectors_[1];                     // 45° (front-left)
        left_distance_ = sectors_[2];                           // 90° (left)
        back_left_distance_ = sectors_[3];                      // 135°
        back_distance_ = sectors_[4];                           // 180° (back)
        back_right_distance_ = sectors_[5];                     // 225°
        right_distance_ = sectors_[6];                          // 270° (right)
        front_right_distance_ = sectors_[7];                    // 315° (front-right)
        
        // Find closest obstacle overall
        closest_obstacle_distance_ = *std::min_element(sectors_.begin(), sectors_.end());
        
        // Determine if we're in danger
        obstacle_critical_ = closest_obstacle_distance_ < obstacle_distance_;
        obstacle_warning_ = closest_obstacle_distance_ < safe_distance_;
    }
    
    void controlLoop() {
        if (!has_odom_) return;
        
        auto elapsed = (this->now() - start_time_).seconds();
        
        // Check if exploration complete
        if (elapsed > exploration_time_) {
            if (!returning_home_) {
                RCLCPP_INFO(this->get_logger(), "Exploration complete! Returning home...");
                returning_home_ = true;
            }
            returnHome();
            return;
        }
        
        double progress = elapsed / exploration_time_;
        
        // Get desired velocity from pattern
        geometry_msgs::msg::Twist cmd;
        
        if (survey_pattern_ == "spiral") {
            cmd = spiralPattern(progress);
        } else if (survey_pattern_ == "figure8") {
            cmd = figure8Pattern(progress);
        } else if (survey_pattern_ == "circle") {
            cmd = circlePattern(progress);
        } else {
            cmd = spiralPattern(progress);
        }
        
        // Apply collision avoidance (CRITICAL!)
        cmd = applyCollisionAvoidance(cmd);
        
        // Altitude control
        double current_height = current_position_.z - start_position_.z;
        double height_error = cruise_height_ - current_height;
        cmd.linear.z = std::max(-0.5, std::min(0.5, height_error * 0.3));
        
        // Boundary enforcement
        double dist_from_start = std::sqrt(
            std::pow(current_position_.x - start_position_.x, 2) +
            std::pow(current_position_.y - start_position_.y, 2)
        );
        
        if (dist_from_start > exploration_radius_ * 0.95) {
            cmd = returnTowardsStart(cmd, dist_from_start);
        }
        
        // Publish command
        cmd_vel_pub_->publish(cmd);
        
        // Logging
        if (static_cast<int>(elapsed) % 10 == 0 && elapsed - last_log_time_ > 9.0) {
            RCLCPP_INFO(this->get_logger(),
                       "Progress: %.0f%% | Dist: %.1fm | Height: %.1fm | Closest obstacle: %.1fm",
                       progress * 100, dist_from_start, current_height, closest_obstacle_distance_);
            last_log_time_ = elapsed;
        }
    }
    
    geometry_msgs::msg::Twist applyCollisionAvoidance(geometry_msgs::msg::Twist desired_cmd) {
        geometry_msgs::msg::Twist safe_cmd = desired_cmd;
        
        // CRITICAL STOP - Very close obstacle
        if (obstacle_critical_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "CRITICAL: Obstacle at %.2fm! Emergency stop.", closest_obstacle_distance_);
            
            safe_cmd.linear.x = 0.0;
            safe_cmd.linear.y = 0.0;
            
            // Turn away from obstacle
            safe_cmd.angular.z = calculateEscapeDirection();
            
            return safe_cmd;
        }
        
        // WARNING - Obstacle nearby, slow down and adjust
        if (obstacle_warning_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                               "Warning: Obstacle at %.2fm. Adjusting course.", closest_obstacle_distance_);
            
            // Scale down speed based on distance
            double speed_factor = (closest_obstacle_distance_ - obstacle_distance_) / 
                                 (safe_distance_ - obstacle_distance_);
            speed_factor = std::max(0.2, std::min(1.0, speed_factor));
            
            safe_cmd.linear.x *= speed_factor;
            safe_cmd.linear.y *= speed_factor;
            
            // Add avoidance component
            double avoidance_turn = calculateAvoidanceDirection();
            safe_cmd.angular.z += avoidance_turn;
        }
        
        // Specific obstacle ahead - redirect
        if (front_distance_ < safe_distance_) {
            double obstacle_factor = 1.0 - (front_distance_ / safe_distance_);
            
            // Slow forward motion
            safe_cmd.linear.x *= (1.0 - obstacle_factor * 0.8);
            
            // Choose better direction: left or right?
            if (front_left_distance_ > front_right_distance_) {
                // Go left
                safe_cmd.angular.z += angular_speed_ * obstacle_factor * avoidance_gain_;
                safe_cmd.linear.y += cruise_speed_ * 0.3 * obstacle_factor;
            } else {
                // Go right
                safe_cmd.angular.z -= angular_speed_ * obstacle_factor * avoidance_gain_;
                safe_cmd.linear.y -= cruise_speed_ * 0.3 * obstacle_factor;
            }
            
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Obstacle ahead: %.2fm. Turning %s", 
                                front_distance_,
                                (front_left_distance_ > front_right_distance_) ? "LEFT" : "RIGHT");
        }
        
        // Clamp velocities to safe limits
        safe_cmd.linear.x = std::max(-cruise_speed_, std::min(cruise_speed_, safe_cmd.linear.x));
        safe_cmd.linear.y = std::max(-cruise_speed_ * 0.5, std::min(cruise_speed_ * 0.5, safe_cmd.linear.y));
        safe_cmd.angular.z = std::max(-angular_speed_ * 2.0, std::min(angular_speed_ * 2.0, safe_cmd.angular.z));
        
        return safe_cmd;
    }
    
    double calculateEscapeDirection() {
        // Find the direction with most free space
        double max_distance = 0;
        int best_sector = 4;  // Default: turn around
        
        for (size_t i = 0; i < sectors_.size(); ++i) {
            if (sectors_[i] > max_distance) {
                max_distance = sectors_[i];
                best_sector = i;
            }
        }
        
        // Convert sector to angular velocity
        // Sector 0 = front, 2 = left, 4 = back, 6 = right
        int turn_direction = 0;
        if (best_sector == 1 || best_sector == 2) turn_direction = 1;      // Turn left
        else if (best_sector == 6 || best_sector == 7) turn_direction = -1; // Turn right
        else if (best_sector >= 3 && best_sector <= 5) turn_direction = 1;  // Turn around (pick a side)
        
        return turn_direction * angular_speed_ * 1.5;
    }
    
    double calculateAvoidanceDirection() {
        // Calculate a weighted turn based on obstacle distribution
        double turn_bias = 0.0;
        
        // More space on left → turn left (positive)
        // More space on right → turn right (negative)
        double left_space = (left_distance_ + front_left_distance_ + back_left_distance_) / 3.0;
        double right_space = (right_distance_ + front_right_distance_ + back_right_distance_) / 3.0;
        
        if (left_space > right_space) {
            turn_bias = angular_speed_ * 0.5 * (left_space / (left_space + right_space));
        } else {
            turn_bias = -angular_speed_ * 0.5 * (right_space / (left_space + right_space));
        }
        
        return turn_bias;
    }
    
    geometry_msgs::msg::Twist returnTowardsStart(geometry_msgs::msg::Twist cmd, double dist) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                           "Near boundary (%.1fm). Turning toward start.", dist);
        
        // Calculate angle to start
        double dx = start_position_.x - current_position_.x;
        double dy = start_position_.y - current_position_.y;
        double angle_to_start = std::atan2(dy, dx);
        
        // Angle difference
        double angle_diff = angle_to_start - current_yaw_;
        while (angle_diff > M_PI) angle_diff -= 2*M_PI;
        while (angle_diff < -M_PI) angle_diff += 2*M_PI;
        
        // Override command to head inward
        cmd.linear.x = cruise_speed_ * 0.7;
        cmd.angular.z = std::max(-angular_speed_, std::min(angular_speed_, angle_diff * 0.8));
        
        return cmd;
    }
    
    geometry_msgs::msg::Twist spiralPattern(double progress) {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = cruise_speed_ * (0.7 + 0.3 * progress);
        cmd.linear.y = 0.0;
        cmd.angular.z = angular_speed_ * (1.0 - 0.7 * progress);
        return cmd;
    }
    
    geometry_msgs::msg::Twist figure8Pattern(double progress) {
        geometry_msgs::msg::Twist cmd;
        double t = progress * 2 * M_PI * 2;
        cmd.linear.x = cruise_speed_ * std::cos(t);
        cmd.linear.y = cruise_speed_ * std::sin(2 * t) * 0.5;
        cmd.angular.z = angular_speed_ * std::cos(t);
        return cmd;
    }
    
    geometry_msgs::msg::Twist circlePattern(double progress) {
        geometry_msgs::msg::Twist cmd;
        double radius = 5.0 + progress * (exploration_radius_ - 5.0);
        double angular_vel = cruise_speed_ / radius;
        cmd.linear.x = cruise_speed_;
        cmd.linear.y = 0.0;
        cmd.angular.z = angular_vel;
        return cmd;
    }
    
    void returnHome() {
        geometry_msgs::msg::Twist cmd;
        
        double dx = start_position_.x - current_position_.x;
        double dy = start_position_.y - current_position_.y;
        double dz = start_position_.z - current_position_.z;
        double dist = std::sqrt(dx*dx + dy*dy);
        
        if (dist < 1.0) {
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.linear.z = 0.0;
            cmd.angular.z = 0.0;
            
            if (!home_reached_) {
                RCLCPP_INFO(this->get_logger(), "✓ Home reached! Mission complete.");
                home_reached_ = true;
                timer_->cancel();
            }
        } else {
            double angle_to_home = std::atan2(dy, dx);
            double angle_diff = angle_to_home - current_yaw_;
            while (angle_diff > M_PI) angle_diff -= 2*M_PI;
            while (angle_diff < -M_PI) angle_diff += 2*M_PI;
            
            cmd.linear.x = std::min(cruise_speed_ * 0.8, dist * 0.5);
            cmd.angular.z = std::max(-angular_speed_, std::min(angular_speed_, angle_diff));
            cmd.linear.z = std::max(-0.5, std::min(0.5, dz * 0.3));
            
            // Still apply collision avoidance on way home!
            cmd = applyCollisionAvoidance(cmd);
        }
        
        cmd_vel_pub_->publish(cmd);
    }
    
    // ROS components
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // State
    geometry_msgs::msg::Point current_position_;
    geometry_msgs::msg::Point start_position_;
    double current_yaw_ = 0.0;
    
    bool has_odom_ = false;
    bool returning_home_ = false;
    bool home_reached_ = false;
    bool obstacle_critical_ = false;
    bool obstacle_warning_ = false;
    
    rclcpp::Time start_time_;
    double last_log_time_ = 0.0;
    
    // Obstacle detection
    std::vector<double> sectors_;
    double front_distance_ = 999.0;
    double front_left_distance_ = 999.0;
    double left_distance_ = 999.0;
    double back_left_distance_ = 999.0;
    double back_distance_ = 999.0;
    double back_right_distance_ = 999.0;
    double right_distance_ = 999.0;
    double front_right_distance_ = 999.0;
    double closest_obstacle_distance_ = 999.0;
    
    // Parameters
    double exploration_radius_;
    double exploration_time_;
    double cruise_speed_;
    double cruise_height_;
    double angular_speed_;
    double obstacle_distance_;
    double safe_distance_;
    double avoidance_gain_;
    std::string survey_pattern_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SmoothExplorer>());
    rclcpp::shutdown();
    return 0;
}