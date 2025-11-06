#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cmath>
#include <vector>
#include <set>
#include <algorithm>

// Grid cell states
enum CellState {
    UNVISITED,
    VISITED,
    UNREACHABLE
};

// 2D grid position
struct GridCell {
    int x;
    int y;
    
    bool operator<(const GridCell& other) const {
        if (x != other.x) return x < other.x;
        return y < other.y;
    }
    
    bool operator==(const GridCell& other) const {
        return x == other.x && y == other.y;
    }
};

class GridExplorer : public rclcpp::Node {
public:
    GridExplorer() : Node("grid_explorer") {
        // Parameters
        this->declare_parameter("grid_spacing", 15.0);           // 2m between grid points
        this->declare_parameter("grid_radius", 2);             // 25 cells (50m) in each direction
        this->declare_parameter("cruise_speed", 1.0);           // 1.5 m/s
        this->declare_parameter("goal_tolerance", 2.0);         // 0.5m to consider goal reached
        this->declare_parameter("obstacle_distance", 4.0);      // 1.5m obstacle detection range
        this->declare_parameter("max_deviation", 5.0);          // 5m max lateral deviation when avoiding
        this->declare_parameter("target_agl", 15.0);            // 10m above ground
        this->declare_parameter("blocked_timeout", 30.0);       // 10s to mark as unreachable
        this->declare_parameter("angular_speed", 0.5);          // Turn rate
        
        grid_spacing_ = this->get_parameter("grid_spacing").as_double();
        grid_radius_ = this->get_parameter("grid_radius").as_int();
        cruise_speed_ = this->get_parameter("cruise_speed").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
        obstacle_distance_ = this->get_parameter("obstacle_distance").as_double();
        max_deviation_ = this->get_parameter("max_deviation").as_double();
        target_agl_ = this->get_parameter("target_agl").as_double();
        blocked_timeout_ = this->get_parameter("blocked_timeout").as_double();
        angular_speed_ = this->get_parameter("angular_speed").as_double();
        
        // Publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry", 10,
            std::bind(&GridExplorer::odomCallback, this, std::placeholders::_1));
        
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&GridExplorer::scanCallback, this, std::placeholders::_1));
        
        agl_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/drone/agl_distance", 10,
            std::bind(&GridExplorer::aglCallback, this, std::placeholders::_1));
        
        // Control loop at 20Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&GridExplorer::controlLoop, this));
        
        RCLCPP_INFO(this->get_logger(), "=== Grid Explorer Started ===");
        RCLCPP_INFO(this->get_logger(), "Grid: %dx%d cells, %.1fm spacing", 
                    2*grid_radius_+1, 2*grid_radius_+1, grid_spacing_);
        RCLCPP_INFO(this->get_logger(), "Coverage area: %.1fm x %.1fm", 
                    2*grid_radius_*grid_spacing_, 2*grid_radius_*grid_spacing_);
        RCLCPP_INFO(this->get_logger(), "Speed: %.1f m/s | AGL: %.1fm | Obstacle range: %.1fm",
                    cruise_speed_, target_agl_, obstacle_distance_);
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_position_ = msg->pose.pose.position;
        
        // Extract yaw from quaternion
        auto q = msg->pose.pose.orientation;
        current_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        
        if (!has_odom_) {
            home_position_ = current_position_;
            has_odom_ = true;
            
            RCLCPP_INFO(this->get_logger(), "Home base: (%.2f, %.2f, %.2f)", 
                       home_position_.x, home_position_.y, home_position_.z);
            
            // Initialize grid exploration - start at center (0,0)
            generateSquareSpiral();
            
            RCLCPP_INFO(this->get_logger(), "Generated %zu grid points to explore", 
                       spiral_path_.size());
        }
    }
    
    void aglCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        current_agl_ = msg->data;
    }
    
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (msg->ranges.empty()) return;
        
        size_t n = msg->ranges.size();
        
        // Divide 360° into 8 sectors
        sectors_.clear();
        sectors_.resize(8, std::numeric_limits<double>::infinity());
        
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            if (!std::isfinite(msg->ranges[i])) continue;
            
            int sector = (i * 8) / n;
            sectors_[sector] = std::min(sectors_[sector], static_cast<double>(msg->ranges[i]));
        }
        
        // Key directions (same as smooth_explorer)
        front_distance_ = sectors_[0];
        front_left_distance_ = sectors_[1];
        left_distance_ = sectors_[2];
        back_left_distance_ = sectors_[3];
        back_distance_ = sectors_[4];
        back_right_distance_ = sectors_[5];
        right_distance_ = sectors_[6];
        front_right_distance_ = sectors_[7];
        
        closest_obstacle_distance_ = *std::min_element(sectors_.begin(), sectors_.end());
    }
    
    void generateSquareSpiral() {
        // Generate square spiral pattern from center outward
        // Pattern: (0,0) -> (1,0) -> (1,1) -> (0,1) -> (-1,1) -> (-1,0) -> (-1,-1) -> (0,-1) -> (1,-1) -> (2,-1) ...
        
        spiral_path_.clear();
        
        // Start at center
        spiral_path_.push_back({0, 0});
        
        int x = 0, y = 0;
        int dx = 1, dy = 0;  // Start moving right
        int segment_length = 1;
        int segment_passed = 0;
        
        // Generate spiral covering grid_radius in all directions
        for (int total = 1; total < (2 * grid_radius_ + 1) * (2 * grid_radius_ + 1); ++total) {
            x += dx;
            y += dy;
            
            // Check if within bounds
            if (std::abs(x) <= grid_radius_ && std::abs(y) <= grid_radius_) {
                spiral_path_.push_back({x, y});
            }
            
            segment_passed++;
            
            if (segment_passed == segment_length) {
                segment_passed = 0;
                
                // Turn 90 degrees left
                int temp = dx;
                dx = -dy;
                dy = temp;
                
                // Increase segment length every 2 turns
                if (dy == 0) {
                    segment_length++;
                }
            }
        }
    }
    
    void controlLoop() {
        if (!has_odom_) return;
        
        geometry_msgs::msg::Twist cmd;
        
        // State machine
        if (returning_home_) {
            returnHome();
            return;
        }
        
        // Check if all points explored
        if (current_path_index_ >= spiral_path_.size()) {
            if (!returning_home_) {
                RCLCPP_INFO(this->get_logger(), 
                           "=== Exploration Complete ===");
                RCLCPP_INFO(this->get_logger(), 
                           "Visited: %zu | Unreachable: %zu | Total: %zu",
                           visited_cells_.size(), unreachable_cells_.size(), 
                           spiral_path_.size());
                returning_home_ = true;
            }
            returnHome();
            return;
        }
        
        // Get current target
        GridCell target = spiral_path_[current_path_index_];
        
        // Skip if already visited or unreachable
        if (visited_cells_.count(target) > 0 || unreachable_cells_.count(target) > 0) {
            current_path_index_++;
            return;
        }
        
        // Convert grid cell to world coordinates (relative to home)
        double target_x = home_position_.x + target.x * grid_spacing_;
        double target_y = home_position_.y + target.y * grid_spacing_;
        
        // Calculate distance to target
        double dx = target_x - current_position_.x;
        double dy = target_y - current_position_.y;
        double distance = std::sqrt(dx*dx + dy*dy);
        
        // Check if goal reached
        if (distance < goal_tolerance_) {
            visited_cells_.insert(target);
            current_path_index_++;
            
            RCLCPP_INFO(this->get_logger(), 
                       "✓ Visited grid [%d, %d] | Progress: %zu/%zu (%.0f%%)",
                       target.x, target.y, 
                       visited_cells_.size() + unreachable_cells_.size(),
                       spiral_path_.size(),
                       100.0 * (visited_cells_.size() + unreachable_cells_.size()) / spiral_path_.size());
            
            // Reset blocked timer
            time_at_goal_attempt_ = this->now();
            stuck_counter_ = 0;
            
            return;
        }
        
        // Check if we've been trying to reach this goal for too long
        if (!attempting_goal_) {
            attempting_goal_ = true;
            time_at_goal_attempt_ = this->now();
            last_progress_position_ = current_position_;
            stuck_counter_ = 0;
        }
        
        double time_attempting = (this->now() - time_at_goal_attempt_).seconds();
        
        // Check if we're making progress
        double progress_dx = current_position_.x - last_progress_position_.x;
        double progress_dy = current_position_.y - last_progress_position_.y;
        double progress_dist = std::sqrt(progress_dx*progress_dx + progress_dy*progress_dy);
        
        if (progress_dist < 0.1) {  // Less than 10cm progress
            stuck_counter_++;
        } else {
            stuck_counter_ = 0;
            last_progress_position_ = current_position_;
        }
        
        // Mark as unreachable if stuck for too long
        if (time_attempting > blocked_timeout_ || stuck_counter_ > 40) {  // 40 * 50ms = 2 seconds stuck
            unreachable_cells_.insert(target);
            current_path_index_++;
            attempting_goal_ = false;
            
            RCLCPP_WARN(this->get_logger(), 
                       "✗ Grid [%d, %d] marked UNREACHABLE (stuck for %.1fs)",
                       target.x, target.y, time_attempting);
            
            return;
        }
        
        // Calculate desired heading
        double angle_to_target = std::atan2(dy, dx);
        double angle_error = angle_to_target - current_yaw_;
        
        // Normalize angle to [-pi, pi]
        while (angle_error > M_PI) angle_error -= 2*M_PI;
        while (angle_error < -M_PI) angle_error += 2*M_PI;
        
        // Basic navigation command
        cmd.linear.x = cruise_speed_ * std::cos(angle_error);
        cmd.linear.y = cruise_speed_ * std::sin(angle_error);
        cmd.angular.z = std::max(-angular_speed_, std::min(angular_speed_, angle_error * 0.8));
        
        // Apply collision avoidance (CRITICAL!)
        cmd = applyCollisionAvoidance(cmd);
        
        // AGL altitude control
        double agl_error = target_agl_ - current_agl_;
        cmd.linear.z = std::max(-0.8, std::min(0.8, agl_error * 0.4));
        
        // Publish command
        cmd_vel_pub_->publish(cmd);
        
        // Periodic logging
        if (static_cast<int>(time_attempting) % 3 == 0 && 
            time_attempting - last_log_time_ > 2.5) {
            RCLCPP_INFO(this->get_logger(),
                       "→ Grid [%d, %d] | Dist: %.1fm | AGL: %.1fm | Obstacle: %.1fm",
                       target.x, target.y, distance, current_agl_, closest_obstacle_distance_);
            last_log_time_ = time_attempting;
        }
    }
    
    geometry_msgs::msg::Twist applyCollisionAvoidance(geometry_msgs::msg::Twist desired_cmd) {
        geometry_msgs::msg::Twist safe_cmd = desired_cmd;
        
        // Check for obstacles within detection range
        bool obstacle_detected = closest_obstacle_distance_ < obstacle_distance_;
        
        if (!obstacle_detected) {
            return desired_cmd;  // Clear path, proceed normally
        }
        
        // OBSTACLE DETECTED - Navigate around while maintaining 1m minimum distance
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                           "Obstacle at %.2fm! Maneuvering around...", closest_obstacle_distance_);
        
        // Reduce forward speed when close to obstacles
        double obstacle_factor = 1.0 - (obstacle_distance_ - closest_obstacle_distance_) / obstacle_distance_;
        obstacle_factor = std::max(0.0, std::min(1.0, obstacle_factor));
        
        // Scale down forward motion
        safe_cmd.linear.x *= (1.0 - obstacle_factor * 0.7);
        safe_cmd.linear.y *= (1.0 - obstacle_factor * 0.7);
        
        // Determine best direction to avoid
        // Find sector with most space
        double max_distance = 0;
        int best_sector = 0;
        
        for (size_t i = 0; i < sectors_.size(); ++i) {
            if (sectors_[i] > max_distance) {
                max_distance = sectors_[i];
                best_sector = i;
            }
        }
        
        // Apply avoidance maneuver based on obstacle location
        if (front_distance_ < obstacle_distance_) {
            // Obstacle in front - turn toward open space
            if (front_left_distance_ > front_right_distance_) {
                // More space on left - turn left
                safe_cmd.angular.z += angular_speed_ * obstacle_factor * 1.5;
                safe_cmd.linear.y += cruise_speed_ * 0.3 * obstacle_factor;
            } else {
                // More space on right - turn right
                safe_cmd.angular.z -= angular_speed_ * obstacle_factor * 1.5;
                safe_cmd.linear.y -= cruise_speed_ * 0.3 * obstacle_factor;
            }
        }
        
        // Additional lateral avoidance for side obstacles
        if (left_distance_ < obstacle_distance_) {
            safe_cmd.linear.y -= cruise_speed_ * 0.4 * obstacle_factor;  // Move right
        }
        if (right_distance_ < obstacle_distance_) {
            safe_cmd.linear.y += cruise_speed_ * 0.4 * obstacle_factor;  // Move left
        }
        
        // Very close obstacle (< 1m) - stronger reaction
        if (closest_obstacle_distance_ < 1.0) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                                "VERY CLOSE obstacle at %.2fm! Emergency maneuver!", 
                                closest_obstacle_distance_);
            
            // Boost avoidance
            safe_cmd.angular.z *= 2.0;
            safe_cmd.linear.y *= 1.5;
            
            // Slow down significantly
            safe_cmd.linear.x *= 0.3;
        }
        
        // Clamp velocities
        safe_cmd.linear.x = std::max(-cruise_speed_, std::min(cruise_speed_, safe_cmd.linear.x));
        safe_cmd.linear.y = std::max(-cruise_speed_ * 0.6, std::min(cruise_speed_ * 0.6, safe_cmd.linear.y));
        safe_cmd.angular.z = std::max(-angular_speed_ * 2.0, std::min(angular_speed_ * 2.0, safe_cmd.angular.z));
        
        return safe_cmd;
    }
    
    void returnHome() {
        geometry_msgs::msg::Twist cmd;
        
        double dx = home_position_.x - current_position_.x;
        double dy = home_position_.y - current_position_.y;
        double dz = home_position_.z - current_position_.z;
        double dist = std::sqrt(dx*dx + dy*dy);
        
        if (dist < 1.0) {
            // Home reached - stop
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.linear.z = 0.0;
            cmd.angular.z = 0.0;
            
            if (!home_reached_) {
                RCLCPP_INFO(this->get_logger(), "=== HOME REACHED ===");
                RCLCPP_INFO(this->get_logger(), "Mission complete!");
                RCLCPP_INFO(this->get_logger(), "Total visited: %zu | Unreachable: %zu | Coverage: %.1f%%",
                           visited_cells_.size(), unreachable_cells_.size(),
                           100.0 * visited_cells_.size() / spiral_path_.size());
                home_reached_ = true;
                timer_->cancel();
            }
        } else {
            // Navigate home
            double angle_to_home = std::atan2(dy, dx);
            double angle_diff = angle_to_home - current_yaw_;
            
            while (angle_diff > M_PI) angle_diff -= 2*M_PI;
            while (angle_diff < -M_PI) angle_diff += 2*M_PI;
            
            cmd.linear.x = std::min(cruise_speed_ * 0.8, dist * 0.5);
            cmd.angular.z = std::max(-angular_speed_, std::min(angular_speed_, angle_diff));
            cmd.linear.z = std::max(-0.5, std::min(0.5, dz * 0.3));
            
            // Still apply collision avoidance on way home
            cmd = applyCollisionAvoidance(cmd);
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                               "Returning home... %.1fm remaining", dist);
        }
        
        cmd_vel_pub_->publish(cmd);
    }
    
    // ROS components
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr agl_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // State
    geometry_msgs::msg::Point current_position_;
    geometry_msgs::msg::Point home_position_;
    geometry_msgs::msg::Point last_progress_position_;
    double current_yaw_ = 0.0;
    double current_agl_ = 0.0;
    
    bool has_odom_ = false;
    bool returning_home_ = false;
    bool home_reached_ = false;
    bool attempting_goal_ = false;
    
    rclcpp::Time time_at_goal_attempt_;
    double last_log_time_ = 0.0;
    int stuck_counter_ = 0;
    
    // Grid system
    std::vector<GridCell> spiral_path_;
    std::set<GridCell> visited_cells_;
    std::set<GridCell> unreachable_cells_;
    size_t current_path_index_ = 0;
    
    // Obstacle detection (8 sectors)
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
    double grid_spacing_;
    int grid_radius_;
    double cruise_speed_;
    double goal_tolerance_;
    double obstacle_distance_;
    double max_deviation_;
    double target_agl_;
    double blocked_timeout_;
    double angular_speed_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GridExplorer>());
    rclcpp::shutdown();
    return 0;
}