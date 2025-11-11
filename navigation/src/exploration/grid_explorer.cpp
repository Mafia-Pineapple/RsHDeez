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
        this->declare_parameter("grid_spacing", 20.0);           // 2m between grid points
        this->declare_parameter("grid_radius", 2);             // 25 cells (50m) in each direction
        this->declare_parameter("cruise_speed", 1.5);           // 1.5 m/s
        this->declare_parameter("goal_tolerance", 0.5);         // 0.5m to consider goal reached
        this->declare_parameter("obstacle_distance", 3.0);      // 1.5m obstacle detection range
        this->declare_parameter("max_deviation", 5.0);          // 5m max lateral deviation when avoiding
        this->declare_parameter("target_agl", 15.0);            // 10m above ground
        this->declare_parameter("blocked_timeout", 10.0);       // 10s to mark as unreachable
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
            home_landing_altitude_ = current_position_.z + 2.0;  // Land 2m above takeoff position
            has_odom_ = true;
            
            RCLCPP_INFO(this->get_logger(), "Home base: (%.2f, %.2f, %.2f)", 
                       home_position_.x, home_position_.y, home_position_.z);
            RCLCPP_INFO(this->get_logger(), "Landing altitude set to: %.2fm (takeoff + 2m)", 
                       home_landing_altitude_);
            
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
                       " Visited grid [%d, %d] | Progress: %zu/%zu (%.0f%%)",
                       target.x, target.y, 
                       visited_cells_.size() + unreachable_cells_.size(),
                       spiral_path_.size(),
                       100.0 * (visited_cells_.size() + unreachable_cells_.size()) / spiral_path_.size());
            
            // Reset blocked timer
            time_at_goal_attempt_ = this->now();
            stuck_counter_ = 0;
            climb_attempts_ = 0;  // Reset climb attempts for next goal
            climbing_over_obstacle_ = false;
            
            return;
        }
        
        // Check if we've been trying to reach this goal for too long
        if (!attempting_goal_) {
            attempting_goal_ = true;
            time_at_goal_attempt_ = this->now();
            last_progress_position_ = current_position_;
            last_distance_to_goal_ = distance;  // Initialize distance tracking
            stuck_counter_ = 0;
            climb_attempts_ = 0;  // Reset climb attempts for new goal
            climbing_over_obstacle_ = false;
        }
        
        double time_attempting = (this->now() - time_at_goal_attempt_).seconds();
        
        // Check if we're making progress TOWARD THE GOAL (not just moving around)
        double progress_dx = current_position_.x - last_progress_position_.x;
        double progress_dy = current_position_.y - last_progress_position_.y;
        double progress_dist = std::sqrt(progress_dx*progress_dx + progress_dy*progress_dy);
        
        // Check if we're getting closer to the goal
        bool getting_closer = (distance < last_distance_to_goal_ - 0.2);  // At least 20cm closer
        
        if (progress_dist < 0.1 || !getting_closer) {  
            stuck_counter_++;
        } else {
            stuck_counter_ = 0;
            last_progress_position_ = current_position_;
            last_distance_to_goal_ = distance; 
        }
        
       
        bool obstacle_in_way = false;
        if (closest_obstacle_distance_ < 2.0 && distance > 3.0) {
           
            double goal_angle = std::atan2(dy, dx);
            double current_heading = current_yaw_;
            double heading_diff = std::abs(goal_angle - current_heading);
            while (heading_diff > M_PI) heading_diff -= 2*M_PI;
            
          
            if (std::abs(heading_diff) < M_PI/4 && front_distance_ < 2.5) { 
                obstacle_in_way = true;
                stuck_counter_ += 5; 
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                   "Obstacle blocking direct path to goal! Dist: %.1fm", 
                                   front_distance_);
            }
        }
        
    
        double angle_to_target = std::atan2(dy, dx);
        double angle_error = angle_to_target - current_yaw_;
        
        // Normalize angle to [-pi, pi]
        while (angle_error > M_PI) angle_error -= 2*M_PI;
        while (angle_error < -M_PI) angle_error += 2*M_PI;
        
        // Check if we should attempt to climb over obstacle before giving up
        // Criteria: Stuck for 5+ seconds, haven't tried climbing yet, and obstacle is in the way
        bool critically_stuck = stuck_counter_ > 100;  // 5 seconds stuck (was 3)
        bool should_try_climbing = critically_stuck && climb_attempts_ == 0 && obstacle_in_way;
        
        if (should_try_climbing) {
            climbing_over_obstacle_ = true;
            climb_start_time_ = this->now();
            climb_start_agl_ = current_agl_;
            climb_attempts_++;
            
            RCLCPP_WARN(this->get_logger(),
                       "ATTEMPTING CLIMB-OVER MANEUVER! Rising 5m to clear obstacle...");
            RCLCPP_INFO(this->get_logger(),
                       "   Current AGL: %.1fm | Target: %.1fm", current_agl_, target_agl_ + 5.0);
        }
        
        // If climbing over obstacle
        if (climbing_over_obstacle_) {
            double climb_duration = (this->now() - climb_start_time_).seconds();
            double target_climb_agl = target_agl_ + 5.0;  // Climb 5m higher than normal
            
            cmd.linear.x = cruise_speed_ * 0.5 * std::cos(angle_error);  // Half speed while climbing
            cmd.linear.y = cruise_speed_ * 0.5 * std::sin(angle_error);
            cmd.angular.z = std::max(-angular_speed_, std::min(angular_speed_, angle_error * 0.5));
            
            // Climb up
            double climb_error = target_climb_agl - current_agl_;
            cmd.linear.z = std::max(0.0, std::min(1.0, climb_error * 0.5));  // Climb at max 1 m/s
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               " Climbing over obstacle | AGL: %.1fm / %.1fm | Goal dist: %.1fm",
                               current_agl_, target_climb_agl, distance);
            
            // Check if we've climbed high enough
            if (current_agl_ >= target_climb_agl - 0.5) {
                RCLCPP_INFO(this->get_logger(),
                           " Reached climb altitude (%.1fm). Continuing toward goal...", current_agl_);
                climbing_over_obstacle_ = false;
                stuck_counter_ = 0;  // Reset stuck counter after successful climb
                last_progress_position_ = current_position_;
                last_distance_to_goal_ = distance;
            }
            
            
            if (climb_duration > 10.0) {
                RCLCPP_ERROR(this->get_logger(),
                           " Climb-over failed after 10s. Marking unreachable.");
                climbing_over_obstacle_ = false;
                unreachable_cells_.insert(target);
                current_path_index_++;
                attempting_goal_ = false;
                climb_attempts_ = 0;
                return;
            }
            
            // Publish command and return (skip normal navigation while climbing)
            cmd_vel_pub_->publish(cmd);
            return;
        }
        
        // Mark as unreachable if stuck OR taking too long WHILE DEALING WITH OBSTACLES
        // Only after attempting climb-over
        bool stuck_too_long = stuck_counter_ > 240;  // 240 * 50ms = 12 seconds with no progress
        
        // Timeout only applies when obstacles are ACTUALLY BLOCKING (very close, not just detected)
        bool obstacles_blocking = closest_obstacle_distance_ < 2.0;  // Must be within 2m to count as blocking
        bool timeout = obstacles_blocking && time_attempting > 45.0;  // 45s timeout when truly blocked
        
        if (stuck_too_long || timeout) {
            if (timeout) {
                RCLCPP_WARN(this->get_logger(),
                           " Grid [%d, %d] marked UNREACHABLE (timeout after %.1fs with obstacles)",
                           target.x, target.y, time_attempting);
            } else {
                RCLCPP_WARN(this->get_logger(),
                           " Grid [%d, %d] marked UNREACHABLE (stuck for %.1fs with no progress)",
                           target.x, target.y, time_attempting);
            }
            unreachable_cells_.insert(target);
            current_path_index_++;
            attempting_goal_ = false;
            climb_attempts_ = 0;  // Reset for next goal
            
            return;
        }
        
        // Basic navigation command (angle_error already calculated above)
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
        
        // Periodic logging (every 3 seconds)
        if (time_attempting - last_log_time_ > 2.5) {
            RCLCPP_INFO(this->get_logger(),
                       "-> Grid [%d, %d] | Dist: %.1fm | AGL: %.1fm | Obstacle: %.1fm",
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
        
        // Calculate how close we are to obstacle (0.0 = at obstacle_distance_, 1.0 = at 0m)
        double danger_level = 1.0 - (closest_obstacle_distance_ / obstacle_distance_);
        danger_level = std::max(0.0, std::min(1.0, danger_level));
        
        // CRITICAL ZONE: < 1.5m - EMERGENCY STOP AND ESCAPE
        if (closest_obstacle_distance_ < 1.5) {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 200,
                                " CRITICAL! Obstacle at %.2fm - EMERGENCY ESCAPE!", 
                                closest_obstacle_distance_);
            
            // COMPLETELY OVERRIDE navigation commands
            // STOP all forward motion toward obstacle
            safe_cmd.linear.x = 0.0;
            safe_cmd.linear.y = 0.0;
            
            // If VERY close (< 1m), actively move BACKWARD
            if (closest_obstacle_distance_ < 1.0) {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 100,
                                    " COLLISION IMMINENT at %.2fm! BACKING UP!", 
                                    closest_obstacle_distance_);
                
                // Move away from closest obstacle
                // Find which direction obstacle is in
                if (front_distance_ < 1.5) {
                    safe_cmd.linear.x = -cruise_speed_ * 0.5;  // Back up
                }
                if (left_distance_ < 1.5) {
                    safe_cmd.linear.y = -cruise_speed_ * 0.8;  // Move right aggressively
                }
                if (right_distance_ < 1.5) {
                    safe_cmd.linear.y = cruise_speed_ * 0.8;   // Move left aggressively
                }
            }
            
            // Turn aggressively toward open space
            if (front_distance_ < obstacle_distance_) {
                if (front_left_distance_ > front_right_distance_) {
                    safe_cmd.angular.z = angular_speed_ * 2.5;  // Fast left turn
                } else {
                    safe_cmd.angular.z = -angular_speed_ * 2.5; // Fast right turn
                }
            }
            
            return safe_cmd;  // Return immediately - ignore rest of navigation
        }
        
 
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                           " Obstacle at %.2fm! Avoiding...", closest_obstacle_distance_);
        
        // Reduce forward speed based on proximity
        // At 3m: 100% speed, At 1.5m: 0% speed
        double speed_factor = (closest_obstacle_distance_ - 1.5) / (obstacle_distance_ - 1.5);
        speed_factor = std::max(0.0, std::min(1.0, speed_factor));
        
        // Apply speed reduction (more aggressive than before)
        safe_cmd.linear.x *= speed_factor;
        safe_cmd.linear.y *= speed_factor;
        
        // Determine escape direction - find sector with most space
        double max_distance = 0;
        int best_sector = 0;
        
        for (size_t i = 0; i < sectors_.size(); ++i) {
            if (sectors_[i] > max_distance) {
                max_distance = sectors_[i];
                best_sector = i;
            }
        }
        
        // Apply strong avoidance based on obstacle location
        if (front_distance_ < obstacle_distance_) {
            // Obstacle ahead - turn and slide away
            if (front_left_distance_ > front_right_distance_) {
                // More space on left
                safe_cmd.angular.z += angular_speed_ * 1.8;
                safe_cmd.linear.y += cruise_speed_ * 0.6 * danger_level;  // Stronger lateral
            } else {
                // More space on right
                safe_cmd.angular.z -= angular_speed_ * 1.8;
                safe_cmd.linear.y -= cruise_speed_ * 0.6 * danger_level;  // Stronger lateral
            }
        }
        
        // Strong lateral avoidance for side obstacles
        if (left_distance_ < obstacle_distance_) {
            safe_cmd.linear.y -= cruise_speed_ * 0.7 * danger_level;  // Move right strongly
        }
        if (right_distance_ < obstacle_distance_) {
            safe_cmd.linear.y += cruise_speed_ * 0.7 * danger_level;  // Move left strongly
        }
        
        // If back obstacle, move forward to escape
        if (back_distance_ < obstacle_distance_ && back_distance_ < 2.0) {
            safe_cmd.linear.x += cruise_speed_ * 0.4;  // Move forward to escape
        }
        
        // Clamp velocities to safe limits
        safe_cmd.linear.x = std::max(-cruise_speed_, std::min(cruise_speed_, safe_cmd.linear.x));
        safe_cmd.linear.y = std::max(-cruise_speed_, std::min(cruise_speed_, safe_cmd.linear.y));
        safe_cmd.angular.z = std::max(-angular_speed_ * 2.5, std::min(angular_speed_ * 2.5, safe_cmd.angular.z));
        
        return safe_cmd;
    }
    
    void returnHome() {
        geometry_msgs::msg::Twist cmd;
        
        double dx = home_position_.x - current_position_.x;
        double dy = home_position_.y - current_position_.y;
        double dist_xy = std::sqrt(dx*dx + dy*dy);
        
        // Calculate altitude error (want to be at landing altitude = takeoff Z + 2m)
        double altitude_error = home_landing_altitude_ - current_position_.z;
        
        // Check if we're at home position (XY within 1m, Z within 0.5m of landing altitude)
        bool xy_at_home = dist_xy < 1.0;
        bool z_at_landing = std::abs(altitude_error) < 0.5;
        
        if (xy_at_home && z_at_landing) {
            // Perfectly at home - stop and land
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.linear.z = 0.0;
            cmd.angular.z = 0.0;
            
            if (!home_reached_) {
                RCLCPP_INFO(this->get_logger(), "=== HOME REACHED ===");
                RCLCPP_INFO(this->get_logger(), "Position: (%.2f, %.2f, %.2f)", 
                           current_position_.x, current_position_.y, current_position_.z);
                RCLCPP_INFO(this->get_logger(), "Landing altitude: %.2fm", home_landing_altitude_);
                RCLCPP_INFO(this->get_logger(), "Mission complete!");
                RCLCPP_INFO(this->get_logger(), "Total visited: %zu | Unreachable: %zu | Coverage: %.1f%%",
                           visited_cells_.size(), unreachable_cells_.size(),
                           100.0 * visited_cells_.size() / spiral_path_.size());
                home_reached_ = true;
                timer_->cancel();
            }
        } else {
            // Navigate home in XY and adjust altitude to landing height
            
            // XY navigation toward home
            if (dist_xy > 0.5) {
                double angle_to_home = std::atan2(dy, dx);
                double angle_diff = angle_to_home - current_yaw_;
                
                while (angle_diff > M_PI) angle_diff -= 2*M_PI;
                while (angle_diff < -M_PI) angle_diff += 2*M_PI;
                
                cmd.linear.x = std::min(cruise_speed_ * 0.8, dist_xy * 0.5);
                cmd.angular.z = std::max(-angular_speed_, std::min(angular_speed_, angle_diff));
            } else {
                // Already at home XY position, just hovering
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
            }
            
            // Z control - fly to landing altitude
            cmd.linear.z = std::max(-0.8, std::min(0.8, altitude_error * 0.4));
            
            // Still apply collision avoidance on way home
            cmd = applyCollisionAvoidance(cmd);
            
            // Status logging
            if (dist_xy > 1.0) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                   "Returning home... %.1fm remaining | Alt: %.1fm/%.1fm",
                                   dist_xy, current_position_.z, home_landing_altitude_);
            } else {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                   "At home XY, adjusting altitude: %.1fm → %.1fm",
                                   current_position_.z, home_landing_altitude_);
            }
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
    double home_landing_altitude_ = 0.0;  // NEW: Z position at takeoff for landing
    
    bool has_odom_ = false;
    bool returning_home_ = false;
    bool home_reached_ = false;
    bool attempting_goal_ = false;
    bool climbing_over_obstacle_ = false;  // NEW: Climb-over state
    
    rclcpp::Time time_at_goal_attempt_;
    rclcpp::Time climb_start_time_;        // NEW: When climb started
    double last_log_time_ = 0.0;
    int stuck_counter_ = 0;
    int climb_attempts_ = 0;               // NEW: Track number of climb attempts
    double last_distance_to_goal_ = 0.0;   // Track if getting closer to goal
    double climb_start_agl_ = 0.0;         // NEW: AGL when climb started
    
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