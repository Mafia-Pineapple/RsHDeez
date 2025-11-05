#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <cmath>

class Octomap3DNavigator : public rclcpp::Node {
public:
    Octomap3DNavigator() : Node("octomap_3d_navigator") {
        // Parameters
        this->declare_parameter("cruise_speed", 2.0);
        this->declare_parameter("obstacle_distance", 2.0);     // 3D obstacle check distance
        this->declare_parameter("safe_distance", 4.0);
        this->declare_parameter("bear_avoidance_distance", 8.0);
        this->declare_parameter("goal_tolerance", 1.0);
        this->declare_parameter("vertical_clearance", 1.5);    // Min clearance above/below
        this->declare_parameter("horizontal_clearance", 1.0);  // Min side clearance
        this->declare_parameter("path_check_resolution", 0.5); // Check path every 0.5m
        this->declare_parameter("map_change_threshold", 0.1);  // Replan if >10% map change
        this->declare_parameter("takeoff_altitude", 2.0);      // Initial climb height (meters)
        this->declare_parameter("takeoff_speed", 2.0);         // Slow climb speed (m/s)
        this->declare_parameter("auto_takeoff", true);         // Enable automatic takeoff
        
        cruise_speed_ = this->get_parameter("cruise_speed").as_double();
        obstacle_distance_ = this->get_parameter("obstacle_distance").as_double();
        safe_distance_ = this->get_parameter("safe_distance").as_double();
        bear_avoidance_distance_ = this->get_parameter("bear_avoidance_distance").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
        vertical_clearance_ = this->get_parameter("vertical_clearance").as_double();
        horizontal_clearance_ = this->get_parameter("horizontal_clearance").as_double();
        path_check_resolution_ = this->get_parameter("path_check_resolution").as_double();
        map_change_threshold_ = this->get_parameter("map_change_threshold").as_double();
        takeoff_altitude_ = this->get_parameter("takeoff_altitude").as_double();
        takeoff_speed_ = this->get_parameter("takeoff_speed").as_double();
        auto_takeoff_ = this->get_parameter("auto_takeoff").as_bool();
        
        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry", 10,
            std::bind(&Octomap3DNavigator::odomCallback, this, std::placeholders::_1));
        
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&Octomap3DNavigator::scanCallback, this, std::placeholders::_1));
        
        octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            "/octomap_binary", 10,
            std::bind(&Octomap3DNavigator::octomapCallback, this, std::placeholders::_1));
        
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/nav_goal_3d", 10,
            std::bind(&Octomap3DNavigator::goalCallback, this, std::placeholders::_1));
        
        bear_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/bear_detection", 10,
            std::bind(&Octomap3DNavigator::bearCallback, this, std::placeholders::_1));
        
        status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/detection_status", 10,
            std::bind(&Octomap3DNavigator::statusCallback, this, std::placeholders::_1));
        
        // Publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // TF
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Control timer (20Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&Octomap3DNavigator::navigate, this));
        
        RCLCPP_INFO(this->get_logger(), "Octomap 3D Navigator initialized");
        RCLCPP_INFO(this->get_logger(), "  - Using Octomap for 3D collision avoidance");
        RCLCPP_INFO(this->get_logger(), "  - Cruise speed: %.2f m/s", cruise_speed_);
        RCLCPP_INFO(this->get_logger(), "  - 3D obstacle distance: %.2f m", obstacle_distance_);
        RCLCPP_INFO(this->get_logger(), "  - Vertical clearance: %.2f m", vertical_clearance_);
        
        if (auto_takeoff_) {
            RCLCPP_INFO(this->get_logger(), "  - AUTO TAKEOFF ENABLED: Will climb %.2f m at %.2f m/s", 
                       takeoff_altitude_, takeoff_speed_);
            RCLCPP_INFO(this->get_logger(), "  - Waiting for odometry before takeoff...");
        }
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_z_ = msg->pose.pose.position.z;
        
        // Store initial altitude on first callback
        if (!has_odom_) {
            initial_z_ = current_z_;
            RCLCPP_INFO(this->get_logger(), "Initial altitude: %.2f m", initial_z_);
            
            if (auto_takeoff_) {
                takeoff_target_z_ = initial_z_ + takeoff_altitude_;
                RCLCPP_INFO(this->get_logger(), "Takeoff target: %.2f m (climbing %.2f m)", 
                           takeoff_target_z_, takeoff_altitude_);
            }
        }
        
        // Extract yaw from quaternion
        double qw = msg->pose.pose.orientation.w;
        double qz = msg->pose.pose.orientation.z;
        current_yaw_ = std::atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz);
        
        has_odom_ = true;
    }
    
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        scan_ = *msg;
        has_scan_ = true;
        
        // Analyze 8 sectors for emergency horizontal collision avoidance
        analyzeSectors();
    }
    
    void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
        // Convert ROS message to Octomap
        octomap::AbstractOcTree* abstract_tree = octomap_msgs::binaryMsgToMap(*msg);
        if (abstract_tree) {
            auto new_octree = std::shared_ptr<octomap::OcTree>(
                dynamic_cast<octomap::OcTree*>(abstract_tree));
            
            if (new_octree) {
                // Track map updates
                map_update_count_++;
                last_map_update_time_ = this->now();
                
                // Check if map significantly changed
                bool significant_change = false;
                if (octree_) {
                    size_t old_size = octree_->size();
                    size_t new_size = new_octree->size();
                    double change_ratio = std::abs(static_cast<double>(new_size - old_size)) / 
                                         std::max(old_size, size_t(1));
                    
                    if (change_ratio > map_change_threshold_) {
                        significant_change = true;
                        RCLCPP_INFO(this->get_logger(), 
                                   "Map significantly updated! Old: %zu voxels, New: %zu voxels (%.1f%% change)",
                                   old_size, new_size, change_ratio * 100.0);
                    }
                }
                
                // Update map
                octree_ = new_octree;
                has_octomap_ = true;
                
                // Revalidate current path if map changed significantly
                if (significant_change && has_goal_ && !goal_reached_) {
                    bool path_still_clear = checkPathClear(current_x_, current_y_, current_z_,
                                                          goal_x_, goal_y_, goal_z_);
                    if (!path_still_clear) {
                        RCLCPP_WARN(this->get_logger(), 
                                   "Path to goal BLOCKED by new map data! Replanning...");
                        path_blocked_by_new_data_ = true;
                    }
                }
                
                if (!has_octomap_) {
                    RCLCPP_INFO(this->get_logger(), 
                               "Octomap received! Resolution: %.2f m, Size: %zu voxels", 
                               octree_->getResolution(), octree_->size());
                }
            }
        }
    }
    
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        // Transform goal to map frame if needed
        geometry_msgs::msg::PointStamped goal_in_map;
        
        if (msg->header.frame_id != "map" && msg->header.frame_id != "") {
            // Goal is in a different frame (e.g., base_link) - transform it!
            try {
                tf_buffer_->transform(*msg, goal_in_map, "map", tf2::durationFromSec(1.0));
                
                RCLCPP_INFO(this->get_logger(), 
                           "Transformed goal from '%s' to 'map': (%.2f, %.2f, %.2f) -> (%.2f, %.2f, %.2f)",
                           msg->header.frame_id.c_str(),
                           msg->point.x, msg->point.y, msg->point.z,
                           goal_in_map.point.x, goal_in_map.point.y, goal_in_map.point.z);
            } catch (tf2::TransformException& ex) {
                RCLCPP_ERROR(this->get_logger(), 
                            "Could not transform goal from '%s' to 'map': %s",
                            msg->header.frame_id.c_str(), ex.what());
                return;
            }
        } else {
            // Goal already in map frame
            goal_in_map = *msg;
        }
        
        goal_x_ = goal_in_map.point.x;
        goal_y_ = goal_in_map.point.y;
        goal_z_ = goal_in_map.point.z;
        has_goal_ = true;
        goal_reached_ = false;
        
        RCLCPP_INFO(this->get_logger(), "New 3D goal in MAP frame: (%.2f, %.2f, %.2f)",
                   goal_x_, goal_y_, goal_z_);
        
        // Check if path to goal is collision-free
        if (has_octomap_) {
            bool path_clear = checkPathClear(current_x_, current_y_, current_z_,
                                            goal_x_, goal_y_, goal_z_);
            if (!path_clear) {
                RCLCPP_WARN(this->get_logger(), "Direct path to goal is BLOCKED!");
            } else {
                RCLCPP_INFO(this->get_logger(), "Direct path to goal is CLEAR!");
            }
        }
    }
    
    void bearCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        bear_x_ = msg->point.x;
        bear_y_ = msg->point.y;
        bear_z_ = msg->point.z;
        bear_detected_ = true;
        last_bear_time_ = this->now();
        
        RCLCPP_WARN(this->get_logger(), "BEAR at (%.2f, %.2f, %.2f) - AVOIDING!",
                   bear_x_, bear_y_, bear_z_);
    }
    
    void statusCallback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "BEAR_DETECTED") {
            bear_detected_ = true;
            last_bear_time_ = this->now();
        }
    }
    
    void analyzeSectors() {
        // Divide LIDAR into 8 sectors for quick horizontal checks
        sector_min_dist_.clear();
        sector_min_dist_.resize(8, 999.0);
        
        for (size_t i = 0; i < scan_.ranges.size(); ++i) {
            float range = scan_.ranges[i];
            if (!std::isfinite(range) || range < scan_.range_min || range > scan_.range_max) {
                continue;
            }
            
            float angle = scan_.angle_min + i * scan_.angle_increment;
            int sector = static_cast<int>((angle + M_PI) / (M_PI / 4.0)) % 8;
            
            if (range < sector_min_dist_[sector]) {
                sector_min_dist_[sector] = range;
            }
        }
    }
    
    // ========== 3D COLLISION CHECKING WITH OCTOMAP ==========
    
    bool isOccupied(double x, double y, double z) {
        if (!octree_) return false;
        
        octomap::point3d query(x, y, z);
        octomap::OcTreeNode* node = octree_->search(query);
        
        if (node) {
            return octree_->isNodeOccupied(node);
        }
        return false;  // Unknown = free (optimistic)
    }
    
    bool checkCylinderCollision(double x, double y, double z, double radius, double height) {
        // Check if a cylinder around (x,y,z) has any obstacles
        // This checks the drone's body + safety margin
        
        if (!octree_) return false;
        
        double check_radius = radius + horizontal_clearance_;
        double check_height_below = height / 2.0 + vertical_clearance_;
        double check_height_above = height / 2.0 + vertical_clearance_;
        
        // Sample points in a cylinder
        for (double dz = -check_height_below; dz <= check_height_above; dz += octree_->getResolution()) {
            for (double angle = 0; angle < 2 * M_PI; angle += M_PI / 8.0) {
                for (double r = 0; r <= check_radius; r += octree_->getResolution()) {
                    double px = x + r * std::cos(angle);
                    double py = y + r * std::sin(angle);
                    double pz = z + dz;
                    
                    if (isOccupied(px, py, pz)) {
                        return true;  // Collision!
                    }
                }
            }
        }
        
        return false;  // No collision
    }
    
    bool checkPathClear(double x1, double y1, double z1, 
                       double x2, double y2, double z2) {
        // Check if straight-line path is collision-free
        
        if (!octree_) return true;  // No map = assume clear
        
        double dx = x2 - x1;
        double dy = y2 - y1;
        double dz = z2 - z1;
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        if (dist < 0.1) return true;
        
        // Sample points along path
        int num_checks = static_cast<int>(dist / path_check_resolution_) + 1;
        
        for (int i = 0; i <= num_checks; ++i) {
            double t = static_cast<double>(i) / num_checks;
            double x = x1 + t * dx;
            double y = y1 + t * dy;
            double z = z1 + t * dz;
            
            if (checkCylinderCollision(x, y, z, 0.5, 1.0)) {  // 0.5m radius, 1m height
                return false;  // Path blocked!
            }
        }
        
        return true;  // Path clear!
    }
    
    void findBestEscapeDirection(double& escape_x, double& escape_y, double& escape_z) {
        // Use Octomap to find the most open direction
        
        if (!octree_) {
            // Fallback to LIDAR-based escape
            escape_x = std::cos(current_yaw_ + M_PI);  // Go backward
            escape_y = std::sin(current_yaw_ + M_PI);
            escape_z = 0.0;
            return;
        }
        
        // Sample directions around drone
        std::vector<std::pair<double, std::tuple<double, double, double>>> clearances;
        
        for (double yaw = 0; yaw < 2 * M_PI; yaw += M_PI / 8.0) {
            for (double pitch = -M_PI/6; pitch <= M_PI/6; pitch += M_PI/12.0) {
                double test_x = current_x_ + 3.0 * std::cos(pitch) * std::cos(yaw);
                double test_y = current_y_ + 3.0 * std::cos(pitch) * std::sin(yaw);
                double test_z = current_z_ + 3.0 * std::sin(pitch);
                
                if (checkPathClear(current_x_, current_y_, current_z_, 
                                  test_x, test_y, test_z)) {
                    double clearance = 3.0;  // Full clearance
                    clearances.push_back({clearance, {
                        std::cos(pitch) * std::cos(yaw),
                        std::cos(pitch) * std::sin(yaw),
                        std::sin(pitch)
                    }});
                }
            }
        }
        
        if (!clearances.empty()) {
            // Choose direction with most clearance
            auto best = std::max_element(clearances.begin(), clearances.end());
            auto [dx, dy, dz] = best->second;
            escape_x = dx;
            escape_y = dy;
            escape_z = dz;
        } else {
            // Emergency: go up!
            escape_x = 0.0;
            escape_y = 0.0;
            escape_z = 1.0;
            RCLCPP_WARN(this->get_logger(), "EMERGENCY: Going UP!");
        }
    }
    
    // ========== NAVIGATION LOGIC ==========
    
    void navigate() {
        if (!has_odom_ || !has_scan_) return;
        
        geometry_msgs::msg::Twist cmd;
        
        // ========== TAKEOFF SEQUENCE ==========
        if (auto_takeoff_ && !takeoff_complete_) {
            double altitude_gain = current_z_ - initial_z_;
            double altitude_error = takeoff_target_z_ - current_z_;
            
            if (altitude_gain >= takeoff_altitude_ - 0.1) {
                // Takeoff complete!
                takeoff_complete_ = true;
                RCLCPP_INFO(this->get_logger(), 
                           "✓ TAKEOFF COMPLETE! Altitude: %.2f m (gained %.2f m)", 
                           current_z_, altitude_gain);
                RCLCPP_INFO(this->get_logger(), "Ready for navigation goals!");
                
                // Hover briefly after takeoff
                cmd.linear.x = 0.0;
                cmd.linear.y = 0.0;
                cmd.linear.z = 0.0;
                cmd_vel_pub_->publish(cmd);
                return;
                
            } else {
                // Still climbing
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                    "TAKEOFF: Climbing... %.2f / %.2f m", 
                                    altitude_gain, takeoff_altitude_);
                
                // Smooth vertical climb with deceleration near target
                double climb_speed = takeoff_speed_;
                if (altitude_error < 0.5) {
                    // Slow down as we approach target
                    climb_speed = takeoff_speed_ * (altitude_error / 0.5);
                    climb_speed = std::max(0.1, climb_speed);
                }
                
                cmd.linear.x = 0.0;
                cmd.linear.y = 0.0;
                cmd.linear.z = climb_speed;
                cmd.angular.z = 0.0;
                cmd_vel_pub_->publish(cmd);
                return;
            }
        }
        
        // ========== NORMAL NAVIGATION (after takeoff) ==========
        
        if (!has_goal_) {
            // No goal - hover
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.linear.z = 0.0;
            cmd_vel_pub_->publish(cmd);
            return;
        }
        
        // Check if goal reached
        double dx = goal_x_ - current_x_;
        double dy = goal_y_ - current_y_;
        double dz = goal_z_ - current_z_;
        double dist_to_goal = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        if (dist_to_goal < goal_tolerance_) {
            if (!goal_reached_) {
                RCLCPP_INFO(this->get_logger(), "Goal reached!");
                goal_reached_ = true;
            }
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.linear.z = 0.0;
            cmd_vel_pub_->publish(cmd);
            return;
        }
        
        // Check bear proximity
        double bear_dist = 999.0;
        if (bear_detected_ && (this->now() - last_bear_time_).seconds() < 10.0) {
            double bear_dx = bear_x_ - current_x_;
            double bear_dy = bear_y_ - current_y_;
            bear_dist = std::sqrt(bear_dx*bear_dx + bear_dy*bear_dy);
        } else {
            bear_detected_ = false;
        }
        
        // Calculate desired direction
        double goal_angle = std::atan2(dy, dx);
        double angle_error = goal_angle - current_yaw_;
        while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
        while (angle_error < -M_PI) angle_error += 2.0 * M_PI;
        
        // Check if path ahead is clear using Octomap
        bool path_clear = true;
        double check_dist = std::min(obstacle_distance_, dist_to_goal);
        
        if (has_octomap_) {
            double next_x = current_x_ + check_dist * std::cos(goal_angle);
            double next_y = current_y_ + check_dist * std::sin(goal_angle);
            double next_z = current_z_ + dz / dist_to_goal * check_dist;
            
            path_clear = checkPathClear(current_x_, current_y_, current_z_,
                                       next_x, next_y, next_z);
            
            // Dynamic replanning: Check if newly discovered obstacles block path
            if (path_blocked_by_new_data_ || !path_clear) {
                if (path_blocked_by_new_data_) {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                        "Dynamically replanning around new obstacles...");
                    path_blocked_by_new_data_ = false;
                }
                
                // Don't give up - find alternate route
                // The escape logic below will handle it
            }
        }
        
        // Also check horizontal LIDAR for immediate threats
        double front_dist = sector_min_dist_[0];
        
        // Decision logic
        if (!path_clear || front_dist < obstacle_distance_ || bear_dist < bear_avoidance_distance_) {
            // CRITICAL: Obstacle or bear detected!
            
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "OBSTACLE AHEAD! Path clear: %d, Front: %.2fm, Bear: %.2fm",
                               path_clear, front_dist, bear_dist);
            
            if (bear_dist < bear_avoidance_distance_) {
                // Escape from bear
                double bear_angle = std::atan2(bear_y_ - current_y_, bear_x_ - current_x_);
                double escape_angle = bear_angle + M_PI;
                
                cmd.linear.x = cruise_speed_ * 0.5 * std::cos(escape_angle - current_yaw_);
                cmd.linear.y = cruise_speed_ * 0.5 * std::sin(escape_angle - current_yaw_);
                cmd.linear.z = 0.3;  // Also climb
                cmd.angular.z = angle_error;
                
            } else {
                // Find best escape direction using Octomap
                double escape_dx, escape_dy, escape_dz;
                findBestEscapeDirection(escape_dx, escape_dy, escape_dz);
                
                cmd.linear.x = cruise_speed_ * 0.4 * escape_dx;
                cmd.linear.y = cruise_speed_ * 0.4 * escape_dy;
                cmd.linear.z = cruise_speed_ * 0.4 * escape_dz;
                cmd.angular.z = findBestDirection();
            }
            
        } else {
            // SAFE: Navigate toward goal
            double speed = std::min(cruise_speed_, dist_to_goal * 0.5);
            
            cmd.linear.x = speed * std::cos(angle_error);
            cmd.linear.y = speed * std::sin(angle_error);
            cmd.linear.z = (goal_z_ - current_z_) * 0.3;
            cmd.angular.z = angle_error * 0.8;
        }
        
        // Clamp velocities
        cmd.linear.x = std::max(-cruise_speed_, std::min(cruise_speed_, cmd.linear.x));
        cmd.linear.y = std::max(-cruise_speed_*0.5, std::min(cruise_speed_*0.5, cmd.linear.y));
        cmd.linear.z = std::max(-0.5, std::min(0.5, cmd.linear.z));
        cmd.angular.z = std::max(-0.5, std::min(0.5, cmd.angular.z));
        
        cmd_vel_pub_->publish(cmd);
    }
    
    double findBestDirection() {
        // Find LIDAR sector with most space
        int best_sector = 0;
        double max_dist = 0.0;
        
        for (int i = 0; i < 8; ++i) {
            if (sector_min_dist_[i] > max_dist) {
                max_dist = sector_min_dist_[i];
                best_sector = i;
            }
        }
        
        double target_angle = (best_sector - 4) * (M_PI / 4.0);
        return target_angle * 0.5;
    }
    
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr bear_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    
    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // State
    double current_x_ = 0.0, current_y_ = 0.0, current_z_ = 0.0;
    double current_yaw_ = 0.0;
    double initial_z_ = 0.0;
    double takeoff_target_z_ = 0.0;
    double goal_x_ = 0.0, goal_y_ = 0.0, goal_z_ = 0.0;
    double bear_x_ = 0.0, bear_y_ = 0.0, bear_z_ = 0.0;
    
    bool has_odom_ = false;
    bool has_scan_ = false;
    bool has_octomap_ = false;
    bool has_goal_ = false;
    bool goal_reached_ = false;
    bool bear_detected_ = false;
    bool path_blocked_by_new_data_ = false;
    bool takeoff_complete_ = false;
    
    rclcpp::Time last_bear_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_map_update_time_{0, 0, RCL_ROS_TIME};
    size_t map_update_count_ = 0;
    
    // Octomap
    std::shared_ptr<octomap::OcTree> octree_;
    
    // LIDAR data
    sensor_msgs::msg::LaserScan scan_;
    std::vector<double> sector_min_dist_;
    
    // Parameters
    double cruise_speed_;
    double obstacle_distance_;
    double safe_distance_;
    double bear_avoidance_distance_;
    double goal_tolerance_;
    double vertical_clearance_;
    double horizontal_clearance_;
    double path_check_resolution_;
    double map_change_threshold_;
    double takeoff_altitude_;
    double takeoff_speed_;
    bool auto_takeoff_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Octomap3DNavigator>());
    rclcpp::shutdown();
    return 0;
}