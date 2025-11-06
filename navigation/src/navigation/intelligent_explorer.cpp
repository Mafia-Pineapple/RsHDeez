#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <vector>
#include <cmath>
#include <unordered_set>
#include <queue>
#include <algorithm>

// Structure to represent a 2D grid cell
struct Cell {
    int x, y;
    bool operator==(const Cell& other) const {
        return x == other.x && y == other.y;
    }
};

// Hash function for Cell to use in unordered_set
struct CellHash {
    std::size_t operator()(const Cell& cell) const {
        return std::hash<int>()(cell.x) ^ (std::hash<int>()(cell.y) << 1);
    }
};

// Frontier point for exploration
struct Frontier {
    double x, y, z;
    double score;
    int cluster_size;
};

class IntelligentExplorer : public rclcpp::Node {
public:
    IntelligentExplorer() : Node("intelligent_explorer") {
        // Parameters
        this->declare_parameter("cruise_speed", 1.0);
        this->declare_parameter("target_altitude", 5.0);  // Start at 5m
        this->declare_parameter("obstacle_distance", 3.0);
        this->declare_parameter("safe_distance", 5.0);
        this->declare_parameter("bear_avoidance_distance", 8.0);
        this->declare_parameter("goal_tolerance", 1.5);  // Increased tolerance
        this->declare_parameter("grid_resolution", 1.0);  // Larger cells for faster exploration
        this->declare_parameter("exploration_radius", 50.0);  // meters
        this->declare_parameter("frontier_cluster_size", 1);  // Accept single cells as frontiers
        
        cruise_speed_ = this->get_parameter("cruise_speed").as_double();
        target_altitude_ = this->get_parameter("target_altitude").as_double();
        obstacle_distance_ = this->get_parameter("obstacle_distance").as_double();
        safe_distance_ = this->get_parameter("safe_distance").as_double();
        bear_avoidance_distance_ = this->get_parameter("bear_avoidance_distance").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
        grid_resolution_ = this->get_parameter("grid_resolution").as_double();
        exploration_radius_ = this->get_parameter("exploration_radius").as_double();
        frontier_cluster_size_ = this->get_parameter("frontier_cluster_size").as_int();
        
        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry", 10,
            std::bind(&IntelligentExplorer::odomCallback, this, std::placeholders::_1));
        
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&IntelligentExplorer::scanCallback, this, std::placeholders::_1));
        
        agl_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/drone/agl", 10,
            std::bind(&IntelligentExplorer::aglCallback, this, std::placeholders::_1));
        
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10,
            std::bind(&IntelligentExplorer::imuCallback, this, std::placeholders::_1));
        
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/nav_goal_3d", 10,
            std::bind(&IntelligentExplorer::goalCallback, this, std::placeholders::_1));
        
        bear_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/bear_detection", 10,
            std::bind(&IntelligentExplorer::bearCallback, this, std::placeholders::_1));
        
        status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/detection_status", 10,
            std::bind(&IntelligentExplorer::statusCallback, this, std::placeholders::_1));
        
        // Publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/exploration_map", 10);
        frontier_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/current_frontier", 10);
        
        // TF
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Control timer (20Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&IntelligentExplorer::navigate, this));
        
        // Map update timer (2Hz)
        map_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&IntelligentExplorer::updateMap, this));
        
        // Frontier update timer (1Hz)
        frontier_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&IntelligentExplorer::updateFrontiers, this));
        
        RCLCPP_INFO(this->get_logger(), "Intelligent Explorer initialized");
        RCLCPP_INFO(this->get_logger(), "  - Target altitude: %.2f m", target_altitude_);
        RCLCPP_INFO(this->get_logger(), "  - Cruise speed: %.2f m/s", cruise_speed_);
        RCLCPP_INFO(this->get_logger(), "  - Grid resolution: %.2f m", grid_resolution_);
        RCLCPP_INFO(this->get_logger(), "Mode: AUTONOMOUS EXPLORATION");
    }

private:
    // ========== CALLBACK FUNCTIONS ==========
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_z_ = msg->pose.pose.position.z;
        
        // Record starting position on first callback
        if (!has_odom_) {
            start_x_ = current_x_;
            start_y_ = current_y_;
            RCLCPP_INFO(this->get_logger(), "Starting position: (%.2f, %.2f)", start_x_, start_y_);
        }
        
        // Extract yaw from quaternion
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, current_yaw_);
        
        has_odom_ = true;
    }
    
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        scan_ = *msg;
        has_scan_ = true;
        analyzeSectors();
    }
    
    void aglCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (!msg->ranges.empty() && std::isfinite(msg->ranges[0])) {
            agl_height_ = msg->ranges[0];
            has_agl_ = true;
        }
    }
    
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // Extract roll and pitch for horizontal stabilization
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double yaw;
        m.getRPY(current_roll_, current_pitch_, yaw);
        has_imu_ = true;
    }
    
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        current_goal_.x = msg->point.x;
        current_goal_.y = msg->point.y;
        current_goal_.z = msg->point.z;
        has_manual_goal_ = true;
        exploration_mode_ = false;  // Manual goal takes precedence
        
        RCLCPP_INFO(this->get_logger(), "Manual goal: (%.2f, %.2f, %.2f)",
                   current_goal_.x, current_goal_.y, current_goal_.z);
    }
    
    void bearCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        bear_x_ = msg->point.x;
        bear_y_ = msg->point.y;
        bear_z_ = msg->point.z;
        bear_detected_ = true;
        last_bear_time_ = this->now();
        
        // Add bear location to obstacle map
        Cell bear_cell = worldToGrid(bear_x_, bear_y_);
        for (int dx = -5; dx <= 5; ++dx) {
            for (int dy = -5; dy <= 5; ++dy) {
                Cell c{bear_cell.x + dx, bear_cell.y + dy};
                obstacle_cells_.insert(c);
            }
        }
        
        RCLCPP_WARN(this->get_logger(), "BEAR DETECTED at (%.2f, %.2f) - AVOIDING!",
                   bear_x_, bear_y_);
    }
    
    void statusCallback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "BEAR_DETECTED") {
            bear_detected_ = true;
            last_bear_time_ = this->now();
        }
    }
    
    // ========== MAP MANAGEMENT ==========
    
    Cell worldToGrid(double x, double y) {
        return Cell{
            static_cast<int>(std::floor(x / grid_resolution_)),
            static_cast<int>(std::floor(y / grid_resolution_))
        };
    }
    
    void updateMap() {
        if (!has_odom_ || !has_scan_) return;
        
        Cell current_cell = worldToGrid(current_x_, current_y_);
        visited_cells_.insert(current_cell);
        
        // Mark cells around current position as visited (drone body footprint)
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                Cell c{current_cell.x + dx, current_cell.y + dy};
                visited_cells_.insert(c);
                free_cells_.insert(c);
            }
        }
        
        // Update map based on LIDAR scan
        int ray_count = 0;
        for (size_t i = 0; i < scan_.ranges.size(); i += 5) {  // Sample every 5th ray for speed
            float range = scan_.ranges[i];
            if (!std::isfinite(range) || range < scan_.range_min || range > scan_.range_max) {
                continue;
            }
            
            float angle = scan_.angle_min + i * scan_.angle_increment + current_yaw_;
            double obs_x = current_x_ + range * std::cos(angle);
            double obs_y = current_y_ + range * std::sin(angle);
            
            Cell obs_cell = worldToGrid(obs_x, obs_y);
            
            // Mark as obstacle if detected
            if (range < scan_.range_max - 0.5) {  // Not max range
                obstacle_cells_.insert(obs_cell);
                // Mark area around obstacle
                for (int dx = -1; dx <= 1; ++dx) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        Cell c{obs_cell.x + dx, obs_cell.y + dy};
                        obstacle_cells_.insert(c);
                    }
                }
            }
            
            // Ray trace to mark free space
            rayTraceFreeSpace(current_cell, obs_cell);
            ray_count++;
        }
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Map: %zu visited, %zu free, %zu obstacles, %d rays traced",
                           visited_cells_.size(), free_cells_.size(), 
                           obstacle_cells_.size(), ray_count);
        
        // Publish occupancy grid
        publishMap();
    }
    
    void rayTraceFreeSpace(Cell start, Cell end) {
        // Bresenham's line algorithm
        int dx = std::abs(end.x - start.x);
        int dy = std::abs(end.y - start.y);
        int sx = (start.x < end.x) ? 1 : -1;
        int sy = (start.y < end.y) ? 1 : -1;
        int err = dx - dy;
        
        Cell current = start;
        while (true) {
            if (obstacle_cells_.find(current) == obstacle_cells_.end()) {
                free_cells_.insert(current);
            }
            
            if (current == end) break;
            
            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                current.x += sx;
            }
            if (e2 < dx) {
                err += dx;
                current.y += sy;
            }
        }
    }
    
    void publishMap() {
        if (visited_cells_.empty()) return;
        
        nav_msgs::msg::OccupancyGrid map;
        map.header.stamp = this->now();
        map.header.frame_id = "map";
        map.info.resolution = grid_resolution_;
        map.info.width = 200;
        map.info.height = 200;
        map.info.origin.position.x = current_x_ - (map.info.width * grid_resolution_ / 2.0);
        map.info.origin.position.y = current_y_ - (map.info.height * grid_resolution_ / 2.0);
        
        map.data.resize(map.info.width * map.info.height, -1);  // Unknown
        
        for (const auto& cell : free_cells_) {
            int mx = cell.x - static_cast<int>(map.info.origin.position.x / grid_resolution_);
            int my = cell.y - static_cast<int>(map.info.origin.position.y / grid_resolution_);
            if (mx >= 0 && mx < static_cast<int>(map.info.width) && 
                my >= 0 && my < static_cast<int>(map.info.height)) {
                map.data[my * map.info.width + mx] = 0;  // Free
            }
        }
        
        for (const auto& cell : obstacle_cells_) {
            int mx = cell.x - static_cast<int>(map.info.origin.position.x / grid_resolution_);
            int my = cell.y - static_cast<int>(map.info.origin.position.y / grid_resolution_);
            if (mx >= 0 && mx < static_cast<int>(map.info.width) && 
                my >= 0 && my < static_cast<int>(map.info.height)) {
                map.data[my * map.info.width + mx] = 100;  // Obstacle
            }
        }
        
        map_pub_->publish(map);
    }
    
    // ========== FRONTIER DETECTION ==========
    
    void updateFrontiers() {
        if (!has_odom_) return;
        
        frontiers_.clear();
        std::vector<Cell> frontier_cells;
        
        // Find frontier cells (free cells adjacent to unknown cells)
        for (const auto& free_cell : free_cells_) {
            if (isFrontierCell(free_cell)) {
                frontier_cells.push_back(free_cell);
            }
        }
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Found %zu frontier cells from %zu free cells", 
                           frontier_cells.size(), free_cells_.size());
        
        // Cluster frontier cells
        std::vector<std::vector<Cell>> clusters = clusterFrontiers(frontier_cells);
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Clustered into %zu frontier groups", clusters.size());
        
        // Convert clusters to frontier points with scores
        for (const auto& cluster : clusters) {
            if (static_cast<int>(cluster.size()) < frontier_cluster_size_) continue;
            
            // Calculate centroid
            double cx = 0.0, cy = 0.0;
            for (const auto& cell : cluster) {
                cx += cell.x * grid_resolution_;
                cy += cell.y * grid_resolution_;
            }
            cx /= cluster.size();
            cy /= cluster.size();
            
            // Calculate score (distance + cluster size)
            double dist = std::sqrt(std::pow(cx - current_x_, 2) + std::pow(cy - current_y_, 2));
            double score = cluster.size() * 10.0 - dist;  // Prefer larger, closer frontiers
            
            // Check if path is clear (no bears)
            if (bear_detected_ && (this->now() - last_bear_time_).seconds() < 30.0) {
                double bear_dist = std::sqrt(std::pow(cx - bear_x_, 2) + std::pow(cy - bear_y_, 2));
                if (bear_dist < bear_avoidance_distance_ * 2.0) {
                    score -= 1000.0;  // Heavy penalty for bear proximity
                }
            }
            
            Frontier f{cx, cy, target_altitude_, score, static_cast<int>(cluster.size())};
            frontiers_.push_back(f);
        }
        
        // Sort by score (highest first)
        std::sort(frontiers_.begin(), frontiers_.end(), 
                  [](const Frontier& a, const Frontier& b) { 
                      return a.score > b.score; 
                  });
        
        // Select best frontier as goal if in exploration mode
        if (exploration_mode_) {
            if (!frontiers_.empty()) {
                current_goal_.x = frontiers_[0].x;
                current_goal_.y = frontiers_[0].y;
                current_goal_.z = frontiers_[0].z;
                
                // Publish frontier marker
                geometry_msgs::msg::PointStamped frontier_msg;
                frontier_msg.header.stamp = this->now();
                frontier_msg.header.frame_id = "map";
                frontier_msg.point.x = current_goal_.x;
                frontier_msg.point.y = current_goal_.y;
                frontier_msg.point.z = current_goal_.z;
                frontier_pub_->publish(frontier_msg);
                
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                   "Frontier-based goal: (%.2f, %.2f) - score: %.1f, size: %d",
                                   current_goal_.x, current_goal_.y, frontiers_[0].score,
                                   frontiers_[0].cluster_size);
            } else {
                // No frontiers found - use spiral search pattern
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                   "No frontiers detected, using spiral search pattern");
                auto spiral_goal = generateSpiralGoal();
                current_goal_.x = spiral_goal.x;
                current_goal_.y = spiral_goal.y;
                current_goal_.z = spiral_goal.z;
            }
        }
    }
    
    bool isFrontierCell(const Cell& cell) {
        // Check 4-connected neighbors
        std::vector<Cell> neighbors = {
            {cell.x + 1, cell.y},
            {cell.x - 1, cell.y},
            {cell.x, cell.y + 1},
            {cell.x, cell.y - 1}
        };
        
        for (const auto& neighbor : neighbors) {
            if (free_cells_.find(neighbor) == free_cells_.end() &&
                obstacle_cells_.find(neighbor) == obstacle_cells_.end()) {
                return true;  // Adjacent to unknown cell
            }
        }
        return false;
    }
    
    std::vector<std::vector<Cell>> clusterFrontiers(const std::vector<Cell>& frontier_cells) {
        std::vector<std::vector<Cell>> clusters;
        std::unordered_set<Cell, CellHash> visited;
        
        for (const auto& cell : frontier_cells) {
            if (visited.find(cell) != visited.end()) continue;
            
            // BFS to find connected component
            std::vector<Cell> cluster;
            std::queue<Cell> queue;
            queue.push(cell);
            visited.insert(cell);
            
            while (!queue.empty()) {
                Cell current = queue.front();
                queue.pop();
                cluster.push_back(current);
                
                // Check 8-connected neighbors
                for (int dx = -1; dx <= 1; ++dx) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        if (dx == 0 && dy == 0) continue;
                        Cell neighbor{current.x + dx, current.y + dy};
                        
                        if (visited.find(neighbor) == visited.end() &&
                            std::find(frontier_cells.begin(), frontier_cells.end(), neighbor) != frontier_cells.end()) {
                            visited.insert(neighbor);
                            queue.push(neighbor);
                        }
                    }
                }
            }
            
            clusters.push_back(cluster);
        }
        
        return clusters;
    }
    
    // ========== SENSOR ANALYSIS ==========
    
    void analyzeSectors() {
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
    
    // ========== NAVIGATION AND CONTROL ==========
    
    geometry_msgs::msg::Point generateSpiralGoal() {
        // Generate outward spiral pattern from start position
        const double radius_increment = 5.0;  // Move out 5m each loop
        const int points_per_loop = 8;        // 8 points per circle
        
        spiral_index_++;
        
        int loop = spiral_index_ / points_per_loop;
        int point = spiral_index_ % points_per_loop;
        
        double radius = radius_increment * (loop + 1);
        double angle = (2.0 * M_PI * point) / points_per_loop;
        
        geometry_msgs::msg::Point goal;
        goal.x = start_x_ + radius * std::cos(angle);
        goal.y = start_y_ + radius * std::sin(angle);
        goal.z = target_altitude_;
        
        RCLCPP_INFO(this->get_logger(), "Generated spiral goal #%d: (%.2f, %.2f) at radius %.1fm",
                   spiral_index_, goal.x, goal.y, radius);
        
        return goal;
    }
    
    void navigate() {
        if (!has_odom_ || !has_scan_) return;
        
        geometry_msgs::msg::Twist cmd;
        
        // Enable exploration mode if no manual goal
        if (!has_manual_goal_ && !exploration_mode_) {
            exploration_mode_ = true;
            // Set initial goal upward to climb to altitude
            current_goal_.x = current_x_;
            current_goal_.y = current_y_;
            current_goal_.z = target_altitude_;
            RCLCPP_INFO(this->get_logger(), "Starting autonomous exploration - climbing to %.1fm", 
                       target_altitude_);
        }
        
        // Check if current goal is reached
        double dx = current_goal_.x - current_x_;
        double dy = current_goal_.y - current_y_;
        double dz = current_goal_.z - current_z_;
        double dist_to_goal = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        if (dist_to_goal < goal_tolerance_) {
            RCLCPP_INFO(this->get_logger(), "Goal reached at (%.2f, %.2f, %.2f)!", 
                       current_x_, current_y_, current_z_);
            
            if (exploration_mode_) {
                // Generate new goal immediately - don't stop!
                if (!frontiers_.empty()) {
                    // Use next frontier
                    current_goal_.x = frontiers_[0].x;
                    current_goal_.y = frontiers_[0].y;
                    current_goal_.z = target_altitude_;
                    RCLCPP_INFO(this->get_logger(), "Continuing to next frontier: (%.2f, %.2f)",
                               current_goal_.x, current_goal_.y);
                } else {
                    // Use spiral pattern
                    auto spiral_goal = generateSpiralGoal();
                    current_goal_.x = spiral_goal.x;
                    current_goal_.y = spiral_goal.y;
                    current_goal_.z = spiral_goal.z;
                    RCLCPP_INFO(this->get_logger(), "Continuing spiral exploration to (%.2f, %.2f)",
                               current_goal_.x, current_goal_.y);
                }
            } else {
                // Manual goal reached - stop
                has_manual_goal_ = false;
                cmd.linear.x = 0.0;
                cmd.linear.y = 0.0;
                cmd.linear.z = 0.0;
                cmd_vel_pub_->publish(cmd);
                return;
            }
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
        
        // Calculate desired heading to goal
        double goal_angle = std::atan2(dy, dx);
        double angle_error = goal_angle - current_yaw_;
        while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
        while (angle_error < -M_PI) angle_error += 2.0 * M_PI;
        
        // Base velocity (proportional)
        double base_speed = std::min(cruise_speed_, dist_to_goal * 0.5);
        
        // Check obstacles
        double front_dist = sector_min_dist_[0];
        
        // 3-tier obstacle response
        if (front_dist < obstacle_distance_ || bear_dist < bear_avoidance_distance_) {
            // CRITICAL: Emergency avoidance
            if (bear_dist < bear_avoidance_distance_) {
                double bear_angle = std::atan2(bear_y_ - current_y_, bear_x_ - current_x_);
                double escape_angle = bear_angle + M_PI;
                cmd.linear.x = cruise_speed_ * 0.5 * std::cos(escape_angle - current_yaw_);
                cmd.linear.y = cruise_speed_ * 0.5 * std::sin(escape_angle - current_yaw_);
                cmd.angular.z = -angle_error * 2.0;
            } else {
                cmd.linear.x = -0.3;
                cmd.linear.y = findLateralEscape();
                cmd.angular.z = findBestDirection();
            }
        } else if (front_dist < safe_distance_) {
            // WARNING: Slow down and adjust
            double speed_factor = (front_dist - obstacle_distance_) / (safe_distance_ - obstacle_distance_);
            speed_factor = std::max(0.2, std::min(1.0, speed_factor));
            
            cmd.linear.x = base_speed * speed_factor * std::cos(angle_error);
            cmd.linear.y = base_speed * speed_factor * std::sin(angle_error) + findLateralEscape() * 0.3;
            cmd.angular.z = angle_error * 0.5 + findBestDirection() * 0.2;
        } else {
            // SAFE: Normal navigation
            cmd.linear.x = base_speed * std::cos(angle_error);
            cmd.linear.y = base_speed * std::sin(angle_error);
            cmd.angular.z = angle_error * 0.8;
        }
        
        // ========== HORIZONTAL STABILIZATION ==========
        // Use AGL for altitude control
        double altitude_error = 0.0;
        if (has_agl_) {
            altitude_error = target_altitude_ - agl_height_;
        } else {
            altitude_error = target_altitude_ - (current_z_ - 0.0);  // Assume ground at z=0
        }
        cmd.linear.z = altitude_error * 0.4;
        
        // Use IMU to maintain level flight (zero roll and pitch)
        if (has_imu_) {
            // Add corrective terms based on roll/pitch
            // If rolled left (negative roll), add rightward velocity
            cmd.linear.y -= current_roll_ * 0.3;
            // If pitched forward (positive pitch), reduce forward velocity
            cmd.linear.x -= current_pitch_ * 0.3;
        }
        
        // Clamp velocities
        cmd.linear.x = std::max(-cruise_speed_, std::min(cruise_speed_, cmd.linear.x));
        cmd.linear.y = std::max(-cruise_speed_*0.5, std::min(cruise_speed_*0.5, cmd.linear.y));
        cmd.linear.z = std::max(-0.5, std::min(0.5, cmd.linear.z));
        cmd.angular.z = std::max(-0.8, std::min(0.8, cmd.angular.z));
        
        cmd_vel_pub_->publish(cmd);
    }
    
    double findLateralEscape() {
        double left_space = (sector_min_dist_[2] + sector_min_dist_[3]) / 2.0;
        double right_space = (sector_min_dist_[6] + sector_min_dist_[7]) / 2.0;
        return (left_space > right_space) ? 0.5 : -0.5;
    }
    
    double findBestDirection() {
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
    
    // ========== MEMBER VARIABLES ==========
    
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr agl_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr bear_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr frontier_pub_;
    
    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr map_timer_;
    rclcpp::TimerBase::SharedPtr frontier_timer_;
    
    // State
    double current_x_ = 0.0, current_y_ = 0.0, current_z_ = 0.0;
    double current_yaw_ = 0.0, current_roll_ = 0.0, current_pitch_ = 0.0;
    double bear_x_ = 0.0, bear_y_ = 0.0, bear_z_ = 0.0;
    double agl_height_ = 0.0;
    
    double start_x_ = 0.0, start_y_ = 0.0;  // Starting position
    int spiral_index_ = 0;                   // For spiral search pattern
    
    Frontier current_goal_{0.0, 0.0, 0.0, 0.0, 0};
    
    bool has_odom_ = false;
    bool has_scan_ = false;
    bool has_agl_ = false;
    bool has_imu_ = false;
    bool has_manual_goal_ = false;
    bool bear_detected_ = false;
    bool exploration_mode_ = false;
    
    rclcpp::Time last_bear_time_{0, 0, RCL_ROS_TIME};
    
    // LIDAR data
    sensor_msgs::msg::LaserScan scan_;
    std::vector<double> sector_min_dist_;
    
    // Map data
    std::unordered_set<Cell, CellHash> visited_cells_;
    std::unordered_set<Cell, CellHash> free_cells_;
    std::unordered_set<Cell, CellHash> obstacle_cells_;
    std::vector<Frontier> frontiers_;
    
    // Parameters
    double cruise_speed_;
    double target_altitude_;
    double obstacle_distance_;
    double safe_distance_;
    double bear_avoidance_distance_;
    double goal_tolerance_;
    double grid_resolution_;
    double exploration_radius_;
    int frontier_cluster_size_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IntelligentExplorer>());
    rclcpp::shutdown();
    return 0;
}