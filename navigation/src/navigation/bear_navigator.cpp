#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <cmath>

class BearNavigator : public rclcpp::Node {
public:
    BearNavigator() : Node("bear_navigator") {
        // Parameters
        this->declare_parameter("cruise_speed", 0.8);
        this->declare_parameter("obstacle_distance", 3.0);
        this->declare_parameter("safe_distance", 5.0);
        this->declare_parameter("bear_avoidance_distance", 8.0);
        this->declare_parameter("goal_tolerance", 1.0);
        
        cruise_speed_ = this->get_parameter("cruise_speed").as_double();
        obstacle_distance_ = this->get_parameter("obstacle_distance").as_double();
        safe_distance_ = this->get_parameter("safe_distance").as_double();
        bear_avoidance_distance_ = this->get_parameter("bear_avoidance_distance").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
        
        // Subscribers
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry", 10,
            std::bind(&BearNavigator::odomCallback, this, std::placeholders::_1));
        
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&BearNavigator::scanCallback, this, std::placeholders::_1));
        
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/nav_goal_3d", 10,
            std::bind(&BearNavigator::goalCallback, this, std::placeholders::_1));
        
        bear_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/bear_detection", 10,
            std::bind(&BearNavigator::bearCallback, this, std::placeholders::_1));
        
        status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/detection_status", 10,
            std::bind(&BearNavigator::statusCallback, this, std::placeholders::_1));
        
        // Publishers
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // TF
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Control timer (20Hz)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&BearNavigator::navigate, this));
        
        RCLCPP_INFO(this->get_logger(), "Bear Navigator initialized");
        RCLCPP_INFO(this->get_logger(), "  - Cruise speed: %.2f m/s", cruise_speed_);
        RCLCPP_INFO(this->get_logger(), "  - Obstacle distance: %.2f m", obstacle_distance_);
        RCLCPP_INFO(this->get_logger(), "  - Bear avoidance distance: %.2f m", bear_avoidance_distance_);
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_z_ = msg->pose.pose.position.z;
        
        // Extract yaw from quaternion
        double qw = msg->pose.pose.orientation.w;
        double qz = msg->pose.pose.orientation.z;
        current_yaw_ = std::atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz);
        
        has_odom_ = true;
    }
    
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        scan_ = *msg;
        has_scan_ = true;
        
        // Analyze 8 sectors for collision avoidance
        analyzeSectors();
    }
    
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        goal_x_ = msg->point.x;
        goal_y_ = msg->point.y;
        goal_z_ = msg->point.z;
        has_goal_ = true;
        goal_reached_ = false;
        
        RCLCPP_INFO(this->get_logger(), "New 3D goal: (%.2f, %.2f, %.2f)",
                   goal_x_, goal_y_, goal_z_);
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
        // Divide LIDAR into 8 sectors (45° each)
        sector_min_dist_.clear();
        sector_min_dist_.resize(8, 999.0);
        
        for (size_t i = 0; i < scan_.ranges.size(); ++i) {
            float range = scan_.ranges[i];
            if (!std::isfinite(range) || range < scan_.range_min || range > scan_.range_max) {
                continue;
            }
            
            // Calculate angle
            float angle = scan_.angle_min + i * scan_.angle_increment;
            
            // Determine sector (0=front, 1=front-left, 2=left, etc.)
            int sector = static_cast<int>((angle + M_PI) / (M_PI / 4.0)) % 8;
            
            if (range < sector_min_dist_[sector]) {
                sector_min_dist_[sector] = range;
            }
        }
    }
    
    void navigate() {
        if (!has_odom_ || !has_scan_) return;
        
        geometry_msgs::msg::Twist cmd;
        
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
        
        // Check bear proximity (expires after 10 seconds)
        double bear_dist = 999.0;
        if (bear_detected_ && (this->now() - last_bear_time_).seconds() < 10.0) {
            double bear_dx = bear_x_ - current_x_;
            double bear_dy = bear_y_ - current_y_;
            bear_dist = std::sqrt(bear_dx*bear_dx + bear_dy*bear_dy);
        } else {
            bear_detected_ = false;
        }
        
        // Calculate desired velocity toward goal
        double goal_angle = std::atan2(dy, dx);
        double angle_error = goal_angle - current_yaw_;
        
        // Normalize angle to [-pi, pi]
        while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
        while (angle_error < -M_PI) angle_error += 2.0 * M_PI;
        
        // Base velocity (proportional to distance)
        double base_speed = std::min(cruise_speed_, dist_to_goal * 0.5);
        
        // Check obstacles in front
        double front_dist = sector_min_dist_[0];  // Front sector
        
        // 3-tier response system
        if (front_dist < obstacle_distance_ || bear_dist < bear_avoidance_distance_) {
            // CRITICAL: Emergency stop + escape
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "CRITICAL obstacle at %.2fm or bear at %.2fm - EMERGENCY STOP",
                               front_dist, bear_dist);
            
            if (bear_dist < bear_avoidance_distance_) {
                // Move away from bear
                double bear_angle = std::atan2(bear_y_ - current_y_, bear_x_ - current_x_);
                double escape_angle = bear_angle + M_PI;  // Opposite direction
                
                cmd.linear.x = -cruise_speed_ * 0.5 * std::cos(escape_angle - current_yaw_);
                cmd.linear.y = -cruise_speed_ * 0.5 * std::sin(escape_angle - current_yaw_);
                cmd.angular.z = angle_error * 1.5;  // Turn away
            } else {
                // Back away from obstacle
                cmd.linear.x = -0.3;
                cmd.linear.y = findLateralEscape();
                cmd.angular.z = findBestDirection();
            }
            
        } else if (front_dist < safe_distance_) {
            // WARNING: Slow down + adjust path
            double speed_factor = (front_dist - obstacle_distance_) / (safe_distance_ - obstacle_distance_);
            speed_factor = std::max(0.2, std::min(1.0, speed_factor));
            
            cmd.linear.x = base_speed * speed_factor * std::cos(angle_error);
            cmd.linear.y = base_speed * speed_factor * std::sin(angle_error) + findLateralEscape() * 0.5;
            cmd.angular.z = angle_error + findBestDirection() * 0.3;
            
        } else {
            // SAFE: Normal navigation
            cmd.linear.x = base_speed * std::cos(angle_error);
            cmd.linear.y = base_speed * std::sin(angle_error);
            cmd.angular.z = angle_error * 0.8;
        }
        
        // Altitude control (simple proportional)
        cmd.linear.z = (goal_z_ - current_z_) * 0.3;
        
        // Clamp velocities
        cmd.linear.x = std::max(-cruise_speed_, std::min(cruise_speed_, cmd.linear.x));
        cmd.linear.y = std::max(-cruise_speed_*0.5, std::min(cruise_speed_*0.5, cmd.linear.y));
        cmd.linear.z = std::max(-0.5, std::min(0.5, cmd.linear.z));
        cmd.angular.z = std::max(-0.5, std::min(0.5, cmd.angular.z));
        
        cmd_vel_pub_->publish(cmd);
    }
    
    double findLateralEscape() {
        // Find which side has more space
        double left_space = (sector_min_dist_[2] + sector_min_dist_[3]) / 2.0;  // Left
        double right_space = (sector_min_dist_[6] + sector_min_dist_[7]) / 2.0; // Right
        
        if (left_space > right_space) {
            return 0.5;  // Move left
        } else {
            return -0.5;  // Move right
        }
    }
    
    double findBestDirection() {
        // Find sector with most space
        int best_sector = 0;
        double max_dist = 0.0;
        
        for (int i = 0; i < 8; ++i) {
            if (sector_min_dist_[i] > max_dist) {
                max_dist = sector_min_dist_[i];
                best_sector = i;
            }
        }
        
        // Convert sector to angular correction
        double target_angle = (best_sector - 4) * (M_PI / 4.0);  // -pi to pi
        return target_angle * 0.5;
    }
    
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
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
    double goal_x_ = 0.0, goal_y_ = 0.0, goal_z_ = 0.0;
    double bear_x_ = 0.0, bear_y_ = 0.0, bear_z_ = 0.0;
    
    bool has_odom_ = false;
    bool has_scan_ = false;
    bool has_goal_ = false;
    bool goal_reached_ = false;
    bool bear_detected_ = false;
    
    rclcpp::Time last_bear_time_{0, 0, RCL_ROS_TIME};
    
    // LIDAR data
    sensor_msgs::msg::LaserScan scan_;
    std::vector<double> sector_min_dist_;
    
    // Parameters
    double cruise_speed_;
    double obstacle_distance_;
    double safe_distance_;
    double bear_avoidance_distance_;
    double goal_tolerance_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BearNavigator>());
    rclcpp::shutdown();
    return 0;
}