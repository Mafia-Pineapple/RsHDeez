#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class Simple3DNavigator : public rclcpp::Node {
public:
    Simple3DNavigator() : Node("simple_3d_navigator") {
        // Get odometry from RTAB-Map, not Gazebo!
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/rtabmap/odom", 10,
            std::bind(&Simple3DNavigator::odomCallback, this, std::placeholders::_1));
        
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&Simple3DNavigator::scanCallback, this, std::placeholders::_1));
        
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/nav_goal_3d", 10,
            std::bind(&Simple3DNavigator::goalCallback, this, std::placeholders::_1));
        
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // TF listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Control timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&Simple3DNavigator::navigate, this));
        
        RCLCPP_INFO(this->get_logger(), "3D Navigator using RTAB-Map odometry (NO GAZEBO!)");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose_ = msg->pose.pose;
        has_odom_ = true;
    }
    
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Process LIDAR for obstacle avoidance
        // (same as your smooth_explorer collision avoidance)
    }
    
    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        goal_ = msg->point;
        has_goal_ = true;
        
        RCLCPP_INFO(this->get_logger(), "New 3D goal: (%.2f, %.2f, %.2f)",
                   goal_.x, goal_.y, goal_.z);
    }
    
    void navigate() {
        if (!has_odom_ || !has_goal_) return;
        
        // Simple 3D proportional navigation
        double dx = goal_.x - current_pose_.position.x;
        double dy = goal_.y - current_pose_.position.y;
        double dz = goal_.z - current_pose_.position.z;
        
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        if (dist < 1.0) {
            // Goal reached
            geometry_msgs::msg::Twist cmd;
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.linear.z = 0.0;
            cmd_vel_pub_->publish(cmd);
            has_goal_ = false;
            return;
        }
        
        // Proportional control with collision avoidance
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = std::max(-1.0, std::min(1.0, dx * 0.5));
        cmd.linear.y = std::max(-0.5, std::min(0.5, dy * 0.5));
        cmd.linear.z = std::max(-0.5, std::min(0.5, dz * 0.3));
        
        // Apply obstacle avoidance (from your smooth_explorer)
        // ...
        
        cmd_vel_pub_->publish(cmd);
    }
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    geometry_msgs::msg::Pose current_pose_;
    geometry_msgs::msg::Point goal_;
    bool has_odom_ = false;
    bool has_goal_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Simple3DNavigator>());
    rclcpp::shutdown();
    return 0;
}