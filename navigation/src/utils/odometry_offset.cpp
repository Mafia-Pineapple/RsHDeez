#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

class OdometryOffset : public rclcpp::Node {
public:
  OdometryOffset() : Node("odometry_offset") {
    // Declare parameters for the offset
    this->declare_parameter("offset_x", 0.0);
    this->declare_parameter("offset_y", 0.0);
    this->declare_parameter("offset_z", 0.0);
    
    // Subscribe to raw odometry from Gazebo bridge
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry_raw", 10,
        std::bind(&OdometryOffset::odomCallback, this, std::placeholders::_1));
    
    // Publish offset odometry
    pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry", 10);
    
    // TF broadcaster for odom->base_link
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    
    RCLCPP_INFO(this->get_logger(), "Odometry Offset Node started");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Store initial position on first message
    if (!initial_set_) {
      initial_x_ = msg->pose.pose.position.x;
      initial_y_ = msg->pose.pose.position.y;
      initial_z_ = msg->pose.pose.position.z;
      initial_set_ = true;
      
      // Override with parameters if provided
      this->get_parameter("offset_x", initial_x_);
      this->get_parameter("offset_y", initial_y_);
      this->get_parameter("offset_z", initial_z_);
      
      RCLCPP_INFO(this->get_logger(), 
                  "Initial offset set to: [%.2f, %.2f, %.2f]", 
                  initial_x_, initial_y_, initial_z_);
    }
    
    // Create offset odometry message
    nav_msgs::msg::Odometry offset_odom = *msg;
    offset_odom.pose.pose.position.x -= initial_x_;
    offset_odom.pose.pose.position.y -= initial_y_;
    offset_odom.pose.pose.position.z -= initial_z_;
    
    // Publish offset odometry
    pub_->publish(offset_odom);
    
    // Publish TF: odom -> base_link (with offset)
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = msg->header.stamp;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = offset_odom.pose.pose.position.x;
    tf.transform.translation.y = offset_odom.pose.pose.position.y;
    tf.transform.translation.z = offset_odom.pose.pose.position.z;
    tf.transform.rotation = msg->pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  bool initial_set_ = false;
  double initial_x_ = 0.0;
  double initial_y_ = 0.0;
  double initial_z_ = 0.0;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometryOffset>());
  rclcpp::shutdown();
  return 0;
}