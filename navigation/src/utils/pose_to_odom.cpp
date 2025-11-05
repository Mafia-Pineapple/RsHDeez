#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

class PoseToOdom : public rclcpp::Node {
public:
  PoseToOdom() : Node("pose_to_odom") {
    sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/drone/pose", 10,
        std::bind(&PoseToOdom::poseCallback, this, std::placeholders::_1));
    pub_ = create_publisher<nav_msgs::msg::Odometry>("/drone/odom3d", 10);
  }

private:
  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    rclcpp::Time now = msg->header.stamp;
    double dt = (last_time_.nanoseconds() == 0)
                    ? 0.0
                    : (now - last_time_).seconds();
    last_time_ = now;

    double vx=0, vy=0, vz=0;
    if (has_prev_ && dt > 1e-6) {
      vx = (msg->pose.position.x - prev_[0]) / dt;
      vy = (msg->pose.position.y - prev_[1]) / dt;
      vz = (msg->pose.position.z - prev_[2]) / dt;
    }
    prev_[0] = msg->pose.position.x;
    prev_[1] = msg->pose.position.y;
    prev_[2] = msg->pose.position.z;
    has_prev_ = true;

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose = msg->pose;
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.linear.z = vz;
    pub_->publish(odom);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Time last_time_{0,0,get_clock()->get_clock_type()};
  bool has_prev_ = false;
  double prev_[3]{0,0,0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PoseToOdom>());
  rclcpp::shutdown();
  return 0;
}
