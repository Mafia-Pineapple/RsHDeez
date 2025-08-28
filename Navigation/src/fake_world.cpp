#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>

class FakeWorld : public rclcpp::Node {
public:
  FakeWorld() : Node("fake_world") {
    sub_cmd_ = create_subscription<geometry_msgs::msg::Twist>(
      "drone/cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist &msg){ last_cmd_ = msg; });

    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("drone/gt_odom", 10);

    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]{ tick(); }); // 50 Hz
    RCLCPP_INFO(get_logger(), "FakeWorld started (publishing /drone/gt_odom)");
  }

private:
  static void yawToQuat(double yaw, double &x, double &y, double &z, double &w) {
    // roll=pitch=0, yaw only
    const double half = yaw * 0.5;
    x = 0.0;
    y = 0.0;
    z = std::sin(half);
    w = std::cos(half);
  }

  void tick() {
    const double dt = 0.02; // 50 Hz

    // Integrate a super-simple kinematics model:
    // body->world approximation (no rotation of velocities for simplicity)
    x_ += last_cmd_.linear.x * dt;
    y_ += last_cmd_.linear.y * dt;
    z_ += last_cmd_.linear.z * dt;
    yaw_ += last_cmd_.angular.z * dt;

    nav_msgs::msg::Odometry odo;
    odo.header.stamp = now();
    odo.header.frame_id = "map";
    odo.child_frame_id  = "base_link";
    odo.pose.pose.position.x = x_;
    odo.pose.pose.position.y = y_;
    odo.pose.pose.position.z = z_;

    double qx, qy, qz, qw;
    yawToQuat(yaw_, qx, qy, qz, qw);
    odo.pose.pose.orientation.x = qx;
    odo.pose.pose.orientation.y = qy;
    odo.pose.pose.orientation.z = qz;
    odo.pose.pose.orientation.w = qw;

    pub_odom_->publish(odo);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Twist last_cmd_;
  double x_{0}, y_{0}, z_{0}, yaw_{0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeWorld>());
  rclcpp::shutdown();
  return 0;
}
