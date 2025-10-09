

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
using std::placeholders::_1;

class slam : public rclcpp::Node
{
  public:
    slam()
    : Node("slam_node")
    {
      sub_laser = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10, std::bind(&slam::topic_callback_laser, this, _1));

      sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", 10, std::bind(&slam::topic_callback_imu, this, _1));

      sub_odo = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry", 10, std::bind(&slam::topic_callback_odo, this, _1));
    }

  private:
    void topic_callback_laser(const sensor_msgs::msg::LaserScan::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Laser scan rec");
    }
    //, msg->ranges.c_str());
    void topic_callback_imu(const sensor_msgs::msg::Imu::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "IMU scan rec");
    }
    void topic_callback_odo(const nav_msgs::msg::Odometry::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "ODO scan rec");
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odo;
};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<slam>());
  rclcpp::shutdown();
  return 0;
}


