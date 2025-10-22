

#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "sensor_msgs/msg/channel_float32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
using std::placeholders::_1;

using namespace std::chrono_literals;

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

      pub_nearest = this->create_publisher<sensor_msgs::msg::ChannelFloat32>("/slam/insights/nearest", 10);
      timer_ = this->create_wall_timer(250ms, std::bind(&slam::timer_callback, this));
    }
    float iadm = 0.5; //insights angle distance minimum

  private:
    void topic_callback_laser(const sensor_msgs::msg::LaserScan msg)
    {
      //RCLCPP_INFO(this->get_logger(), "Laser scan rec");
      laser_mtx.lock();
      rec_laser = msg.ranges;
      rec_increment = msg.angle_increment;
      laser_mtx.unlock();
    }
    //, msg->ranges.c_str());
    void topic_callback_imu(const sensor_msgs::msg::Imu msg) const
    {
      //RCLCPP_INFO(this->get_logger(), "IMU scan rec");
    }
    void topic_callback_odo(const nav_msgs::msg::Odometry msg) const
    {
      //RCLCPP_INFO(this->get_logger(), "ODO scan rec");
    }
    void timer_callback()
    {
      std::vector<std::array<float, 3>> smallthree;
      smallthree.reserve(2);
      slam_insight_nearest(smallthree);

      //publish data as 1d array
      sensor_msgs::msg::ChannelFloat32 array; //all i want is a 1d float32 array ros2 message
      array.name = "distance then rad packed";
      array.values.reserve(6);
      array.values[0] = smallthree[0][0];
      array.values[1] = smallthree[1][0];
      array.values[2] = smallthree[0][1];
      array.values[3] = smallthree[1][1];
      array.values[4] = smallthree[0][2];
      array.values[5] = smallthree[1][2];
      //std::cout << array.values[0] << std::endl;
      std::string message;
      for (int i = 0; i < 6; i++)
      {
        float num = array.values[i];
        //std::cout << num << std::endl;
        std::string msg = std::to_string(num);

        if (i % 2){ message = message + "radian, "; }
        else      { message = message + "meters, "; }

        message = message + msg + '\n';
      }
      //std::cout << message << std::endl;
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.c_str());
      pub_nearest->publish(array);
    }
    void slam_insight_nearest(std::vector<std::array<float, 3>>& smallthree)
    {
      //acquire rec data
      laser_mtx.lock();
      auto l_laser = rec_laser;
      auto l_increment = rec_increment;
      laser_mtx.unlock();
      //find 3 largest points, separated by more than 0.349066 rads (20 deg) away because objects are more than 1 scan point big

      /*
      {near1 near2 near3}
      {angle1 angle2 angle3}
      */

      smallthree[0][0] = std::numeric_limits<float>::max();
      smallthree[0][1] = std::numeric_limits<float>::max();
      smallthree[0][2] = std::numeric_limits<float>::max();
      smallthree[1][0] = std::numeric_limits<float>::max();
      smallthree[1][1] = std::numeric_limits<float>::max();
      smallthree[1][2] = std::numeric_limits<float>::max();

      for (size_t i = 0; i < l_laser.size(); i++)
      {
        if (l_laser[i] < smallthree[0][0])
        {
          smallthree[0][0] = l_laser[i];
          smallthree[1][0] = l_increment * i; 
        }
      }
      //now have closest object, find next closest object
      for (size_t i = 0; i < l_laser.size(); i++)
      {
        if (l_laser[i] < smallthree[0][1])
        {
          //check that we're not currently same distance as l_laser AND further than 20 deg away
          if(l_laser[i] != smallthree[0][0] && abs(smallthree[1][0] - (i * l_increment)) > iadm)
          {
            smallthree[0][1] = l_laser[i];
            smallthree[1][1] = l_increment * i; 
          }
        }
      }
      //AGAIN for third object 
      for (size_t i = 0; i < l_laser.size(); i++)
      {
        if (l_laser[i] < smallthree[0][2])
        {
          //check that we're not currently same distance as scan 1 and 2 AND further than 20 deg away for both
          if(l_laser[i] != smallthree[0][0] && abs(smallthree[1][0] - (i * l_increment)) > iadm &&
             l_laser[i] != smallthree[0][1] && abs(smallthree[1][1] - (i * l_increment)) > iadm)
          {
            smallthree[0][2] = l_laser[i];
            smallthree[1][2] = l_increment * i; 
          }
        }
      }
    }
    
    
    std::vector<float> rec_laser;
    float rec_increment;
    std::mutex laser_mtx;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odo;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::ChannelFloat32>::SharedPtr pub_nearest;
};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<slam>());
  rclcpp::shutdown();
  return 0;
}


