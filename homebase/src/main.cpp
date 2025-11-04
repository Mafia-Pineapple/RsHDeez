

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
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "custom_msgs/msg/sightings.hpp"
using std::placeholders::_1;

#include <iostream>
#include <fstream>

using namespace std::chrono_literals;

class homebase : public rclcpp::Node
{
  public:
    homebase()
    : Node("homebase_node")
    {
      // sub_laser = this->create_subscription<sensor_msgs::msg::LaserScan>(
      // "/scan", 10, std::bind(&homebase::topic_callback_laser, this, _1));

      sub_sightings = this->create_subscription<custom_msgs::msg::Sightings>(
      "/homebase/sightings", 10, std::bind(&homebase::topic_callback_sightings, this, _1));

      // pub_nearest = this->create_publisher<sensor_msgs::msg::ChannelFloat32>(
      // "/slam/insights/nearest", 10);
      timer_ = this->create_wall_timer(250ms, std::bind(&homebase::timer_callback, this));
    }
    float iadm = 0.5; //insights angle distance minimum

  private:
    // void topic_callback_laser(const sensor_msgs::msg::LaserScan msg)
    // {
    //   //RCLCPP_INFO(this->get_logger(), "Laser scan rec");
    //   laser_mtx.lock();
    //   rec_laser = msg.ranges;
    //   rec_increment = msg.angle_increment;
    //   laser_mtx.unlock();
    // }

    void topic_callback_sightings(const custom_msgs::msg::Sightings msg)
    {
      RCLCPP_INFO(this->get_logger(), "Sightings recieved: %d", msg.sighting_count);
      //only if save=true
      if (msg.save)
      {
        std::ofstream file;
        file.open(msg.save_dir, std::ios_base::app); //append mode
        for (uint32_t i = 0; i < msg.sighting_count; i++)
        {
          file << msg.animal_type[i] << "," << msg.x[i] << "," << msg.y[i] << "," << msg.z[i] << "\n";
        }
        file.close();
        RCLCPP_INFO(this->get_logger(), "Sightings saved to %s", msg.save_dir.c_str());
      }
    }
    
    void timer_callback()
    {
      
    }
    //message request boolean
    rclcpp::Subscription<custom_msgs::msg::Sightings>::SharedPtr sub_sightings;
    //rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_xyz;

    std::vector<std::pair<std::vector<float>, nav_msgs::msg::Odometry>> scan_associated;  
    std::mutex scan_assoc_mtx; 


    // std::vector<float> rec_laser;
    // float rec_increment;
    // std::mutex laser_mtx;

    // rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser;
    // rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;

    // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odo;
    // nav_msgs::msg::Odometry rec_odo;
    // std::mutex odo_mtx;

    rclcpp::TimerBase::SharedPtr timer_;
    // rclcpp::Publisher<sensor_msgs::msg::ChannelFloat32>::SharedPtr pub_nearest;
};


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<homebase>());
  rclcpp::shutdown();
  return 0;
}


