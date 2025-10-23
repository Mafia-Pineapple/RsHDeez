

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
using std::placeholders::_1;

#include <iostream>
#include <fstream>

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

      sub_request = this->create_subscription<std_msgs::msg::Bool>(
      "/slam/cloud/request_data", 10, std::bind(&slam::topic_callback_data_request, this, _1));

      pub_xyz = this->create_publisher<sensor_msgs::msg::JointState>(
      "/slam/cloud/xyz", 10);

      pub_nearest = this->create_publisher<sensor_msgs::msg::ChannelFloat32>(
      "/slam/insights/nearest", 10);
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
    void topic_callback_odo(const nav_msgs::msg::Odometry msg)
    {
      //RCLCPP_INFO(this->get_logger(), "ODO scan rec");
      odo_mtx.lock();
      rec_odo = msg;
      odo_mtx.unlock();
    }
    void topic_callback_data_request(const std_msgs::msg::Bool msg) //TODO
    {
      if (msg.data)
      {
        RCLCPP_INFO(this->get_logger(), "Slam insight data requested");
        scan_assoc_mtx.lock();
        //local copy
        auto local_scan_assoc = scan_associated;
        auto local_rec_increment = rec_increment;
        scan_assoc_mtx.unlock();
        //publish as jointstate
        sensor_msgs::msg::JointState msg;
        msg.name.push_back("XYZ data");
        
        //std::ofstream myfile;
        //myfile.open("/home/james/git/RsHDeez/slam/nonsense.csv");
        for (int i = 0; i < local_scan_assoc.size(); i++)
        {
          auto scan = local_scan_assoc[i].first;
          auto odo = local_scan_assoc[i].second;
          std::vector<std::array<float, 2>> points_xy;
          //convert scan to local xyz
          for (size_t j = 0; j < scan.size(); j++)
          {
            if (scan[j] == std::numeric_limits<float>::infinity() || std::isnan(scan[j]))
            {
              continue; //skip invalid points
            }
            std::array<float, 2> point;
            float angle = j * local_rec_increment;
            point[0] = scan[j] * cos(angle);
            point[1] = scan[j] * sin(angle);
            //point[2] = 0.0; //2d lidar, z=0
            //z doesnt matter its always 0 in local-to-drone space
            points_xy.push_back(point);
          }
          
          float x_global, y_global, z_global;
          
          //transform to global using odo
          for (int j = 0; j < points_xy.size(); j++)
          {
            float x_local = points_xy[j][0];
            float y_local = points_xy[j][1];
            float z_local = 0;
            //convert xyz local to global using odometry data
            
            double orx = odo.pose.pose.orientation.x;
            double ory = odo.pose.pose.orientation.y;
            double orz = odo.pose.pose.orientation.z;
            double orw = odo.pose.pose.orientation.w;
            double opx = odo.pose.pose.position.x;
            double opy = odo.pose.pose.position.y;
            double opz = odo.pose.pose.position.z;
          
            //rotation matrix from quaternion
            double R11 = 1 - 2 * (ory * ory + orz * orz);
            double R12 = 2 * (orx * ory - orz * orw);
            double R13 = 2 * (orx * orz + ory * orw);
            double R21 = 2 * (orx * ory + orz * orw);
            double R22 = 1 - 2 * (orx * orx + orz * orz);
            double R23 = 2 * (ory * orz - orx * orw);
            double R31 = 2 * (orx * orz - ory * orw);
            double R32 = 2 * (ory * orz + orx * orw);
            double R33 = 1 - 2 * (orx * orx + ory * ory);

            x_global = R11 * x_local + R12 * y_local + R13 * z_local + opx;
            y_global = R21 * x_local + R22 * y_local + R23 * z_local + opy;
            z_global = R31 * x_local + R32 * y_local + R33 * z_local + opz;
            //myfile << x_global << ',' << y_global << ',' << z_global << '\n';
            msg.position.push_back(x_global);
            msg.velocity.push_back(y_global);
            msg.effort.push_back(z_global);
          }
        }
        pub_xyz->publish(msg);
        //myfile.close(); 
      }
    }
    void timer_callback()
    {
      std::vector<std::array<float, 3>> smallthree;
      smallthree.reserve(2);
      slam_insight_nearest(smallthree);

      //publish data as 1d array
      sensor_msgs::msg::ChannelFloat32 array; 
      array.name = "distance then rad repeating";
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


      //associate laser and odo datas
      laser_mtx.lock();
      odo_mtx.lock();
      scan_assoc_mtx.lock();

      scan_associated.push_back(std::make_pair(rec_laser, rec_odo));

      laser_mtx.unlock();
      odo_mtx.unlock();
      scan_assoc_mtx.unlock();
    }
    void slam_insight_nearest(std::vector<std::array<float, 3>>& smallthree)
    {
      //acquire rec data
      laser_mtx.lock();
      auto l_laser = rec_laser;
      auto l_increment = rec_increment;
      laser_mtx.unlock();
      //find 3 largest points, separated by more than 0.349066 rads (20 deg) away because objects are more than 1 scan point big or whatever variable i set it to

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
    

    //message request boolean
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_request;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_xyz;

    std::vector<std::pair<std::vector<float>, nav_msgs::msg::Odometry>> scan_associated;  
    std::mutex scan_assoc_mtx; 


    std::vector<float> rec_laser;
    float rec_increment;
    std::mutex laser_mtx;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odo;
    nav_msgs::msg::Odometry rec_odo;
    std::mutex odo_mtx;

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


