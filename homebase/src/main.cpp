#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <mutex>
#include <cctype>

#include "curl/curl.h"

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
#include "custom_msgs/srv/weather_service.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
#include <iostream>
#include <fstream>

using namespace std::chrono_literals;

struct MemoryStruct
{
  char *memory;
  size_t size;
};

static size_t write_cb(void *contents, size_t size, size_t nmemb,
                       void *userp)
{
  size_t realsize = size * nmemb;
  struct MemoryStruct *mem = (struct MemoryStruct *)userp;

  char *ptr = (char *)realloc(mem->memory, mem->size + realsize + 1);
  if (!ptr)
  {
    /* out of memory! */
    printf("not enough memory (realloc returned NULL)\n");
    return 0;
  }

  mem->memory = ptr;
  memcpy(&(mem->memory[mem->size]), contents, realsize);
  mem->size += realsize;
  mem->memory[mem->size] = 0;

  return realsize;
}

class homebase : public rclcpp::Node
{
public:
  homebase()
      : Node("homebase_node")
  {
    // sub_laser = this->create_subscription<sensor_msgs::msg::LaserScan>(
    // "/scan", 10, std::bind(&homebase::topic_callback_laser, this, _1));

    sub_sightings = this->create_subscription<custom_msgs::msg::Sightings>("/homebase/sightings", 10, std::bind(&homebase::topic_callback_sightings, this, _1));

    pub_xyz = this->create_publisher<custom_msgs::msg::Sightings>("/homebase/sightings", 10);

    sub_example = this->create_subscription<std_msgs::msg::Bool>("/example/topic", 10, std::bind(&homebase::topic_do_example, this, _1));

    srv_weather = this->create_service<custom_msgs::srv::WeatherService>("homebase_weather_report", std::bind(&homebase::handle_weather_request, this, _1, _2 ,_3));
    // pub_nearest = this->create_publisher<sensor_msgs::msg::ChannelFloat32>(
    // "/slam/insights/nearest", 10);
    timer_ = this->create_wall_timer(250ms, std::bind(&homebase::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Homebase system started");
  }
  
  float iadm = 0.5; // insights angle distance minimum

private:
  // void topic_callback_laser(const sensor_msgs::msg::LaserScan msg)
  // {
  //   //RCLCPP_INFO(this->get_logger(), "Laser scan rec");
  //   laser_mtx.lock();
  //   rec_laser = msg.ranges;
  //   rec_increment = msg.angle_increment;
  //   laser_mtx.unlock();
  // }

  

  void handle_weather_request(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<custom_msgs::srv::WeatherService::Request> request,
            std::shared_ptr<custom_msgs::srv::WeatherService::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Weather request received for lat: %f, lon: %f", request->lat, request->lon);
    CURL *curl = curl_easy_init();
    std::string base_url = "https://api.open-meteo.com/v1/forecast?latitude=" + std::to_string(request->lat) + "&longitude=" + std::to_string(request->lon) + "&current=temperature_2m,apparent_temperature,relative_humidity_2m,precipitation,weather_code,wind_speed_10m,wind_direction_10m&timezone=Australia%2FSydney";
    if (curl)
    {
      
      /* send all data to this function  */
      curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_cb);

    
      /* some servers do not like requests that are made without a user-agent
          field, so we provide one */
      curl_easy_setopt(curl, CURLOPT_USERAGENT, "libcurl-agent/1.0");
      CURLcode res;
      /* we pass our 'chunk' struct to the callback function */
      struct MemoryStruct chunk;
      curl_easy_setopt(curl, CURLOPT_WRITEDATA, (void *)&chunk);
 
      res = curl_global_init(CURL_GLOBAL_ALL);
 
      chunk.memory = (char*)malloc(1);  /* grown as needed by the realloc above */
      chunk.size = 0;    /* no data at this point */
      curl_easy_setopt(curl, CURLOPT_URL, base_url.c_str());
      res = curl_easy_perform(curl);

      /* check for errors */
      if (res != CURLE_OK)
      {
        fprintf(stderr, "curl_easy_perform() failed: %s\n",
                curl_easy_strerror(res));
      }
      else
      {
        /*
         * Now, our chunk.memory points to a memory block that is chunk.size
         * bytes big and contains the remote file.
         *
         * Do something nice with it!
         */
        auto result = std::string(chunk.memory); // convert to string
        //find weather variable

        auto current_weather = std::strstr(result.c_str(),"\"current\":");
        //example response: "current":{"time":"2025-11-05T17:00","interval":900,"temperature_2m":17.6,"apparent_temperature":13.9,"relative_humidity_2m":39,"precipitation":0.00,"weather_code":0,"wind_speed_10m":15.3,"wind_direction_10m":262}}
        //extract temperature
        response->temperature = static_cast<int32_t>(extract_number_from_json(std::strstr(current_weather, "\"temperature_2m\":") + 17));
        response->wind = extract_number_from_json(std::strstr(current_weather, "\"wind_speed_10m\":") + 17);
        response->direction = static_cast<uint8_t>(extract_number_from_json(std::strstr(current_weather, "\"wind_direction_10m\":") + 21));
        response->weather_type = static_cast<uint8_t>(extract_number_from_json(std::strstr(current_weather, "\"weather_code\":") + 15));
        // for simplicity, just a few weather types
        switch (response->weather_type) {
    case 0:
        response->weather_message = "Clear sky";
        break;
    case 1:
    case 2:
    case 3:
        response->weather_message = "Mainly clear, partly cloudy, and overcast";
        break;
    case 45:
    case 48:
        response->weather_message = "Fog and depositing rime fog";
        break;
    case 51:
    case 53:
    case 55:
        response->weather_message = "Drizzle: Light, moderate, and dense intensity";
        break;
    case 56:
    case 57:
        response->weather_message = "Freezing Drizzle: Light and dense intensity";
        break;
    case 61:
    case 63:
    case 65:
        response->weather_message = "Rain: Slight, moderate and heavy intensity";
        break;
    case 66:
    case 67:
        response->weather_message = "Freezing Rain: Light and heavy intensity";
        break;
    case 71:
    case 73:
    case 75:
        response->weather_message = "Snow fall: Slight, moderate, and heavy intensity";
        break;
    case 77:
        response->weather_message = "Snow grains";
        break;
    case 80:
    case 81:
    case 82:
        response->weather_message = "Rain showers: Slight, moderate, and violent";
        break;
    case 85:
    case 86:
        response->weather_message = "Snow showers slight and heavy";
        break;
    case 95:
        response->weather_message = "Thunderstorm: Slight or moderate";
        break;
    case 96:
    case 99:
        response->weather_message = "Thunderstorm with slight and heavy hail";
        break;
    default:
        response->weather_message = "Unknown weather condition";
        break;
}
        std::cout << "Received data: " << current_weather << std::endl;
        printf("%lu bytes retrieved\n", (unsigned long)chunk.size);
      }
      curl_easy_cleanup(curl);
    }
    response->temperature = 23;
  }

  float extract_number_from_json(const char* json_ptr)
  {
    std::string num_str;
    int i = 0;
    while (i < 19)
    {
      num_str.push_back(json_ptr[i]);
      
      i++;
    }
    std::cout << num_str << std::endl;
    return std::stof(num_str);
  }

  void topic_do_example(const std_msgs::msg::Bool msg)
  {
    RCLCPP_INFO(this->get_logger(), "Example topic recieved %s", msg.data);
    
    // construct example message
    custom_msgs::msg::Sightings message;
    message.animal_type.push_back(1);
    message.x.push_back(1.0);
    message.y.push_back(2.0);
    message.z.push_back(0.0);
    message.sighting_count = 1;
    message.save = true;
    message.save_dir = "/tmp/example.csv";
    // publish example message
    pub_xyz->publish(message);
  }

  void topic_callback_sightings(const custom_msgs::msg::Sightings msg)
  {
    RCLCPP_INFO(this->get_logger(), "Sightings recieved: %d", msg.sighting_count);
    // only if save=true
    if (msg.save)
    {
      std::ofstream file;
      file.open(msg.save_dir, std::ios_base::app); // append mode
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
  // listen to publish example
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_example;
  rclcpp::Publisher<custom_msgs::msg::Sightings>::SharedPtr pub_xyz;

  rclcpp::Subscription<custom_msgs::msg::Sightings>::SharedPtr sub_sightings;
  rclcpp::Service<custom_msgs::srv::WeatherService>::SharedPtr srv_weather;
  // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_xyz;

  // Weather reporting service
  

  // std::vector<std::pair<std::vector<float>, nav_msgs::msg::Odometry>> scan_associated;
  // std::mutex scan_assoc_mtx;

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

// i grabbed the next two functions from the curl examples online
//  https://curl.se/libcurl/c/getinmemory.html


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<homebase>());
  rclcpp::shutdown();
  return 0;
}
