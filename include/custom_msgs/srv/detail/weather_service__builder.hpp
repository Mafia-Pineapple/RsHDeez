// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:srv/WeatherService.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__SRV__DETAIL__WEATHER_SERVICE__BUILDER_HPP_
#define CUSTOM_MSGS__SRV__DETAIL__WEATHER_SERVICE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/srv/detail/weather_service__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace srv
{

namespace builder
{

class Init_WeatherService_Request_lon
{
public:
  explicit Init_WeatherService_Request_lon(::custom_msgs::srv::WeatherService_Request & msg)
  : msg_(msg)
  {}
  ::custom_msgs::srv::WeatherService_Request lon(::custom_msgs::srv::WeatherService_Request::_lon_type arg)
  {
    msg_.lon = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::srv::WeatherService_Request msg_;
};

class Init_WeatherService_Request_lat
{
public:
  Init_WeatherService_Request_lat()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WeatherService_Request_lon lat(::custom_msgs::srv::WeatherService_Request::_lat_type arg)
  {
    msg_.lat = std::move(arg);
    return Init_WeatherService_Request_lon(msg_);
  }

private:
  ::custom_msgs::srv::WeatherService_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::srv::WeatherService_Request>()
{
  return custom_msgs::srv::builder::Init_WeatherService_Request_lat();
}

}  // namespace custom_msgs


namespace custom_msgs
{

namespace srv
{

namespace builder
{

class Init_WeatherService_Response_weather_message
{
public:
  explicit Init_WeatherService_Response_weather_message(::custom_msgs::srv::WeatherService_Response & msg)
  : msg_(msg)
  {}
  ::custom_msgs::srv::WeatherService_Response weather_message(::custom_msgs::srv::WeatherService_Response::_weather_message_type arg)
  {
    msg_.weather_message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::srv::WeatherService_Response msg_;
};

class Init_WeatherService_Response_weather_type
{
public:
  explicit Init_WeatherService_Response_weather_type(::custom_msgs::srv::WeatherService_Response & msg)
  : msg_(msg)
  {}
  Init_WeatherService_Response_weather_message weather_type(::custom_msgs::srv::WeatherService_Response::_weather_type_type arg)
  {
    msg_.weather_type = std::move(arg);
    return Init_WeatherService_Response_weather_message(msg_);
  }

private:
  ::custom_msgs::srv::WeatherService_Response msg_;
};

class Init_WeatherService_Response_direction
{
public:
  explicit Init_WeatherService_Response_direction(::custom_msgs::srv::WeatherService_Response & msg)
  : msg_(msg)
  {}
  Init_WeatherService_Response_weather_type direction(::custom_msgs::srv::WeatherService_Response::_direction_type arg)
  {
    msg_.direction = std::move(arg);
    return Init_WeatherService_Response_weather_type(msg_);
  }

private:
  ::custom_msgs::srv::WeatherService_Response msg_;
};

class Init_WeatherService_Response_wind
{
public:
  explicit Init_WeatherService_Response_wind(::custom_msgs::srv::WeatherService_Response & msg)
  : msg_(msg)
  {}
  Init_WeatherService_Response_direction wind(::custom_msgs::srv::WeatherService_Response::_wind_type arg)
  {
    msg_.wind = std::move(arg);
    return Init_WeatherService_Response_direction(msg_);
  }

private:
  ::custom_msgs::srv::WeatherService_Response msg_;
};

class Init_WeatherService_Response_temperature
{
public:
  Init_WeatherService_Response_temperature()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WeatherService_Response_wind temperature(::custom_msgs::srv::WeatherService_Response::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_WeatherService_Response_wind(msg_);
  }

private:
  ::custom_msgs::srv::WeatherService_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::srv::WeatherService_Response>()
{
  return custom_msgs::srv::builder::Init_WeatherService_Response_temperature();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__SRV__DETAIL__WEATHER_SERVICE__BUILDER_HPP_
