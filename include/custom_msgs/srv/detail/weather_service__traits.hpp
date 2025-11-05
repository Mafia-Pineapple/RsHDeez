// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_msgs:srv/WeatherService.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__SRV__DETAIL__WEATHER_SERVICE__TRAITS_HPP_
#define CUSTOM_MSGS__SRV__DETAIL__WEATHER_SERVICE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_msgs/srv/detail/weather_service__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace custom_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const WeatherService_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: lat
  {
    out << "lat: ";
    rosidl_generator_traits::value_to_yaml(msg.lat, out);
    out << ", ";
  }

  // member: lon
  {
    out << "lon: ";
    rosidl_generator_traits::value_to_yaml(msg.lon, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const WeatherService_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: lat
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lat: ";
    rosidl_generator_traits::value_to_yaml(msg.lat, out);
    out << "\n";
  }

  // member: lon
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lon: ";
    rosidl_generator_traits::value_to_yaml(msg.lon, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const WeatherService_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace custom_msgs

namespace rosidl_generator_traits
{

[[deprecated("use custom_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_msgs::srv::WeatherService_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const custom_msgs::srv::WeatherService_Request & msg)
{
  return custom_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<custom_msgs::srv::WeatherService_Request>()
{
  return "custom_msgs::srv::WeatherService_Request";
}

template<>
inline const char * name<custom_msgs::srv::WeatherService_Request>()
{
  return "custom_msgs/srv/WeatherService_Request";
}

template<>
struct has_fixed_size<custom_msgs::srv::WeatherService_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<custom_msgs::srv::WeatherService_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<custom_msgs::srv::WeatherService_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace custom_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const WeatherService_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: temperature
  {
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
    out << ", ";
  }

  // member: wind
  {
    out << "wind: ";
    rosidl_generator_traits::value_to_yaml(msg.wind, out);
    out << ", ";
  }

  // member: direction
  {
    out << "direction: ";
    rosidl_generator_traits::character_value_to_yaml(msg.direction, out);
    out << ", ";
  }

  // member: weather_type
  {
    out << "weather_type: ";
    rosidl_generator_traits::character_value_to_yaml(msg.weather_type, out);
    out << ", ";
  }

  // member: weather_message
  {
    out << "weather_message: ";
    rosidl_generator_traits::value_to_yaml(msg.weather_message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const WeatherService_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.temperature, out);
    out << "\n";
  }

  // member: wind
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wind: ";
    rosidl_generator_traits::value_to_yaml(msg.wind, out);
    out << "\n";
  }

  // member: direction
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "direction: ";
    rosidl_generator_traits::character_value_to_yaml(msg.direction, out);
    out << "\n";
  }

  // member: weather_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "weather_type: ";
    rosidl_generator_traits::character_value_to_yaml(msg.weather_type, out);
    out << "\n";
  }

  // member: weather_message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "weather_message: ";
    rosidl_generator_traits::value_to_yaml(msg.weather_message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const WeatherService_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace custom_msgs

namespace rosidl_generator_traits
{

[[deprecated("use custom_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_msgs::srv::WeatherService_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const custom_msgs::srv::WeatherService_Response & msg)
{
  return custom_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<custom_msgs::srv::WeatherService_Response>()
{
  return "custom_msgs::srv::WeatherService_Response";
}

template<>
inline const char * name<custom_msgs::srv::WeatherService_Response>()
{
  return "custom_msgs/srv/WeatherService_Response";
}

template<>
struct has_fixed_size<custom_msgs::srv::WeatherService_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<custom_msgs::srv::WeatherService_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<custom_msgs::srv::WeatherService_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<custom_msgs::srv::WeatherService>()
{
  return "custom_msgs::srv::WeatherService";
}

template<>
inline const char * name<custom_msgs::srv::WeatherService>()
{
  return "custom_msgs/srv/WeatherService";
}

template<>
struct has_fixed_size<custom_msgs::srv::WeatherService>
  : std::integral_constant<
    bool,
    has_fixed_size<custom_msgs::srv::WeatherService_Request>::value &&
    has_fixed_size<custom_msgs::srv::WeatherService_Response>::value
  >
{
};

template<>
struct has_bounded_size<custom_msgs::srv::WeatherService>
  : std::integral_constant<
    bool,
    has_bounded_size<custom_msgs::srv::WeatherService_Request>::value &&
    has_bounded_size<custom_msgs::srv::WeatherService_Response>::value
  >
{
};

template<>
struct is_service<custom_msgs::srv::WeatherService>
  : std::true_type
{
};

template<>
struct is_service_request<custom_msgs::srv::WeatherService_Request>
  : std::true_type
{
};

template<>
struct is_service_response<custom_msgs::srv::WeatherService_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_MSGS__SRV__DETAIL__WEATHER_SERVICE__TRAITS_HPP_
