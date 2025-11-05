// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_msgs:srv/WeatherService.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__SRV__DETAIL__WEATHER_SERVICE__STRUCT_HPP_
#define CUSTOM_MSGS__SRV__DETAIL__WEATHER_SERVICE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_msgs__srv__WeatherService_Request __attribute__((deprecated))
#else
# define DEPRECATED__custom_msgs__srv__WeatherService_Request __declspec(deprecated)
#endif

namespace custom_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct WeatherService_Request_
{
  using Type = WeatherService_Request_<ContainerAllocator>;

  explicit WeatherService_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->lat = -33.725117;
      this->lon = 150.320997;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->lat = 0.0;
      this->lon = 0.0;
    }
  }

  explicit WeatherService_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->lat = -33.725117;
      this->lon = 150.320997;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->lat = 0.0;
      this->lon = 0.0;
    }
  }

  // field types and members
  using _lat_type =
    double;
  _lat_type lat;
  using _lon_type =
    double;
  _lon_type lon;

  // setters for named parameter idiom
  Type & set__lat(
    const double & _arg)
  {
    this->lat = _arg;
    return *this;
  }
  Type & set__lon(
    const double & _arg)
  {
    this->lon = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_msgs::srv::WeatherService_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_msgs::srv::WeatherService_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_msgs::srv::WeatherService_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_msgs::srv::WeatherService_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_msgs::srv::WeatherService_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::srv::WeatherService_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_msgs::srv::WeatherService_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::srv::WeatherService_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_msgs::srv::WeatherService_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_msgs::srv::WeatherService_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_msgs__srv__WeatherService_Request
    std::shared_ptr<custom_msgs::srv::WeatherService_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_msgs__srv__WeatherService_Request
    std::shared_ptr<custom_msgs::srv::WeatherService_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WeatherService_Request_ & other) const
  {
    if (this->lat != other.lat) {
      return false;
    }
    if (this->lon != other.lon) {
      return false;
    }
    return true;
  }
  bool operator!=(const WeatherService_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WeatherService_Request_

// alias to use template instance with default allocator
using WeatherService_Request =
  custom_msgs::srv::WeatherService_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace custom_msgs


#ifndef _WIN32
# define DEPRECATED__custom_msgs__srv__WeatherService_Response __attribute__((deprecated))
#else
# define DEPRECATED__custom_msgs__srv__WeatherService_Response __declspec(deprecated)
#endif

namespace custom_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct WeatherService_Response_
{
  using Type = WeatherService_Response_<ContainerAllocator>;

  explicit WeatherService_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->temperature = 20l;
      this->wind = 0.0f;
      this->direction = 0;
      this->weather_type = 0;
      this->weather_message = "Clear sky";
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->temperature = 0l;
      this->wind = 0.0f;
      this->direction = 0;
      this->weather_type = 0;
      this->weather_message = "";
    }
  }

  explicit WeatherService_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : weather_message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->temperature = 20l;
      this->wind = 0.0f;
      this->direction = 0;
      this->weather_type = 0;
      this->weather_message = "Clear sky";
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->temperature = 0l;
      this->wind = 0.0f;
      this->direction = 0;
      this->weather_type = 0;
      this->weather_message = "";
    }
  }

  // field types and members
  using _temperature_type =
    int32_t;
  _temperature_type temperature;
  using _wind_type =
    float;
  _wind_type wind;
  using _direction_type =
    unsigned char;
  _direction_type direction;
  using _weather_type_type =
    unsigned char;
  _weather_type_type weather_type;
  using _weather_message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _weather_message_type weather_message;

  // setters for named parameter idiom
  Type & set__temperature(
    const int32_t & _arg)
  {
    this->temperature = _arg;
    return *this;
  }
  Type & set__wind(
    const float & _arg)
  {
    this->wind = _arg;
    return *this;
  }
  Type & set__direction(
    const unsigned char & _arg)
  {
    this->direction = _arg;
    return *this;
  }
  Type & set__weather_type(
    const unsigned char & _arg)
  {
    this->weather_type = _arg;
    return *this;
  }
  Type & set__weather_message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->weather_message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_msgs::srv::WeatherService_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_msgs::srv::WeatherService_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_msgs::srv::WeatherService_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_msgs::srv::WeatherService_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_msgs::srv::WeatherService_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::srv::WeatherService_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_msgs::srv::WeatherService_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::srv::WeatherService_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_msgs::srv::WeatherService_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_msgs::srv::WeatherService_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_msgs__srv__WeatherService_Response
    std::shared_ptr<custom_msgs::srv::WeatherService_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_msgs__srv__WeatherService_Response
    std::shared_ptr<custom_msgs::srv::WeatherService_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WeatherService_Response_ & other) const
  {
    if (this->temperature != other.temperature) {
      return false;
    }
    if (this->wind != other.wind) {
      return false;
    }
    if (this->direction != other.direction) {
      return false;
    }
    if (this->weather_type != other.weather_type) {
      return false;
    }
    if (this->weather_message != other.weather_message) {
      return false;
    }
    return true;
  }
  bool operator!=(const WeatherService_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WeatherService_Response_

// alias to use template instance with default allocator
using WeatherService_Response =
  custom_msgs::srv::WeatherService_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace custom_msgs

namespace custom_msgs
{

namespace srv
{

struct WeatherService
{
  using Request = custom_msgs::srv::WeatherService_Request;
  using Response = custom_msgs::srv::WeatherService_Response;
};

}  // namespace srv

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__SRV__DETAIL__WEATHER_SERVICE__STRUCT_HPP_
