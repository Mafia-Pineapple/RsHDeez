// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msgs:srv/WeatherService.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__SRV__DETAIL__WEATHER_SERVICE__STRUCT_H_
#define CUSTOM_MSGS__SRV__DETAIL__WEATHER_SERVICE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/WeatherService in the package custom_msgs.
typedef struct custom_msgs__srv__WeatherService_Request
{
  double lat;
  double lon;
} custom_msgs__srv__WeatherService_Request;

// Struct for a sequence of custom_msgs__srv__WeatherService_Request.
typedef struct custom_msgs__srv__WeatherService_Request__Sequence
{
  custom_msgs__srv__WeatherService_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msgs__srv__WeatherService_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'weather_message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/WeatherService in the package custom_msgs.
typedef struct custom_msgs__srv__WeatherService_Response
{
  int32_t temperature;
  float wind;
  uint8_t direction;
  uint8_t weather_type;
  rosidl_runtime_c__String weather_message;
} custom_msgs__srv__WeatherService_Response;

// Struct for a sequence of custom_msgs__srv__WeatherService_Response.
typedef struct custom_msgs__srv__WeatherService_Response__Sequence
{
  custom_msgs__srv__WeatherService_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msgs__srv__WeatherService_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSGS__SRV__DETAIL__WEATHER_SERVICE__STRUCT_H_
