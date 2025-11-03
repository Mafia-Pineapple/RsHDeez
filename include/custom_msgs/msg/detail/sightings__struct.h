// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_msgs:msg/Sightings.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__SIGHTINGS__STRUCT_H_
#define CUSTOM_MSGS__MSG__DETAIL__SIGHTINGS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'animal_type'
// Member 'x'
// Member 'y'
// Member 'z'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'save_dir'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Sightings in the package custom_msgs.
typedef struct custom_msgs__msg__Sightings
{
  uint32_t sighting_count;
  rosidl_runtime_c__uint16__Sequence animal_type;
  /// these are in global frame
  rosidl_runtime_c__double__Sequence x;
  rosidl_runtime_c__double__Sequence y;
  rosidl_runtime_c__double__Sequence z;
  bool save;
  rosidl_runtime_c__String save_dir;
} custom_msgs__msg__Sightings;

// Struct for a sequence of custom_msgs__msg__Sightings.
typedef struct custom_msgs__msg__Sightings__Sequence
{
  custom_msgs__msg__Sightings * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_msgs__msg__Sightings__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MSGS__MSG__DETAIL__SIGHTINGS__STRUCT_H_
