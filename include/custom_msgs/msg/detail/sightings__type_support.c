// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from custom_msgs:msg/Sightings.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "custom_msgs/msg/detail/sightings__rosidl_typesupport_introspection_c.h"
#include "custom_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "custom_msgs/msg/detail/sightings__functions.h"
#include "custom_msgs/msg/detail/sightings__struct.h"


// Include directives for member types
// Member `animal_type`
// Member `x`
// Member `y`
// Member `z`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `save_dir`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__Sightings_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_msgs__msg__Sightings__init(message_memory);
}

void custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__Sightings_fini_function(void * message_memory)
{
  custom_msgs__msg__Sightings__fini(message_memory);
}

size_t custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__size_function__Sightings__animal_type(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return member->size;
}

const void * custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__get_const_function__Sightings__animal_type(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint16__Sequence * member =
    (const rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void * custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__get_function__Sightings__animal_type(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  return &member->data[index];
}

void custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__fetch_function__Sightings__animal_type(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint16_t * item =
    ((const uint16_t *)
    custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__get_const_function__Sightings__animal_type(untyped_member, index));
  uint16_t * value =
    (uint16_t *)(untyped_value);
  *value = *item;
}

void custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__assign_function__Sightings__animal_type(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint16_t * item =
    ((uint16_t *)
    custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__get_function__Sightings__animal_type(untyped_member, index));
  const uint16_t * value =
    (const uint16_t *)(untyped_value);
  *item = *value;
}

bool custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__resize_function__Sightings__animal_type(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint16__Sequence * member =
    (rosidl_runtime_c__uint16__Sequence *)(untyped_member);
  rosidl_runtime_c__uint16__Sequence__fini(member);
  return rosidl_runtime_c__uint16__Sequence__init(member, size);
}

size_t custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__size_function__Sightings__x(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__get_const_function__Sightings__x(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__get_function__Sightings__x(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__fetch_function__Sightings__x(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__get_const_function__Sightings__x(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__assign_function__Sightings__x(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__get_function__Sightings__x(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__resize_function__Sightings__x(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__size_function__Sightings__y(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__get_const_function__Sightings__y(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__get_function__Sightings__y(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__fetch_function__Sightings__y(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__get_const_function__Sightings__y(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__assign_function__Sightings__y(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__get_function__Sightings__y(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__resize_function__Sightings__y(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__size_function__Sightings__z(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__get_const_function__Sightings__z(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__get_function__Sightings__z(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__fetch_function__Sightings__z(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__get_const_function__Sightings__z(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__assign_function__Sightings__z(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__get_function__Sightings__z(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__resize_function__Sightings__z(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__Sightings_message_member_array[7] = {
  {
    "sighting_count",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_msgs__msg__Sightings, sighting_count),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "animal_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_msgs__msg__Sightings, animal_type),  // bytes offset in struct
    NULL,  // default value
    custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__size_function__Sightings__animal_type,  // size() function pointer
    custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__get_const_function__Sightings__animal_type,  // get_const(index) function pointer
    custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__get_function__Sightings__animal_type,  // get(index) function pointer
    custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__fetch_function__Sightings__animal_type,  // fetch(index, &value) function pointer
    custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__assign_function__Sightings__animal_type,  // assign(index, value) function pointer
    custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__resize_function__Sightings__animal_type  // resize(index) function pointer
  },
  {
    "x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_msgs__msg__Sightings, x),  // bytes offset in struct
    NULL,  // default value
    custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__size_function__Sightings__x,  // size() function pointer
    custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__get_const_function__Sightings__x,  // get_const(index) function pointer
    custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__get_function__Sightings__x,  // get(index) function pointer
    custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__fetch_function__Sightings__x,  // fetch(index, &value) function pointer
    custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__assign_function__Sightings__x,  // assign(index, value) function pointer
    custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__resize_function__Sightings__x  // resize(index) function pointer
  },
  {
    "y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_msgs__msg__Sightings, y),  // bytes offset in struct
    NULL,  // default value
    custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__size_function__Sightings__y,  // size() function pointer
    custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__get_const_function__Sightings__y,  // get_const(index) function pointer
    custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__get_function__Sightings__y,  // get(index) function pointer
    custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__fetch_function__Sightings__y,  // fetch(index, &value) function pointer
    custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__assign_function__Sightings__y,  // assign(index, value) function pointer
    custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__resize_function__Sightings__y  // resize(index) function pointer
  },
  {
    "z",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_msgs__msg__Sightings, z),  // bytes offset in struct
    NULL,  // default value
    custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__size_function__Sightings__z,  // size() function pointer
    custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__get_const_function__Sightings__z,  // get_const(index) function pointer
    custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__get_function__Sightings__z,  // get(index) function pointer
    custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__fetch_function__Sightings__z,  // fetch(index, &value) function pointer
    custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__assign_function__Sightings__z,  // assign(index, value) function pointer
    custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__resize_function__Sightings__z  // resize(index) function pointer
  },
  {
    "save",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_msgs__msg__Sightings, save),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "save_dir",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_msgs__msg__Sightings, save_dir),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__Sightings_message_members = {
  "custom_msgs__msg",  // message namespace
  "Sightings",  // message name
  7,  // number of fields
  sizeof(custom_msgs__msg__Sightings),
  custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__Sightings_message_member_array,  // message members
  custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__Sightings_init_function,  // function to initialize message memory (memory has to be allocated)
  custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__Sightings_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__Sightings_message_type_support_handle = {
  0,
  &custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__Sightings_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_msgs, msg, Sightings)() {
  if (!custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__Sightings_message_type_support_handle.typesupport_identifier) {
    custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__Sightings_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &custom_msgs__msg__Sightings__rosidl_typesupport_introspection_c__Sightings_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
