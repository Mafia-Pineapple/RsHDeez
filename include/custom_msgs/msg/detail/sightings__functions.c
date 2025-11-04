// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_msgs:msg/Sightings.idl
// generated code does not contain a copyright notice
#include "custom_msgs/msg/detail/sightings__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `animal_type`
// Member `x`
// Member `y`
// Member `z`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `save_dir`
#include "rosidl_runtime_c/string_functions.h"

bool
custom_msgs__msg__Sightings__init(custom_msgs__msg__Sightings * msg)
{
  if (!msg) {
    return false;
  }
  // sighting_count
  // animal_type
  if (!rosidl_runtime_c__uint16__Sequence__init(&msg->animal_type, 0)) {
    custom_msgs__msg__Sightings__fini(msg);
    return false;
  }
  // x
  if (!rosidl_runtime_c__double__Sequence__init(&msg->x, 0)) {
    custom_msgs__msg__Sightings__fini(msg);
    return false;
  }
  // y
  if (!rosidl_runtime_c__double__Sequence__init(&msg->y, 0)) {
    custom_msgs__msg__Sightings__fini(msg);
    return false;
  }
  // z
  if (!rosidl_runtime_c__double__Sequence__init(&msg->z, 0)) {
    custom_msgs__msg__Sightings__fini(msg);
    return false;
  }
  // save
  msg->save = true;
  // save_dir
  if (!rosidl_runtime_c__String__init(&msg->save_dir)) {
    custom_msgs__msg__Sightings__fini(msg);
    return false;
  }
  {
    bool success = rosidl_runtime_c__String__assign(&msg->save_dir, "/tmp/sightings.csv");
    if (!success) {
      goto abort_init_0;
    }
  }
  return true;
abort_init_0:
  return false;
}

void
custom_msgs__msg__Sightings__fini(custom_msgs__msg__Sightings * msg)
{
  if (!msg) {
    return;
  }
  // sighting_count
  // animal_type
  rosidl_runtime_c__uint16__Sequence__fini(&msg->animal_type);
  // x
  rosidl_runtime_c__double__Sequence__fini(&msg->x);
  // y
  rosidl_runtime_c__double__Sequence__fini(&msg->y);
  // z
  rosidl_runtime_c__double__Sequence__fini(&msg->z);
  // save
  // save_dir
  rosidl_runtime_c__String__fini(&msg->save_dir);
}

bool
custom_msgs__msg__Sightings__are_equal(const custom_msgs__msg__Sightings * lhs, const custom_msgs__msg__Sightings * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // sighting_count
  if (lhs->sighting_count != rhs->sighting_count) {
    return false;
  }
  // animal_type
  if (!rosidl_runtime_c__uint16__Sequence__are_equal(
      &(lhs->animal_type), &(rhs->animal_type)))
  {
    return false;
  }
  // x
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->x), &(rhs->x)))
  {
    return false;
  }
  // y
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->y), &(rhs->y)))
  {
    return false;
  }
  // z
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->z), &(rhs->z)))
  {
    return false;
  }
  // save
  if (lhs->save != rhs->save) {
    return false;
  }
  // save_dir
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->save_dir), &(rhs->save_dir)))
  {
    return false;
  }
  return true;
}

bool
custom_msgs__msg__Sightings__copy(
  const custom_msgs__msg__Sightings * input,
  custom_msgs__msg__Sightings * output)
{
  if (!input || !output) {
    return false;
  }
  // sighting_count
  output->sighting_count = input->sighting_count;
  // animal_type
  if (!rosidl_runtime_c__uint16__Sequence__copy(
      &(input->animal_type), &(output->animal_type)))
  {
    return false;
  }
  // x
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->x), &(output->x)))
  {
    return false;
  }
  // y
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->y), &(output->y)))
  {
    return false;
  }
  // z
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->z), &(output->z)))
  {
    return false;
  }
  // save
  output->save = input->save;
  // save_dir
  if (!rosidl_runtime_c__String__copy(
      &(input->save_dir), &(output->save_dir)))
  {
    return false;
  }
  return true;
}

custom_msgs__msg__Sightings *
custom_msgs__msg__Sightings__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msgs__msg__Sightings * msg = (custom_msgs__msg__Sightings *)allocator.allocate(sizeof(custom_msgs__msg__Sightings), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_msgs__msg__Sightings));
  bool success = custom_msgs__msg__Sightings__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_msgs__msg__Sightings__destroy(custom_msgs__msg__Sightings * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_msgs__msg__Sightings__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_msgs__msg__Sightings__Sequence__init(custom_msgs__msg__Sightings__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msgs__msg__Sightings * data = NULL;

  if (size) {
    data = (custom_msgs__msg__Sightings *)allocator.zero_allocate(size, sizeof(custom_msgs__msg__Sightings), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_msgs__msg__Sightings__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_msgs__msg__Sightings__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
custom_msgs__msg__Sightings__Sequence__fini(custom_msgs__msg__Sightings__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      custom_msgs__msg__Sightings__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

custom_msgs__msg__Sightings__Sequence *
custom_msgs__msg__Sightings__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_msgs__msg__Sightings__Sequence * array = (custom_msgs__msg__Sightings__Sequence *)allocator.allocate(sizeof(custom_msgs__msg__Sightings__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_msgs__msg__Sightings__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_msgs__msg__Sightings__Sequence__destroy(custom_msgs__msg__Sightings__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_msgs__msg__Sightings__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_msgs__msg__Sightings__Sequence__are_equal(const custom_msgs__msg__Sightings__Sequence * lhs, const custom_msgs__msg__Sightings__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_msgs__msg__Sightings__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_msgs__msg__Sightings__Sequence__copy(
  const custom_msgs__msg__Sightings__Sequence * input,
  custom_msgs__msg__Sightings__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_msgs__msg__Sightings);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom_msgs__msg__Sightings * data =
      (custom_msgs__msg__Sightings *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_msgs__msg__Sightings__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom_msgs__msg__Sightings__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_msgs__msg__Sightings__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
