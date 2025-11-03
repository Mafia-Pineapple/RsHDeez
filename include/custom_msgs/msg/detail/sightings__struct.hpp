// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_msgs:msg/Sightings.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__SIGHTINGS__STRUCT_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__SIGHTINGS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_msgs__msg__Sightings __attribute__((deprecated))
#else
# define DEPRECATED__custom_msgs__msg__Sightings __declspec(deprecated)
#endif

namespace custom_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Sightings_
{
  using Type = Sightings_<ContainerAllocator>;

  explicit Sightings_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->save = true;
      this->save_dir = "/tmp/sightings.csv";
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->sighting_count = 0ul;
      this->save = false;
      this->save_dir = "";
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sighting_count = 0ul;
    }
  }

  explicit Sightings_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : save_dir(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->save = true;
      this->save_dir = "/tmp/sightings.csv";
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->sighting_count = 0ul;
      this->save = false;
      this->save_dir = "";
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->sighting_count = 0ul;
    }
  }

  // field types and members
  using _sighting_count_type =
    uint32_t;
  _sighting_count_type sighting_count;
  using _animal_type_type =
    std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>>;
  _animal_type_type animal_type;
  using _x_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _x_type x;
  using _y_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _y_type y;
  using _z_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _z_type z;
  using _save_type =
    bool;
  _save_type save;
  using _save_dir_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _save_dir_type save_dir;

  // setters for named parameter idiom
  Type & set__sighting_count(
    const uint32_t & _arg)
  {
    this->sighting_count = _arg;
    return *this;
  }
  Type & set__animal_type(
    const std::vector<uint16_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint16_t>> & _arg)
  {
    this->animal_type = _arg;
    return *this;
  }
  Type & set__x(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->z = _arg;
    return *this;
  }
  Type & set__save(
    const bool & _arg)
  {
    this->save = _arg;
    return *this;
  }
  Type & set__save_dir(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->save_dir = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_msgs::msg::Sightings_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_msgs::msg::Sightings_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_msgs::msg::Sightings_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_msgs::msg::Sightings_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::Sightings_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::Sightings_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_msgs::msg::Sightings_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_msgs::msg::Sightings_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_msgs::msg::Sightings_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_msgs::msg::Sightings_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_msgs__msg__Sightings
    std::shared_ptr<custom_msgs::msg::Sightings_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_msgs__msg__Sightings
    std::shared_ptr<custom_msgs::msg::Sightings_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Sightings_ & other) const
  {
    if (this->sighting_count != other.sighting_count) {
      return false;
    }
    if (this->animal_type != other.animal_type) {
      return false;
    }
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    if (this->save != other.save) {
      return false;
    }
    if (this->save_dir != other.save_dir) {
      return false;
    }
    return true;
  }
  bool operator!=(const Sightings_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Sightings_

// alias to use template instance with default allocator
using Sightings =
  custom_msgs::msg::Sightings_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__SIGHTINGS__STRUCT_HPP_
