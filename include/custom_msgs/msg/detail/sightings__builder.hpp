// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_msgs:msg/Sightings.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__SIGHTINGS__BUILDER_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__SIGHTINGS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_msgs/msg/detail/sightings__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_msgs
{

namespace msg
{

namespace builder
{

class Init_Sightings_save_dir
{
public:
  explicit Init_Sightings_save_dir(::custom_msgs::msg::Sightings & msg)
  : msg_(msg)
  {}
  ::custom_msgs::msg::Sightings save_dir(::custom_msgs::msg::Sightings::_save_dir_type arg)
  {
    msg_.save_dir = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_msgs::msg::Sightings msg_;
};

class Init_Sightings_save
{
public:
  explicit Init_Sightings_save(::custom_msgs::msg::Sightings & msg)
  : msg_(msg)
  {}
  Init_Sightings_save_dir save(::custom_msgs::msg::Sightings::_save_type arg)
  {
    msg_.save = std::move(arg);
    return Init_Sightings_save_dir(msg_);
  }

private:
  ::custom_msgs::msg::Sightings msg_;
};

class Init_Sightings_z
{
public:
  explicit Init_Sightings_z(::custom_msgs::msg::Sightings & msg)
  : msg_(msg)
  {}
  Init_Sightings_save z(::custom_msgs::msg::Sightings::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_Sightings_save(msg_);
  }

private:
  ::custom_msgs::msg::Sightings msg_;
};

class Init_Sightings_y
{
public:
  explicit Init_Sightings_y(::custom_msgs::msg::Sightings & msg)
  : msg_(msg)
  {}
  Init_Sightings_z y(::custom_msgs::msg::Sightings::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Sightings_z(msg_);
  }

private:
  ::custom_msgs::msg::Sightings msg_;
};

class Init_Sightings_x
{
public:
  explicit Init_Sightings_x(::custom_msgs::msg::Sightings & msg)
  : msg_(msg)
  {}
  Init_Sightings_y x(::custom_msgs::msg::Sightings::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Sightings_y(msg_);
  }

private:
  ::custom_msgs::msg::Sightings msg_;
};

class Init_Sightings_animal_type
{
public:
  explicit Init_Sightings_animal_type(::custom_msgs::msg::Sightings & msg)
  : msg_(msg)
  {}
  Init_Sightings_x animal_type(::custom_msgs::msg::Sightings::_animal_type_type arg)
  {
    msg_.animal_type = std::move(arg);
    return Init_Sightings_x(msg_);
  }

private:
  ::custom_msgs::msg::Sightings msg_;
};

class Init_Sightings_sighting_count
{
public:
  Init_Sightings_sighting_count()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Sightings_animal_type sighting_count(::custom_msgs::msg::Sightings::_sighting_count_type arg)
  {
    msg_.sighting_count = std::move(arg);
    return Init_Sightings_animal_type(msg_);
  }

private:
  ::custom_msgs::msg::Sightings msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_msgs::msg::Sightings>()
{
  return custom_msgs::msg::builder::Init_Sightings_sighting_count();
}

}  // namespace custom_msgs

#endif  // CUSTOM_MSGS__MSG__DETAIL__SIGHTINGS__BUILDER_HPP_
