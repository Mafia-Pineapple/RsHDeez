// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_msgs:msg/Sightings.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MSGS__MSG__DETAIL__SIGHTINGS__TRAITS_HPP_
#define CUSTOM_MSGS__MSG__DETAIL__SIGHTINGS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_msgs/msg/detail/sightings__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace custom_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Sightings & msg,
  std::ostream & out)
{
  out << "{";
  // member: sighting_count
  {
    out << "sighting_count: ";
    rosidl_generator_traits::value_to_yaml(msg.sighting_count, out);
    out << ", ";
  }

  // member: animal_type
  {
    if (msg.animal_type.size() == 0) {
      out << "animal_type: []";
    } else {
      out << "animal_type: [";
      size_t pending_items = msg.animal_type.size();
      for (auto item : msg.animal_type) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: x
  {
    if (msg.x.size() == 0) {
      out << "x: []";
    } else {
      out << "x: [";
      size_t pending_items = msg.x.size();
      for (auto item : msg.x) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: y
  {
    if (msg.y.size() == 0) {
      out << "y: []";
    } else {
      out << "y: [";
      size_t pending_items = msg.y.size();
      for (auto item : msg.y) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: z
  {
    if (msg.z.size() == 0) {
      out << "z: []";
    } else {
      out << "z: [";
      size_t pending_items = msg.z.size();
      for (auto item : msg.z) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: save
  {
    out << "save: ";
    rosidl_generator_traits::value_to_yaml(msg.save, out);
    out << ", ";
  }

  // member: save_dir
  {
    out << "save_dir: ";
    rosidl_generator_traits::value_to_yaml(msg.save_dir, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Sightings & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: sighting_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sighting_count: ";
    rosidl_generator_traits::value_to_yaml(msg.sighting_count, out);
    out << "\n";
  }

  // member: animal_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.animal_type.size() == 0) {
      out << "animal_type: []\n";
    } else {
      out << "animal_type:\n";
      for (auto item : msg.animal_type) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.x.size() == 0) {
      out << "x: []\n";
    } else {
      out << "x:\n";
      for (auto item : msg.x) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.y.size() == 0) {
      out << "y: []\n";
    } else {
      out << "y:\n";
      for (auto item : msg.y) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.z.size() == 0) {
      out << "z: []\n";
    } else {
      out << "z:\n";
      for (auto item : msg.z) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: save
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "save: ";
    rosidl_generator_traits::value_to_yaml(msg.save, out);
    out << "\n";
  }

  // member: save_dir
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "save_dir: ";
    rosidl_generator_traits::value_to_yaml(msg.save_dir, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Sightings & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace custom_msgs

namespace rosidl_generator_traits
{

[[deprecated("use custom_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_msgs::msg::Sightings & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const custom_msgs::msg::Sightings & msg)
{
  return custom_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<custom_msgs::msg::Sightings>()
{
  return "custom_msgs::msg::Sightings";
}

template<>
inline const char * name<custom_msgs::msg::Sightings>()
{
  return "custom_msgs/msg/Sightings";
}

template<>
struct has_fixed_size<custom_msgs::msg::Sightings>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<custom_msgs::msg::Sightings>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<custom_msgs::msg::Sightings>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_MSGS__MSG__DETAIL__SIGHTINGS__TRAITS_HPP_
