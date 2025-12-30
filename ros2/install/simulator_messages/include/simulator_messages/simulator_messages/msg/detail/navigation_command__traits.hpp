// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from simulator_messages:msg/NavigationCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "simulator_messages/msg/navigation_command.hpp"


#ifndef SIMULATOR_MESSAGES__MSG__DETAIL__NAVIGATION_COMMAND__TRAITS_HPP_
#define SIMULATOR_MESSAGES__MSG__DETAIL__NAVIGATION_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "simulator_messages/msg/detail/navigation_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace simulator_messages
{

namespace msg
{

inline void to_flow_style_yaml(
  const NavigationCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: local_position_offset
  {
    if (msg.local_position_offset.size() == 0) {
      out << "local_position_offset: []";
    } else {
      out << "local_position_offset: [";
      size_t pending_items = msg.local_position_offset.size();
      for (auto item : msg.local_position_offset) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: local_rotation_offset
  {
    if (msg.local_rotation_offset.size() == 0) {
      out << "local_rotation_offset: []";
    } else {
      out << "local_rotation_offset: [";
      size_t pending_items = msg.local_rotation_offset.size();
      for (auto item : msg.local_rotation_offset) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: is_stop
  {
    out << "is_stop: ";
    rosidl_generator_traits::value_to_yaml(msg.is_stop, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NavigationCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: local_position_offset
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.local_position_offset.size() == 0) {
      out << "local_position_offset: []\n";
    } else {
      out << "local_position_offset:\n";
      for (auto item : msg.local_position_offset) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: local_rotation_offset
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.local_rotation_offset.size() == 0) {
      out << "local_rotation_offset: []\n";
    } else {
      out << "local_rotation_offset:\n";
      for (auto item : msg.local_rotation_offset) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: is_stop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_stop: ";
    rosidl_generator_traits::value_to_yaml(msg.is_stop, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NavigationCommand & msg, bool use_flow_style = false)
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

}  // namespace simulator_messages

namespace rosidl_generator_traits
{

[[deprecated("use simulator_messages::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const simulator_messages::msg::NavigationCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  simulator_messages::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use simulator_messages::msg::to_yaml() instead")]]
inline std::string to_yaml(const simulator_messages::msg::NavigationCommand & msg)
{
  return simulator_messages::msg::to_yaml(msg);
}

template<>
inline const char * data_type<simulator_messages::msg::NavigationCommand>()
{
  return "simulator_messages::msg::NavigationCommand";
}

template<>
inline const char * name<simulator_messages::msg::NavigationCommand>()
{
  return "simulator_messages/msg/NavigationCommand";
}

template<>
struct has_fixed_size<simulator_messages::msg::NavigationCommand>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<simulator_messages::msg::NavigationCommand>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<simulator_messages::msg::NavigationCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SIMULATOR_MESSAGES__MSG__DETAIL__NAVIGATION_COMMAND__TRAITS_HPP_
