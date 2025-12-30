// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from simulator_messages:msg/NavigationCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "simulator_messages/msg/navigation_command.hpp"


#ifndef SIMULATOR_MESSAGES__MSG__DETAIL__NAVIGATION_COMMAND__BUILDER_HPP_
#define SIMULATOR_MESSAGES__MSG__DETAIL__NAVIGATION_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "simulator_messages/msg/detail/navigation_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace simulator_messages
{

namespace msg
{

namespace builder
{

class Init_NavigationCommand_is_stop
{
public:
  explicit Init_NavigationCommand_is_stop(::simulator_messages::msg::NavigationCommand & msg)
  : msg_(msg)
  {}
  ::simulator_messages::msg::NavigationCommand is_stop(::simulator_messages::msg::NavigationCommand::_is_stop_type arg)
  {
    msg_.is_stop = std::move(arg);
    return std::move(msg_);
  }

private:
  ::simulator_messages::msg::NavigationCommand msg_;
};

class Init_NavigationCommand_local_rotation_offset
{
public:
  explicit Init_NavigationCommand_local_rotation_offset(::simulator_messages::msg::NavigationCommand & msg)
  : msg_(msg)
  {}
  Init_NavigationCommand_is_stop local_rotation_offset(::simulator_messages::msg::NavigationCommand::_local_rotation_offset_type arg)
  {
    msg_.local_rotation_offset = std::move(arg);
    return Init_NavigationCommand_is_stop(msg_);
  }

private:
  ::simulator_messages::msg::NavigationCommand msg_;
};

class Init_NavigationCommand_local_position_offset
{
public:
  explicit Init_NavigationCommand_local_position_offset(::simulator_messages::msg::NavigationCommand & msg)
  : msg_(msg)
  {}
  Init_NavigationCommand_local_rotation_offset local_position_offset(::simulator_messages::msg::NavigationCommand::_local_position_offset_type arg)
  {
    msg_.local_position_offset = std::move(arg);
    return Init_NavigationCommand_local_rotation_offset(msg_);
  }

private:
  ::simulator_messages::msg::NavigationCommand msg_;
};

class Init_NavigationCommand_header
{
public:
  Init_NavigationCommand_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_NavigationCommand_local_position_offset header(::simulator_messages::msg::NavigationCommand::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_NavigationCommand_local_position_offset(msg_);
  }

private:
  ::simulator_messages::msg::NavigationCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::simulator_messages::msg::NavigationCommand>()
{
  return simulator_messages::msg::builder::Init_NavigationCommand_header();
}

}  // namespace simulator_messages

#endif  // SIMULATOR_MESSAGES__MSG__DETAIL__NAVIGATION_COMMAND__BUILDER_HPP_
