// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from simulator_messages:msg/NavigationCommand.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "simulator_messages/msg/detail/navigation_command__functions.h"
#include "simulator_messages/msg/detail/navigation_command__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace simulator_messages
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void NavigationCommand_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) simulator_messages::msg::NavigationCommand(_init);
}

void NavigationCommand_fini_function(void * message_memory)
{
  auto typed_message = static_cast<simulator_messages::msg::NavigationCommand *>(message_memory);
  typed_message->~NavigationCommand();
}

size_t size_function__NavigationCommand__local_position_offset(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__NavigationCommand__local_position_offset(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__NavigationCommand__local_position_offset(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void fetch_function__NavigationCommand__local_position_offset(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__NavigationCommand__local_position_offset(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__NavigationCommand__local_position_offset(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__NavigationCommand__local_position_offset(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__NavigationCommand__local_rotation_offset(const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * get_const_function__NavigationCommand__local_rotation_offset(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 4> *>(untyped_member);
  return &member[index];
}

void * get_function__NavigationCommand__local_rotation_offset(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 4> *>(untyped_member);
  return &member[index];
}

void fetch_function__NavigationCommand__local_rotation_offset(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__NavigationCommand__local_rotation_offset(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__NavigationCommand__local_rotation_offset(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__NavigationCommand__local_rotation_offset(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember NavigationCommand_message_member_array[4] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(simulator_messages::msg::NavigationCommand, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "local_position_offset",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(simulator_messages::msg::NavigationCommand, local_position_offset),  // bytes offset in struct
    nullptr,  // default value
    size_function__NavigationCommand__local_position_offset,  // size() function pointer
    get_const_function__NavigationCommand__local_position_offset,  // get_const(index) function pointer
    get_function__NavigationCommand__local_position_offset,  // get(index) function pointer
    fetch_function__NavigationCommand__local_position_offset,  // fetch(index, &value) function pointer
    assign_function__NavigationCommand__local_position_offset,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "local_rotation_offset",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(simulator_messages::msg::NavigationCommand, local_rotation_offset),  // bytes offset in struct
    nullptr,  // default value
    size_function__NavigationCommand__local_rotation_offset,  // size() function pointer
    get_const_function__NavigationCommand__local_rotation_offset,  // get_const(index) function pointer
    get_function__NavigationCommand__local_rotation_offset,  // get(index) function pointer
    fetch_function__NavigationCommand__local_rotation_offset,  // fetch(index, &value) function pointer
    assign_function__NavigationCommand__local_rotation_offset,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "is_stop",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(simulator_messages::msg::NavigationCommand, is_stop),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers NavigationCommand_message_members = {
  "simulator_messages::msg",  // message namespace
  "NavigationCommand",  // message name
  4,  // number of fields
  sizeof(simulator_messages::msg::NavigationCommand),
  false,  // has_any_key_member_
  NavigationCommand_message_member_array,  // message members
  NavigationCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  NavigationCommand_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t NavigationCommand_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &NavigationCommand_message_members,
  get_message_typesupport_handle_function,
  &simulator_messages__msg__NavigationCommand__get_type_hash,
  &simulator_messages__msg__NavigationCommand__get_type_description,
  &simulator_messages__msg__NavigationCommand__get_type_description_sources,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace simulator_messages


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<simulator_messages::msg::NavigationCommand>()
{
  return &::simulator_messages::msg::rosidl_typesupport_introspection_cpp::NavigationCommand_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, simulator_messages, msg, NavigationCommand)() {
  return &::simulator_messages::msg::rosidl_typesupport_introspection_cpp::NavigationCommand_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
