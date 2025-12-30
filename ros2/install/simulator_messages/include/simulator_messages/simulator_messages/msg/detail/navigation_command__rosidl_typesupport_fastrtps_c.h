// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from simulator_messages:msg/NavigationCommand.idl
// generated code does not contain a copyright notice
#ifndef SIMULATOR_MESSAGES__MSG__DETAIL__NAVIGATION_COMMAND__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define SIMULATOR_MESSAGES__MSG__DETAIL__NAVIGATION_COMMAND__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "simulator_messages/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "simulator_messages/msg/detail/navigation_command__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_simulator_messages
bool cdr_serialize_simulator_messages__msg__NavigationCommand(
  const simulator_messages__msg__NavigationCommand * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_simulator_messages
bool cdr_deserialize_simulator_messages__msg__NavigationCommand(
  eprosima::fastcdr::Cdr &,
  simulator_messages__msg__NavigationCommand * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_simulator_messages
size_t get_serialized_size_simulator_messages__msg__NavigationCommand(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_simulator_messages
size_t max_serialized_size_simulator_messages__msg__NavigationCommand(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_simulator_messages
bool cdr_serialize_key_simulator_messages__msg__NavigationCommand(
  const simulator_messages__msg__NavigationCommand * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_simulator_messages
size_t get_serialized_size_key_simulator_messages__msg__NavigationCommand(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_simulator_messages
size_t max_serialized_size_key_simulator_messages__msg__NavigationCommand(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_simulator_messages
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, simulator_messages, msg, NavigationCommand)();

#ifdef __cplusplus
}
#endif

#endif  // SIMULATOR_MESSAGES__MSG__DETAIL__NAVIGATION_COMMAND__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
