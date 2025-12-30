// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from simulator_messages:msg/NavigationCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "simulator_messages/msg/navigation_command.h"


#ifndef SIMULATOR_MESSAGES__MSG__DETAIL__NAVIGATION_COMMAND__STRUCT_H_
#define SIMULATOR_MESSAGES__MSG__DETAIL__NAVIGATION_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/NavigationCommand in the package simulator_messages.
/**
  * Header
 */
typedef struct simulator_messages__msg__NavigationCommand
{
  std_msgs__msg__Header header;
  /// 位移偏移（局部坐标）
  double local_position_offset[3];
  /// 四元数旋转
  double local_rotation_offset[4];
  /// 是否停止
  bool is_stop;
} simulator_messages__msg__NavigationCommand;

// Struct for a sequence of simulator_messages__msg__NavigationCommand.
typedef struct simulator_messages__msg__NavigationCommand__Sequence
{
  simulator_messages__msg__NavigationCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} simulator_messages__msg__NavigationCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SIMULATOR_MESSAGES__MSG__DETAIL__NAVIGATION_COMMAND__STRUCT_H_
