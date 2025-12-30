// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from simulator_messages:msg/NavigationCommand.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "simulator_messages/msg/detail/navigation_command__rosidl_typesupport_introspection_c.h"
#include "simulator_messages/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "simulator_messages/msg/detail/navigation_command__functions.h"
#include "simulator_messages/msg/detail/navigation_command__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__NavigationCommand_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  simulator_messages__msg__NavigationCommand__init(message_memory);
}

void simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__NavigationCommand_fini_function(void * message_memory)
{
  simulator_messages__msg__NavigationCommand__fini(message_memory);
}

size_t simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__size_function__NavigationCommand__local_position_offset(
  const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__get_const_function__NavigationCommand__local_position_offset(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__get_function__NavigationCommand__local_position_offset(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__fetch_function__NavigationCommand__local_position_offset(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__get_const_function__NavigationCommand__local_position_offset(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__assign_function__NavigationCommand__local_position_offset(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__get_function__NavigationCommand__local_position_offset(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__size_function__NavigationCommand__local_rotation_offset(
  const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__get_const_function__NavigationCommand__local_rotation_offset(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__get_function__NavigationCommand__local_rotation_offset(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__fetch_function__NavigationCommand__local_rotation_offset(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__get_const_function__NavigationCommand__local_rotation_offset(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__assign_function__NavigationCommand__local_rotation_offset(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__get_function__NavigationCommand__local_rotation_offset(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__NavigationCommand_message_member_array[4] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(simulator_messages__msg__NavigationCommand, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "local_position_offset",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(simulator_messages__msg__NavigationCommand, local_position_offset),  // bytes offset in struct
    NULL,  // default value
    simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__size_function__NavigationCommand__local_position_offset,  // size() function pointer
    simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__get_const_function__NavigationCommand__local_position_offset,  // get_const(index) function pointer
    simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__get_function__NavigationCommand__local_position_offset,  // get(index) function pointer
    simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__fetch_function__NavigationCommand__local_position_offset,  // fetch(index, &value) function pointer
    simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__assign_function__NavigationCommand__local_position_offset,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "local_rotation_offset",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(simulator_messages__msg__NavigationCommand, local_rotation_offset),  // bytes offset in struct
    NULL,  // default value
    simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__size_function__NavigationCommand__local_rotation_offset,  // size() function pointer
    simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__get_const_function__NavigationCommand__local_rotation_offset,  // get_const(index) function pointer
    simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__get_function__NavigationCommand__local_rotation_offset,  // get(index) function pointer
    simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__fetch_function__NavigationCommand__local_rotation_offset,  // fetch(index, &value) function pointer
    simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__assign_function__NavigationCommand__local_rotation_offset,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "is_stop",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(simulator_messages__msg__NavigationCommand, is_stop),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__NavigationCommand_message_members = {
  "simulator_messages__msg",  // message namespace
  "NavigationCommand",  // message name
  4,  // number of fields
  sizeof(simulator_messages__msg__NavigationCommand),
  false,  // has_any_key_member_
  simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__NavigationCommand_message_member_array,  // message members
  simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__NavigationCommand_init_function,  // function to initialize message memory (memory has to be allocated)
  simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__NavigationCommand_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__NavigationCommand_message_type_support_handle = {
  0,
  &simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__NavigationCommand_message_members,
  get_message_typesupport_handle_function,
  &simulator_messages__msg__NavigationCommand__get_type_hash,
  &simulator_messages__msg__NavigationCommand__get_type_description,
  &simulator_messages__msg__NavigationCommand__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_simulator_messages
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, simulator_messages, msg, NavigationCommand)() {
  simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__NavigationCommand_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__NavigationCommand_message_type_support_handle.typesupport_identifier) {
    simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__NavigationCommand_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &simulator_messages__msg__NavigationCommand__rosidl_typesupport_introspection_c__NavigationCommand_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
