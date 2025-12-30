// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from simulator_messages:msg/NavigationCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "simulator_messages/msg/navigation_command.h"


#ifndef SIMULATOR_MESSAGES__MSG__DETAIL__NAVIGATION_COMMAND__FUNCTIONS_H_
#define SIMULATOR_MESSAGES__MSG__DETAIL__NAVIGATION_COMMAND__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "simulator_messages/msg/rosidl_generator_c__visibility_control.h"

#include "simulator_messages/msg/detail/navigation_command__struct.h"

/// Initialize msg/NavigationCommand message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * simulator_messages__msg__NavigationCommand
 * )) before or use
 * simulator_messages__msg__NavigationCommand__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_simulator_messages
bool
simulator_messages__msg__NavigationCommand__init(simulator_messages__msg__NavigationCommand * msg);

/// Finalize msg/NavigationCommand message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_simulator_messages
void
simulator_messages__msg__NavigationCommand__fini(simulator_messages__msg__NavigationCommand * msg);

/// Create msg/NavigationCommand message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * simulator_messages__msg__NavigationCommand__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_simulator_messages
simulator_messages__msg__NavigationCommand *
simulator_messages__msg__NavigationCommand__create(void);

/// Destroy msg/NavigationCommand message.
/**
 * It calls
 * simulator_messages__msg__NavigationCommand__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_simulator_messages
void
simulator_messages__msg__NavigationCommand__destroy(simulator_messages__msg__NavigationCommand * msg);

/// Check for msg/NavigationCommand message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_simulator_messages
bool
simulator_messages__msg__NavigationCommand__are_equal(const simulator_messages__msg__NavigationCommand * lhs, const simulator_messages__msg__NavigationCommand * rhs);

/// Copy a msg/NavigationCommand message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_simulator_messages
bool
simulator_messages__msg__NavigationCommand__copy(
  const simulator_messages__msg__NavigationCommand * input,
  simulator_messages__msg__NavigationCommand * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_simulator_messages
const rosidl_type_hash_t *
simulator_messages__msg__NavigationCommand__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_simulator_messages
const rosidl_runtime_c__type_description__TypeDescription *
simulator_messages__msg__NavigationCommand__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_simulator_messages
const rosidl_runtime_c__type_description__TypeSource *
simulator_messages__msg__NavigationCommand__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_simulator_messages
const rosidl_runtime_c__type_description__TypeSource__Sequence *
simulator_messages__msg__NavigationCommand__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/NavigationCommand messages.
/**
 * It allocates the memory for the number of elements and calls
 * simulator_messages__msg__NavigationCommand__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_simulator_messages
bool
simulator_messages__msg__NavigationCommand__Sequence__init(simulator_messages__msg__NavigationCommand__Sequence * array, size_t size);

/// Finalize array of msg/NavigationCommand messages.
/**
 * It calls
 * simulator_messages__msg__NavigationCommand__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_simulator_messages
void
simulator_messages__msg__NavigationCommand__Sequence__fini(simulator_messages__msg__NavigationCommand__Sequence * array);

/// Create array of msg/NavigationCommand messages.
/**
 * It allocates the memory for the array and calls
 * simulator_messages__msg__NavigationCommand__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_simulator_messages
simulator_messages__msg__NavigationCommand__Sequence *
simulator_messages__msg__NavigationCommand__Sequence__create(size_t size);

/// Destroy array of msg/NavigationCommand messages.
/**
 * It calls
 * simulator_messages__msg__NavigationCommand__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_simulator_messages
void
simulator_messages__msg__NavigationCommand__Sequence__destroy(simulator_messages__msg__NavigationCommand__Sequence * array);

/// Check for msg/NavigationCommand message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_simulator_messages
bool
simulator_messages__msg__NavigationCommand__Sequence__are_equal(const simulator_messages__msg__NavigationCommand__Sequence * lhs, const simulator_messages__msg__NavigationCommand__Sequence * rhs);

/// Copy an array of msg/NavigationCommand messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_simulator_messages
bool
simulator_messages__msg__NavigationCommand__Sequence__copy(
  const simulator_messages__msg__NavigationCommand__Sequence * input,
  simulator_messages__msg__NavigationCommand__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // SIMULATOR_MESSAGES__MSG__DETAIL__NAVIGATION_COMMAND__FUNCTIONS_H_
