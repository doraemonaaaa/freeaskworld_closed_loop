// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from simulator_messages:msg/NavigationCommand.idl
// generated code does not contain a copyright notice
#include "simulator_messages/msg/detail/navigation_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
simulator_messages__msg__NavigationCommand__init(simulator_messages__msg__NavigationCommand * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    simulator_messages__msg__NavigationCommand__fini(msg);
    return false;
  }
  // local_position_offset
  // local_rotation_offset
  // is_stop
  return true;
}

void
simulator_messages__msg__NavigationCommand__fini(simulator_messages__msg__NavigationCommand * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // local_position_offset
  // local_rotation_offset
  // is_stop
}

bool
simulator_messages__msg__NavigationCommand__are_equal(const simulator_messages__msg__NavigationCommand * lhs, const simulator_messages__msg__NavigationCommand * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // local_position_offset
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->local_position_offset[i] != rhs->local_position_offset[i]) {
      return false;
    }
  }
  // local_rotation_offset
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->local_rotation_offset[i] != rhs->local_rotation_offset[i]) {
      return false;
    }
  }
  // is_stop
  if (lhs->is_stop != rhs->is_stop) {
    return false;
  }
  return true;
}

bool
simulator_messages__msg__NavigationCommand__copy(
  const simulator_messages__msg__NavigationCommand * input,
  simulator_messages__msg__NavigationCommand * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // local_position_offset
  for (size_t i = 0; i < 3; ++i) {
    output->local_position_offset[i] = input->local_position_offset[i];
  }
  // local_rotation_offset
  for (size_t i = 0; i < 4; ++i) {
    output->local_rotation_offset[i] = input->local_rotation_offset[i];
  }
  // is_stop
  output->is_stop = input->is_stop;
  return true;
}

simulator_messages__msg__NavigationCommand *
simulator_messages__msg__NavigationCommand__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  simulator_messages__msg__NavigationCommand * msg = (simulator_messages__msg__NavigationCommand *)allocator.allocate(sizeof(simulator_messages__msg__NavigationCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(simulator_messages__msg__NavigationCommand));
  bool success = simulator_messages__msg__NavigationCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
simulator_messages__msg__NavigationCommand__destroy(simulator_messages__msg__NavigationCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    simulator_messages__msg__NavigationCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
simulator_messages__msg__NavigationCommand__Sequence__init(simulator_messages__msg__NavigationCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  simulator_messages__msg__NavigationCommand * data = NULL;

  if (size) {
    data = (simulator_messages__msg__NavigationCommand *)allocator.zero_allocate(size, sizeof(simulator_messages__msg__NavigationCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = simulator_messages__msg__NavigationCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        simulator_messages__msg__NavigationCommand__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
simulator_messages__msg__NavigationCommand__Sequence__fini(simulator_messages__msg__NavigationCommand__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      simulator_messages__msg__NavigationCommand__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

simulator_messages__msg__NavigationCommand__Sequence *
simulator_messages__msg__NavigationCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  simulator_messages__msg__NavigationCommand__Sequence * array = (simulator_messages__msg__NavigationCommand__Sequence *)allocator.allocate(sizeof(simulator_messages__msg__NavigationCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = simulator_messages__msg__NavigationCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
simulator_messages__msg__NavigationCommand__Sequence__destroy(simulator_messages__msg__NavigationCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    simulator_messages__msg__NavigationCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
simulator_messages__msg__NavigationCommand__Sequence__are_equal(const simulator_messages__msg__NavigationCommand__Sequence * lhs, const simulator_messages__msg__NavigationCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!simulator_messages__msg__NavigationCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
simulator_messages__msg__NavigationCommand__Sequence__copy(
  const simulator_messages__msg__NavigationCommand__Sequence * input,
  simulator_messages__msg__NavigationCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(simulator_messages__msg__NavigationCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    simulator_messages__msg__NavigationCommand * data =
      (simulator_messages__msg__NavigationCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!simulator_messages__msg__NavigationCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          simulator_messages__msg__NavigationCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!simulator_messages__msg__NavigationCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
