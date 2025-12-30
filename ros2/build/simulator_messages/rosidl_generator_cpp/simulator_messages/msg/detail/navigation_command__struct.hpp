// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from simulator_messages:msg/NavigationCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "simulator_messages/msg/navigation_command.hpp"


#ifndef SIMULATOR_MESSAGES__MSG__DETAIL__NAVIGATION_COMMAND__STRUCT_HPP_
#define SIMULATOR_MESSAGES__MSG__DETAIL__NAVIGATION_COMMAND__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__simulator_messages__msg__NavigationCommand __attribute__((deprecated))
#else
# define DEPRECATED__simulator_messages__msg__NavigationCommand __declspec(deprecated)
#endif

namespace simulator_messages
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct NavigationCommand_
{
  using Type = NavigationCommand_<ContainerAllocator>;

  explicit NavigationCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 3>::iterator, double>(this->local_position_offset.begin(), this->local_position_offset.end(), 0.0);
      std::fill<typename std::array<double, 4>::iterator, double>(this->local_rotation_offset.begin(), this->local_rotation_offset.end(), 0.0);
      this->is_stop = false;
    }
  }

  explicit NavigationCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    local_position_offset(_alloc),
    local_rotation_offset(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 3>::iterator, double>(this->local_position_offset.begin(), this->local_position_offset.end(), 0.0);
      std::fill<typename std::array<double, 4>::iterator, double>(this->local_rotation_offset.begin(), this->local_rotation_offset.end(), 0.0);
      this->is_stop = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _local_position_offset_type =
    std::array<double, 3>;
  _local_position_offset_type local_position_offset;
  using _local_rotation_offset_type =
    std::array<double, 4>;
  _local_rotation_offset_type local_rotation_offset;
  using _is_stop_type =
    bool;
  _is_stop_type is_stop;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__local_position_offset(
    const std::array<double, 3> & _arg)
  {
    this->local_position_offset = _arg;
    return *this;
  }
  Type & set__local_rotation_offset(
    const std::array<double, 4> & _arg)
  {
    this->local_rotation_offset = _arg;
    return *this;
  }
  Type & set__is_stop(
    const bool & _arg)
  {
    this->is_stop = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    simulator_messages::msg::NavigationCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const simulator_messages::msg::NavigationCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<simulator_messages::msg::NavigationCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<simulator_messages::msg::NavigationCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      simulator_messages::msg::NavigationCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<simulator_messages::msg::NavigationCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      simulator_messages::msg::NavigationCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<simulator_messages::msg::NavigationCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<simulator_messages::msg::NavigationCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<simulator_messages::msg::NavigationCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__simulator_messages__msg__NavigationCommand
    std::shared_ptr<simulator_messages::msg::NavigationCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__simulator_messages__msg__NavigationCommand
    std::shared_ptr<simulator_messages::msg::NavigationCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const NavigationCommand_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->local_position_offset != other.local_position_offset) {
      return false;
    }
    if (this->local_rotation_offset != other.local_rotation_offset) {
      return false;
    }
    if (this->is_stop != other.is_stop) {
      return false;
    }
    return true;
  }
  bool operator!=(const NavigationCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct NavigationCommand_

// alias to use template instance with default allocator
using NavigationCommand =
  simulator_messages::msg::NavigationCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace simulator_messages

#endif  // SIMULATOR_MESSAGES__MSG__DETAIL__NAVIGATION_COMMAND__STRUCT_HPP_
