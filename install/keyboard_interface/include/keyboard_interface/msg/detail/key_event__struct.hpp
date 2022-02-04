// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from keyboard_interface:msg/KeyEvent.idl
// generated code does not contain a copyright notice

#ifndef KEYBOARD_INTERFACE__MSG__DETAIL__KEY_EVENT__STRUCT_HPP_
#define KEYBOARD_INTERFACE__MSG__DETAIL__KEY_EVENT__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__keyboard_interface__msg__KeyEvent __attribute__((deprecated))
#else
# define DEPRECATED__keyboard_interface__msg__KeyEvent __declspec(deprecated)
#endif

namespace keyboard_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct KeyEvent_
{
  using Type = KeyEvent_<ContainerAllocator>;

  explicit KeyEvent_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->event = false;
      this->key = "";
    }
  }

  explicit KeyEvent_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : key(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->event = false;
      this->key = "";
    }
  }

  // field types and members
  using _event_type =
    bool;
  _event_type event;
  using _key_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _key_type key;

  // setters for named parameter idiom
  Type & set__event(
    const bool & _arg)
  {
    this->event = _arg;
    return *this;
  }
  Type & set__key(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->key = _arg;
    return *this;
  }

  // constant declarations
  static constexpr bool RELEASED =
    0;
  static constexpr bool PRESSED =
    1;

  // pointer types
  using RawPtr =
    keyboard_interface::msg::KeyEvent_<ContainerAllocator> *;
  using ConstRawPtr =
    const keyboard_interface::msg::KeyEvent_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<keyboard_interface::msg::KeyEvent_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<keyboard_interface::msg::KeyEvent_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      keyboard_interface::msg::KeyEvent_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<keyboard_interface::msg::KeyEvent_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      keyboard_interface::msg::KeyEvent_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<keyboard_interface::msg::KeyEvent_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<keyboard_interface::msg::KeyEvent_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<keyboard_interface::msg::KeyEvent_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__keyboard_interface__msg__KeyEvent
    std::shared_ptr<keyboard_interface::msg::KeyEvent_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__keyboard_interface__msg__KeyEvent
    std::shared_ptr<keyboard_interface::msg::KeyEvent_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const KeyEvent_ & other) const
  {
    if (this->event != other.event) {
      return false;
    }
    if (this->key != other.key) {
      return false;
    }
    return true;
  }
  bool operator!=(const KeyEvent_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct KeyEvent_

// alias to use template instance with default allocator
using KeyEvent =
  keyboard_interface::msg::KeyEvent_<std::allocator<void>>;

// constant definitions
template<typename ContainerAllocator>
constexpr bool KeyEvent_<ContainerAllocator>::RELEASED;
template<typename ContainerAllocator>
constexpr bool KeyEvent_<ContainerAllocator>::PRESSED;

}  // namespace msg

}  // namespace keyboard_interface

#endif  // KEYBOARD_INTERFACE__MSG__DETAIL__KEY_EVENT__STRUCT_HPP_
