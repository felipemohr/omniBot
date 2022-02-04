// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from keyboard_interface:msg/Keys.idl
// generated code does not contain a copyright notice

#ifndef KEYBOARD_INTERFACE__MSG__DETAIL__KEYS__STRUCT_HPP_
#define KEYBOARD_INTERFACE__MSG__DETAIL__KEYS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__keyboard_interface__msg__Keys __attribute__((deprecated))
#else
# define DEPRECATED__keyboard_interface__msg__Keys __declspec(deprecated)
#endif

namespace keyboard_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Keys_
{
  using Type = Keys_<ContainerAllocator>;

  explicit Keys_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit Keys_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _pressed_keys_type =
    std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other>;
  _pressed_keys_type pressed_keys;

  // setters for named parameter idiom
  Type & set__pressed_keys(
    const std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other> & _arg)
  {
    this->pressed_keys = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    keyboard_interface::msg::Keys_<ContainerAllocator> *;
  using ConstRawPtr =
    const keyboard_interface::msg::Keys_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<keyboard_interface::msg::Keys_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<keyboard_interface::msg::Keys_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      keyboard_interface::msg::Keys_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<keyboard_interface::msg::Keys_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      keyboard_interface::msg::Keys_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<keyboard_interface::msg::Keys_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<keyboard_interface::msg::Keys_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<keyboard_interface::msg::Keys_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__keyboard_interface__msg__Keys
    std::shared_ptr<keyboard_interface::msg::Keys_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__keyboard_interface__msg__Keys
    std::shared_ptr<keyboard_interface::msg::Keys_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Keys_ & other) const
  {
    if (this->pressed_keys != other.pressed_keys) {
      return false;
    }
    return true;
  }
  bool operator!=(const Keys_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Keys_

// alias to use template instance with default allocator
using Keys =
  keyboard_interface::msg::Keys_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace keyboard_interface

#endif  // KEYBOARD_INTERFACE__MSG__DETAIL__KEYS__STRUCT_HPP_
