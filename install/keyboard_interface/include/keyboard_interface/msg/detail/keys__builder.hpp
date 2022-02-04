// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from keyboard_interface:msg/Keys.idl
// generated code does not contain a copyright notice

#ifndef KEYBOARD_INTERFACE__MSG__DETAIL__KEYS__BUILDER_HPP_
#define KEYBOARD_INTERFACE__MSG__DETAIL__KEYS__BUILDER_HPP_

#include "keyboard_interface/msg/detail/keys__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace keyboard_interface
{

namespace msg
{

namespace builder
{

class Init_Keys_pressed_keys
{
public:
  Init_Keys_pressed_keys()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::keyboard_interface::msg::Keys pressed_keys(::keyboard_interface::msg::Keys::_pressed_keys_type arg)
  {
    msg_.pressed_keys = std::move(arg);
    return std::move(msg_);
  }

private:
  ::keyboard_interface::msg::Keys msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::keyboard_interface::msg::Keys>()
{
  return keyboard_interface::msg::builder::Init_Keys_pressed_keys();
}

}  // namespace keyboard_interface

#endif  // KEYBOARD_INTERFACE__MSG__DETAIL__KEYS__BUILDER_HPP_
