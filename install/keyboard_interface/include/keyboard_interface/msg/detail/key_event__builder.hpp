// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from keyboard_interface:msg/KeyEvent.idl
// generated code does not contain a copyright notice

#ifndef KEYBOARD_INTERFACE__MSG__DETAIL__KEY_EVENT__BUILDER_HPP_
#define KEYBOARD_INTERFACE__MSG__DETAIL__KEY_EVENT__BUILDER_HPP_

#include "keyboard_interface/msg/detail/key_event__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace keyboard_interface
{

namespace msg
{

namespace builder
{

class Init_KeyEvent_key
{
public:
  explicit Init_KeyEvent_key(::keyboard_interface::msg::KeyEvent & msg)
  : msg_(msg)
  {}
  ::keyboard_interface::msg::KeyEvent key(::keyboard_interface::msg::KeyEvent::_key_type arg)
  {
    msg_.key = std::move(arg);
    return std::move(msg_);
  }

private:
  ::keyboard_interface::msg::KeyEvent msg_;
};

class Init_KeyEvent_event
{
public:
  Init_KeyEvent_event()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_KeyEvent_key event(::keyboard_interface::msg::KeyEvent::_event_type arg)
  {
    msg_.event = std::move(arg);
    return Init_KeyEvent_key(msg_);
  }

private:
  ::keyboard_interface::msg::KeyEvent msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::keyboard_interface::msg::KeyEvent>()
{
  return keyboard_interface::msg::builder::Init_KeyEvent_event();
}

}  // namespace keyboard_interface

#endif  // KEYBOARD_INTERFACE__MSG__DETAIL__KEY_EVENT__BUILDER_HPP_
