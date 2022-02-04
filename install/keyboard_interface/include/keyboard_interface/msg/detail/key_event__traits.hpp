// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from keyboard_interface:msg/KeyEvent.idl
// generated code does not contain a copyright notice

#ifndef KEYBOARD_INTERFACE__MSG__DETAIL__KEY_EVENT__TRAITS_HPP_
#define KEYBOARD_INTERFACE__MSG__DETAIL__KEY_EVENT__TRAITS_HPP_

#include "keyboard_interface/msg/detail/key_event__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<keyboard_interface::msg::KeyEvent>()
{
  return "keyboard_interface::msg::KeyEvent";
}

template<>
inline const char * name<keyboard_interface::msg::KeyEvent>()
{
  return "keyboard_interface/msg/KeyEvent";
}

template<>
struct has_fixed_size<keyboard_interface::msg::KeyEvent>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<keyboard_interface::msg::KeyEvent>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<keyboard_interface::msg::KeyEvent>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // KEYBOARD_INTERFACE__MSG__DETAIL__KEY_EVENT__TRAITS_HPP_
