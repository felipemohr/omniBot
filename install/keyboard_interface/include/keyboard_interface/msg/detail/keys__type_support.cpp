// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from keyboard_interface:msg/Keys.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "keyboard_interface/msg/detail/keys__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace keyboard_interface
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void Keys_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) keyboard_interface::msg::Keys(_init);
}

void Keys_fini_function(void * message_memory)
{
  auto typed_message = static_cast<keyboard_interface::msg::Keys *>(message_memory);
  typed_message->~Keys();
}

size_t size_function__Keys__pressed_keys(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Keys__pressed_keys(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__Keys__pressed_keys(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void resize_function__Keys__pressed_keys(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Keys_message_member_array[1] = {
  {
    "pressed_keys",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(keyboard_interface::msg::Keys, pressed_keys),  // bytes offset in struct
    nullptr,  // default value
    size_function__Keys__pressed_keys,  // size() function pointer
    get_const_function__Keys__pressed_keys,  // get_const(index) function pointer
    get_function__Keys__pressed_keys,  // get(index) function pointer
    resize_function__Keys__pressed_keys  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Keys_message_members = {
  "keyboard_interface::msg",  // message namespace
  "Keys",  // message name
  1,  // number of fields
  sizeof(keyboard_interface::msg::Keys),
  Keys_message_member_array,  // message members
  Keys_init_function,  // function to initialize message memory (memory has to be allocated)
  Keys_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Keys_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Keys_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace keyboard_interface


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<keyboard_interface::msg::Keys>()
{
  return &::keyboard_interface::msg::rosidl_typesupport_introspection_cpp::Keys_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, keyboard_interface, msg, Keys)() {
  return &::keyboard_interface::msg::rosidl_typesupport_introspection_cpp::Keys_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
