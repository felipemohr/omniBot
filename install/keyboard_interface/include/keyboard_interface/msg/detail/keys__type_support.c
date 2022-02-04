// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from keyboard_interface:msg/Keys.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "keyboard_interface/msg/detail/keys__rosidl_typesupport_introspection_c.h"
#include "keyboard_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "keyboard_interface/msg/detail/keys__functions.h"
#include "keyboard_interface/msg/detail/keys__struct.h"


// Include directives for member types
// Member `pressed_keys`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Keys__rosidl_typesupport_introspection_c__Keys_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  keyboard_interface__msg__Keys__init(message_memory);
}

void Keys__rosidl_typesupport_introspection_c__Keys_fini_function(void * message_memory)
{
  keyboard_interface__msg__Keys__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Keys__rosidl_typesupport_introspection_c__Keys_message_member_array[1] = {
  {
    "pressed_keys",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(keyboard_interface__msg__Keys, pressed_keys),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Keys__rosidl_typesupport_introspection_c__Keys_message_members = {
  "keyboard_interface__msg",  // message namespace
  "Keys",  // message name
  1,  // number of fields
  sizeof(keyboard_interface__msg__Keys),
  Keys__rosidl_typesupport_introspection_c__Keys_message_member_array,  // message members
  Keys__rosidl_typesupport_introspection_c__Keys_init_function,  // function to initialize message memory (memory has to be allocated)
  Keys__rosidl_typesupport_introspection_c__Keys_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Keys__rosidl_typesupport_introspection_c__Keys_message_type_support_handle = {
  0,
  &Keys__rosidl_typesupport_introspection_c__Keys_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_keyboard_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, keyboard_interface, msg, Keys)() {
  if (!Keys__rosidl_typesupport_introspection_c__Keys_message_type_support_handle.typesupport_identifier) {
    Keys__rosidl_typesupport_introspection_c__Keys_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Keys__rosidl_typesupport_introspection_c__Keys_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
