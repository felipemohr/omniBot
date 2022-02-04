// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from keyboard_interface:msg/Keys.idl
// generated code does not contain a copyright notice

#ifndef KEYBOARD_INTERFACE__MSG__DETAIL__KEYS__STRUCT_H_
#define KEYBOARD_INTERFACE__MSG__DETAIL__KEYS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pressed_keys'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/Keys in the package keyboard_interface.
typedef struct keyboard_interface__msg__Keys
{
  rosidl_runtime_c__String__Sequence pressed_keys;
} keyboard_interface__msg__Keys;

// Struct for a sequence of keyboard_interface__msg__Keys.
typedef struct keyboard_interface__msg__Keys__Sequence
{
  keyboard_interface__msg__Keys * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} keyboard_interface__msg__Keys__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KEYBOARD_INTERFACE__MSG__DETAIL__KEYS__STRUCT_H_
