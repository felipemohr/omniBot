// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from keyboard_interface:msg/KeyEvent.idl
// generated code does not contain a copyright notice

#ifndef KEYBOARD_INTERFACE__MSG__DETAIL__KEY_EVENT__STRUCT_H_
#define KEYBOARD_INTERFACE__MSG__DETAIL__KEY_EVENT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'RELEASED'.
static const bool keyboard_interface__msg__KeyEvent__RELEASED = false;

/// Constant 'PRESSED'.
static const bool keyboard_interface__msg__KeyEvent__PRESSED = true;

// Include directives for member types
// Member 'key'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/KeyEvent in the package keyboard_interface.
typedef struct keyboard_interface__msg__KeyEvent
{
  bool event;
  rosidl_runtime_c__String key;
} keyboard_interface__msg__KeyEvent;

// Struct for a sequence of keyboard_interface__msg__KeyEvent.
typedef struct keyboard_interface__msg__KeyEvent__Sequence
{
  keyboard_interface__msg__KeyEvent * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} keyboard_interface__msg__KeyEvent__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KEYBOARD_INTERFACE__MSG__DETAIL__KEY_EVENT__STRUCT_H_
