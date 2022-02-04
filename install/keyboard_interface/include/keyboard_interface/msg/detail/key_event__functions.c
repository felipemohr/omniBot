// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from keyboard_interface:msg/KeyEvent.idl
// generated code does not contain a copyright notice
#include "keyboard_interface/msg/detail/key_event__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `key`
#include "rosidl_runtime_c/string_functions.h"

bool
keyboard_interface__msg__KeyEvent__init(keyboard_interface__msg__KeyEvent * msg)
{
  if (!msg) {
    return false;
  }
  // event
  // key
  if (!rosidl_runtime_c__String__init(&msg->key)) {
    keyboard_interface__msg__KeyEvent__fini(msg);
    return false;
  }
  return true;
}

void
keyboard_interface__msg__KeyEvent__fini(keyboard_interface__msg__KeyEvent * msg)
{
  if (!msg) {
    return;
  }
  // event
  // key
  rosidl_runtime_c__String__fini(&msg->key);
}

keyboard_interface__msg__KeyEvent *
keyboard_interface__msg__KeyEvent__create()
{
  keyboard_interface__msg__KeyEvent * msg = (keyboard_interface__msg__KeyEvent *)malloc(sizeof(keyboard_interface__msg__KeyEvent));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(keyboard_interface__msg__KeyEvent));
  bool success = keyboard_interface__msg__KeyEvent__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
keyboard_interface__msg__KeyEvent__destroy(keyboard_interface__msg__KeyEvent * msg)
{
  if (msg) {
    keyboard_interface__msg__KeyEvent__fini(msg);
  }
  free(msg);
}


bool
keyboard_interface__msg__KeyEvent__Sequence__init(keyboard_interface__msg__KeyEvent__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  keyboard_interface__msg__KeyEvent * data = NULL;
  if (size) {
    data = (keyboard_interface__msg__KeyEvent *)calloc(size, sizeof(keyboard_interface__msg__KeyEvent));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = keyboard_interface__msg__KeyEvent__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        keyboard_interface__msg__KeyEvent__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
keyboard_interface__msg__KeyEvent__Sequence__fini(keyboard_interface__msg__KeyEvent__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      keyboard_interface__msg__KeyEvent__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

keyboard_interface__msg__KeyEvent__Sequence *
keyboard_interface__msg__KeyEvent__Sequence__create(size_t size)
{
  keyboard_interface__msg__KeyEvent__Sequence * array = (keyboard_interface__msg__KeyEvent__Sequence *)malloc(sizeof(keyboard_interface__msg__KeyEvent__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = keyboard_interface__msg__KeyEvent__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
keyboard_interface__msg__KeyEvent__Sequence__destroy(keyboard_interface__msg__KeyEvent__Sequence * array)
{
  if (array) {
    keyboard_interface__msg__KeyEvent__Sequence__fini(array);
  }
  free(array);
}
