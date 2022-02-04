// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from keyboard_interface:msg/Keys.idl
// generated code does not contain a copyright notice
#include "keyboard_interface/msg/detail/keys__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `pressed_keys`
#include "rosidl_runtime_c/string_functions.h"

bool
keyboard_interface__msg__Keys__init(keyboard_interface__msg__Keys * msg)
{
  if (!msg) {
    return false;
  }
  // pressed_keys
  if (!rosidl_runtime_c__String__Sequence__init(&msg->pressed_keys, 0)) {
    keyboard_interface__msg__Keys__fini(msg);
    return false;
  }
  return true;
}

void
keyboard_interface__msg__Keys__fini(keyboard_interface__msg__Keys * msg)
{
  if (!msg) {
    return;
  }
  // pressed_keys
  rosidl_runtime_c__String__Sequence__fini(&msg->pressed_keys);
}

keyboard_interface__msg__Keys *
keyboard_interface__msg__Keys__create()
{
  keyboard_interface__msg__Keys * msg = (keyboard_interface__msg__Keys *)malloc(sizeof(keyboard_interface__msg__Keys));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(keyboard_interface__msg__Keys));
  bool success = keyboard_interface__msg__Keys__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
keyboard_interface__msg__Keys__destroy(keyboard_interface__msg__Keys * msg)
{
  if (msg) {
    keyboard_interface__msg__Keys__fini(msg);
  }
  free(msg);
}


bool
keyboard_interface__msg__Keys__Sequence__init(keyboard_interface__msg__Keys__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  keyboard_interface__msg__Keys * data = NULL;
  if (size) {
    data = (keyboard_interface__msg__Keys *)calloc(size, sizeof(keyboard_interface__msg__Keys));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = keyboard_interface__msg__Keys__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        keyboard_interface__msg__Keys__fini(&data[i - 1]);
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
keyboard_interface__msg__Keys__Sequence__fini(keyboard_interface__msg__Keys__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      keyboard_interface__msg__Keys__fini(&array->data[i]);
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

keyboard_interface__msg__Keys__Sequence *
keyboard_interface__msg__Keys__Sequence__create(size_t size)
{
  keyboard_interface__msg__Keys__Sequence * array = (keyboard_interface__msg__Keys__Sequence *)malloc(sizeof(keyboard_interface__msg__Keys__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = keyboard_interface__msg__Keys__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
keyboard_interface__msg__Keys__Sequence__destroy(keyboard_interface__msg__Keys__Sequence * array)
{
  if (array) {
    keyboard_interface__msg__Keys__Sequence__fini(array);
  }
  free(array);
}
