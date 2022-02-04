// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from keyboard_interface:msg/KeyEvent.idl
// generated code does not contain a copyright notice

#ifndef KEYBOARD_INTERFACE__MSG__DETAIL__KEY_EVENT__FUNCTIONS_H_
#define KEYBOARD_INTERFACE__MSG__DETAIL__KEY_EVENT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "keyboard_interface/msg/rosidl_generator_c__visibility_control.h"

#include "keyboard_interface/msg/detail/key_event__struct.h"

/// Initialize msg/KeyEvent message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * keyboard_interface__msg__KeyEvent
 * )) before or use
 * keyboard_interface__msg__KeyEvent__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_keyboard_interface
bool
keyboard_interface__msg__KeyEvent__init(keyboard_interface__msg__KeyEvent * msg);

/// Finalize msg/KeyEvent message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_keyboard_interface
void
keyboard_interface__msg__KeyEvent__fini(keyboard_interface__msg__KeyEvent * msg);

/// Create msg/KeyEvent message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * keyboard_interface__msg__KeyEvent__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_keyboard_interface
keyboard_interface__msg__KeyEvent *
keyboard_interface__msg__KeyEvent__create();

/// Destroy msg/KeyEvent message.
/**
 * It calls
 * keyboard_interface__msg__KeyEvent__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_keyboard_interface
void
keyboard_interface__msg__KeyEvent__destroy(keyboard_interface__msg__KeyEvent * msg);


/// Initialize array of msg/KeyEvent messages.
/**
 * It allocates the memory for the number of elements and calls
 * keyboard_interface__msg__KeyEvent__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_keyboard_interface
bool
keyboard_interface__msg__KeyEvent__Sequence__init(keyboard_interface__msg__KeyEvent__Sequence * array, size_t size);

/// Finalize array of msg/KeyEvent messages.
/**
 * It calls
 * keyboard_interface__msg__KeyEvent__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_keyboard_interface
void
keyboard_interface__msg__KeyEvent__Sequence__fini(keyboard_interface__msg__KeyEvent__Sequence * array);

/// Create array of msg/KeyEvent messages.
/**
 * It allocates the memory for the array and calls
 * keyboard_interface__msg__KeyEvent__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_keyboard_interface
keyboard_interface__msg__KeyEvent__Sequence *
keyboard_interface__msg__KeyEvent__Sequence__create(size_t size);

/// Destroy array of msg/KeyEvent messages.
/**
 * It calls
 * keyboard_interface__msg__KeyEvent__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_keyboard_interface
void
keyboard_interface__msg__KeyEvent__Sequence__destroy(keyboard_interface__msg__KeyEvent__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // KEYBOARD_INTERFACE__MSG__DETAIL__KEY_EVENT__FUNCTIONS_H_
