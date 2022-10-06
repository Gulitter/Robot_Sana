// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from sana_msgs:msg/Sana.idl
// generated code does not contain a copyright notice

#ifndef SANA_MSGS__MSG__DETAIL__SANA__FUNCTIONS_H_
#define SANA_MSGS__MSG__DETAIL__SANA__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "sana_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "sana_msgs/msg/detail/sana__struct.h"

/// Initialize msg/Sana message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * sana_msgs__msg__Sana
 * )) before or use
 * sana_msgs__msg__Sana__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_sana_msgs
bool
sana_msgs__msg__Sana__init(sana_msgs__msg__Sana * msg);

/// Finalize msg/Sana message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sana_msgs
void
sana_msgs__msg__Sana__fini(sana_msgs__msg__Sana * msg);

/// Create msg/Sana message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * sana_msgs__msg__Sana__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_sana_msgs
sana_msgs__msg__Sana *
sana_msgs__msg__Sana__create();

/// Destroy msg/Sana message.
/**
 * It calls
 * sana_msgs__msg__Sana__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sana_msgs
void
sana_msgs__msg__Sana__destroy(sana_msgs__msg__Sana * msg);

/// Check for msg/Sana message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_sana_msgs
bool
sana_msgs__msg__Sana__are_equal(const sana_msgs__msg__Sana * lhs, const sana_msgs__msg__Sana * rhs);

/// Copy a msg/Sana message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_sana_msgs
bool
sana_msgs__msg__Sana__copy(
  const sana_msgs__msg__Sana * input,
  sana_msgs__msg__Sana * output);

/// Initialize array of msg/Sana messages.
/**
 * It allocates the memory for the number of elements and calls
 * sana_msgs__msg__Sana__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_sana_msgs
bool
sana_msgs__msg__Sana__Sequence__init(sana_msgs__msg__Sana__Sequence * array, size_t size);

/// Finalize array of msg/Sana messages.
/**
 * It calls
 * sana_msgs__msg__Sana__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sana_msgs
void
sana_msgs__msg__Sana__Sequence__fini(sana_msgs__msg__Sana__Sequence * array);

/// Create array of msg/Sana messages.
/**
 * It allocates the memory for the array and calls
 * sana_msgs__msg__Sana__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_sana_msgs
sana_msgs__msg__Sana__Sequence *
sana_msgs__msg__Sana__Sequence__create(size_t size);

/// Destroy array of msg/Sana messages.
/**
 * It calls
 * sana_msgs__msg__Sana__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_sana_msgs
void
sana_msgs__msg__Sana__Sequence__destroy(sana_msgs__msg__Sana__Sequence * array);

/// Check for msg/Sana message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_sana_msgs
bool
sana_msgs__msg__Sana__Sequence__are_equal(const sana_msgs__msg__Sana__Sequence * lhs, const sana_msgs__msg__Sana__Sequence * rhs);

/// Copy an array of msg/Sana messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_sana_msgs
bool
sana_msgs__msg__Sana__Sequence__copy(
  const sana_msgs__msg__Sana__Sequence * input,
  sana_msgs__msg__Sana__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // SANA_MSGS__MSG__DETAIL__SANA__FUNCTIONS_H_
