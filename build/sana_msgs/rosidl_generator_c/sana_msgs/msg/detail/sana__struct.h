// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from sana_msgs:msg/Sana.idl
// generated code does not contain a copyright notice

#ifndef SANA_MSGS__MSG__DETAIL__SANA__STRUCT_H_
#define SANA_MSGS__MSG__DETAIL__SANA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'stt'
// Member 'intent'
// Member 'tts'
#include "rosidl_runtime_c/string.h"

// Struct defined in msg/Sana in the package sana_msgs.
typedef struct sana_msgs__msg__Sana
{
  bool hotword;
  bool sttcon;
  bool cam;
  bool arrive;
  rosidl_runtime_c__String stt;
  rosidl_runtime_c__String intent;
  rosidl_runtime_c__String tts;
} sana_msgs__msg__Sana;

// Struct for a sequence of sana_msgs__msg__Sana.
typedef struct sana_msgs__msg__Sana__Sequence
{
  sana_msgs__msg__Sana * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} sana_msgs__msg__Sana__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SANA_MSGS__MSG__DETAIL__SANA__STRUCT_H_
