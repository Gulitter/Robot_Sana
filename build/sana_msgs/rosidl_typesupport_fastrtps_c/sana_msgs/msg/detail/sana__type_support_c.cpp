// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from sana_msgs:msg/Sana.idl
// generated code does not contain a copyright notice
#include "sana_msgs/msg/detail/sana__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "sana_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "sana_msgs/msg/detail/sana__struct.h"
#include "sana_msgs/msg/detail/sana__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "rosidl_runtime_c/string.h"  // intent, stt, tts
#include "rosidl_runtime_c/string_functions.h"  // intent, stt, tts

// forward declare type support functions


using _Sana__ros_msg_type = sana_msgs__msg__Sana;

static bool _Sana__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Sana__ros_msg_type * ros_message = static_cast<const _Sana__ros_msg_type *>(untyped_ros_message);
  // Field name: hotword
  {
    cdr << (ros_message->hotword ? true : false);
  }

  // Field name: sttcon
  {
    cdr << (ros_message->sttcon ? true : false);
  }

  // Field name: cam
  {
    cdr << (ros_message->cam ? true : false);
  }

  // Field name: arrive
  {
    cdr << (ros_message->arrive ? true : false);
  }

  // Field name: stt
  {
    const rosidl_runtime_c__String * str = &ros_message->stt;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: intent
  {
    const rosidl_runtime_c__String * str = &ros_message->intent;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: tts
  {
    const rosidl_runtime_c__String * str = &ros_message->tts;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _Sana__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Sana__ros_msg_type * ros_message = static_cast<_Sana__ros_msg_type *>(untyped_ros_message);
  // Field name: hotword
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->hotword = tmp ? true : false;
  }

  // Field name: sttcon
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->sttcon = tmp ? true : false;
  }

  // Field name: cam
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->cam = tmp ? true : false;
  }

  // Field name: arrive
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->arrive = tmp ? true : false;
  }

  // Field name: stt
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->stt.data) {
      rosidl_runtime_c__String__init(&ros_message->stt);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->stt,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'stt'\n");
      return false;
    }
  }

  // Field name: intent
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->intent.data) {
      rosidl_runtime_c__String__init(&ros_message->intent);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->intent,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'intent'\n");
      return false;
    }
  }

  // Field name: tts
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->tts.data) {
      rosidl_runtime_c__String__init(&ros_message->tts);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->tts,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'tts'\n");
      return false;
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sana_msgs
size_t get_serialized_size_sana_msgs__msg__Sana(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Sana__ros_msg_type * ros_message = static_cast<const _Sana__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name hotword
  {
    size_t item_size = sizeof(ros_message->hotword);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name sttcon
  {
    size_t item_size = sizeof(ros_message->sttcon);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name cam
  {
    size_t item_size = sizeof(ros_message->cam);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name arrive
  {
    size_t item_size = sizeof(ros_message->arrive);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name stt
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->stt.size + 1);
  // field.name intent
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->intent.size + 1);
  // field.name tts
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->tts.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _Sana__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_sana_msgs__msg__Sana(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_sana_msgs
size_t max_serialized_size_sana_msgs__msg__Sana(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: hotword
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: sttcon
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: cam
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: arrive
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: stt
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: intent
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: tts
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _Sana__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_sana_msgs__msg__Sana(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_Sana = {
  "sana_msgs::msg",
  "Sana",
  _Sana__cdr_serialize,
  _Sana__cdr_deserialize,
  _Sana__get_serialized_size,
  _Sana__max_serialized_size
};

static rosidl_message_type_support_t _Sana__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Sana,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, sana_msgs, msg, Sana)() {
  return &_Sana__type_support;
}

#if defined(__cplusplus)
}
#endif
