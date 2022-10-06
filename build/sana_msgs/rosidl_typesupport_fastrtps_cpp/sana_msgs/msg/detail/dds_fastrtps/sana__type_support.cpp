// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from sana_msgs:msg/Sana.idl
// generated code does not contain a copyright notice
#include "sana_msgs/msg/detail/sana__rosidl_typesupport_fastrtps_cpp.hpp"
#include "sana_msgs/msg/detail/sana__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace sana_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sana_msgs
cdr_serialize(
  const sana_msgs::msg::Sana & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: hotword
  cdr << (ros_message.hotword ? true : false);
  // Member: sttcon
  cdr << (ros_message.sttcon ? true : false);
  // Member: cam
  cdr << (ros_message.cam ? true : false);
  // Member: arrive
  cdr << (ros_message.arrive ? true : false);
  // Member: stt
  cdr << ros_message.stt;
  // Member: intent
  cdr << ros_message.intent;
  // Member: tts
  cdr << ros_message.tts;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sana_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  sana_msgs::msg::Sana & ros_message)
{
  // Member: hotword
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.hotword = tmp ? true : false;
  }

  // Member: sttcon
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.sttcon = tmp ? true : false;
  }

  // Member: cam
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.cam = tmp ? true : false;
  }

  // Member: arrive
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.arrive = tmp ? true : false;
  }

  // Member: stt
  cdr >> ros_message.stt;

  // Member: intent
  cdr >> ros_message.intent;

  // Member: tts
  cdr >> ros_message.tts;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sana_msgs
get_serialized_size(
  const sana_msgs::msg::Sana & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: hotword
  {
    size_t item_size = sizeof(ros_message.hotword);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: sttcon
  {
    size_t item_size = sizeof(ros_message.sttcon);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: cam
  {
    size_t item_size = sizeof(ros_message.cam);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: arrive
  {
    size_t item_size = sizeof(ros_message.arrive);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: stt
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.stt.size() + 1);
  // Member: intent
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.intent.size() + 1);
  // Member: tts
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.tts.size() + 1);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_sana_msgs
max_serialized_size_Sana(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: hotword
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: sttcon
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: cam
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: arrive
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: stt
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: intent
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: tts
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

static bool _Sana__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const sana_msgs::msg::Sana *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Sana__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<sana_msgs::msg::Sana *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Sana__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const sana_msgs::msg::Sana *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Sana__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_Sana(full_bounded, 0);
}

static message_type_support_callbacks_t _Sana__callbacks = {
  "sana_msgs::msg",
  "Sana",
  _Sana__cdr_serialize,
  _Sana__cdr_deserialize,
  _Sana__get_serialized_size,
  _Sana__max_serialized_size
};

static rosidl_message_type_support_t _Sana__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Sana__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace sana_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_sana_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<sana_msgs::msg::Sana>()
{
  return &sana_msgs::msg::typesupport_fastrtps_cpp::_Sana__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, sana_msgs, msg, Sana)() {
  return &sana_msgs::msg::typesupport_fastrtps_cpp::_Sana__handle;
}

#ifdef __cplusplus
}
#endif
