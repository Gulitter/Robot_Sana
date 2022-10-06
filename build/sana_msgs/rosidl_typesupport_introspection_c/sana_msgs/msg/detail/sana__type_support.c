// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from sana_msgs:msg/Sana.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "sana_msgs/msg/detail/sana__rosidl_typesupport_introspection_c.h"
#include "sana_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "sana_msgs/msg/detail/sana__functions.h"
#include "sana_msgs/msg/detail/sana__struct.h"


// Include directives for member types
// Member `stt`
// Member `intent`
// Member `tts`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void Sana__rosidl_typesupport_introspection_c__Sana_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  sana_msgs__msg__Sana__init(message_memory);
}

void Sana__rosidl_typesupport_introspection_c__Sana_fini_function(void * message_memory)
{
  sana_msgs__msg__Sana__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember Sana__rosidl_typesupport_introspection_c__Sana_message_member_array[7] = {
  {
    "hotword",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sana_msgs__msg__Sana, hotword),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "sttcon",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sana_msgs__msg__Sana, sttcon),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cam",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sana_msgs__msg__Sana, cam),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "arrive",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sana_msgs__msg__Sana, arrive),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stt",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sana_msgs__msg__Sana, stt),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "intent",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sana_msgs__msg__Sana, intent),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tts",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(sana_msgs__msg__Sana, tts),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers Sana__rosidl_typesupport_introspection_c__Sana_message_members = {
  "sana_msgs__msg",  // message namespace
  "Sana",  // message name
  7,  // number of fields
  sizeof(sana_msgs__msg__Sana),
  Sana__rosidl_typesupport_introspection_c__Sana_message_member_array,  // message members
  Sana__rosidl_typesupport_introspection_c__Sana_init_function,  // function to initialize message memory (memory has to be allocated)
  Sana__rosidl_typesupport_introspection_c__Sana_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t Sana__rosidl_typesupport_introspection_c__Sana_message_type_support_handle = {
  0,
  &Sana__rosidl_typesupport_introspection_c__Sana_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_sana_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sana_msgs, msg, Sana)() {
  if (!Sana__rosidl_typesupport_introspection_c__Sana_message_type_support_handle.typesupport_identifier) {
    Sana__rosidl_typesupport_introspection_c__Sana_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &Sana__rosidl_typesupport_introspection_c__Sana_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
