// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from sana_msgs:msg/Sana.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "sana_msgs/msg/detail/sana__struct.h"
#include "sana_msgs/msg/detail/sana__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool sana_msgs__msg__sana__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[25];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("sana_msgs.msg._sana.Sana", full_classname_dest, 24) == 0);
  }
  sana_msgs__msg__Sana * ros_message = _ros_message;
  {  // hotword
    PyObject * field = PyObject_GetAttrString(_pymsg, "hotword");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->hotword = (Py_True == field);
    Py_DECREF(field);
  }
  {  // sttcon
    PyObject * field = PyObject_GetAttrString(_pymsg, "sttcon");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->sttcon = (Py_True == field);
    Py_DECREF(field);
  }
  {  // cam
    PyObject * field = PyObject_GetAttrString(_pymsg, "cam");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->cam = (Py_True == field);
    Py_DECREF(field);
  }
  {  // arrive
    PyObject * field = PyObject_GetAttrString(_pymsg, "arrive");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->arrive = (Py_True == field);
    Py_DECREF(field);
  }
  {  // stt
    PyObject * field = PyObject_GetAttrString(_pymsg, "stt");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->stt, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // intent
    PyObject * field = PyObject_GetAttrString(_pymsg, "intent");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->intent, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // tts
    PyObject * field = PyObject_GetAttrString(_pymsg, "tts");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->tts, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * sana_msgs__msg__sana__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Sana */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("sana_msgs.msg._sana");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Sana");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  sana_msgs__msg__Sana * ros_message = (sana_msgs__msg__Sana *)raw_ros_message;
  {  // hotword
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->hotword ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "hotword", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sttcon
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->sttcon ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sttcon", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cam
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->cam ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cam", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // arrive
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->arrive ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "arrive", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // stt
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->stt.data,
      strlen(ros_message->stt.data),
      "strict");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "stt", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // intent
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->intent.data,
      strlen(ros_message->intent.data),
      "strict");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "intent", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // tts
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->tts.data,
      strlen(ros_message->tts.data),
      "strict");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "tts", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
