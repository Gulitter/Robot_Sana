// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from sana_msgs:msg/Sana.idl
// generated code does not contain a copyright notice
#include "sana_msgs/msg/detail/sana__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `stt`
// Member `intent`
// Member `tts`
#include "rosidl_runtime_c/string_functions.h"

bool
sana_msgs__msg__Sana__init(sana_msgs__msg__Sana * msg)
{
  if (!msg) {
    return false;
  }
  // hotword
  // sttcon
  // cam
  // arrive
  // stt
  if (!rosidl_runtime_c__String__init(&msg->stt)) {
    sana_msgs__msg__Sana__fini(msg);
    return false;
  }
  // intent
  if (!rosidl_runtime_c__String__init(&msg->intent)) {
    sana_msgs__msg__Sana__fini(msg);
    return false;
  }
  // tts
  if (!rosidl_runtime_c__String__init(&msg->tts)) {
    sana_msgs__msg__Sana__fini(msg);
    return false;
  }
  return true;
}

void
sana_msgs__msg__Sana__fini(sana_msgs__msg__Sana * msg)
{
  if (!msg) {
    return;
  }
  // hotword
  // sttcon
  // cam
  // arrive
  // stt
  rosidl_runtime_c__String__fini(&msg->stt);
  // intent
  rosidl_runtime_c__String__fini(&msg->intent);
  // tts
  rosidl_runtime_c__String__fini(&msg->tts);
}

bool
sana_msgs__msg__Sana__are_equal(const sana_msgs__msg__Sana * lhs, const sana_msgs__msg__Sana * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // hotword
  if (lhs->hotword != rhs->hotword) {
    return false;
  }
  // sttcon
  if (lhs->sttcon != rhs->sttcon) {
    return false;
  }
  // cam
  if (lhs->cam != rhs->cam) {
    return false;
  }
  // arrive
  if (lhs->arrive != rhs->arrive) {
    return false;
  }
  // stt
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->stt), &(rhs->stt)))
  {
    return false;
  }
  // intent
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->intent), &(rhs->intent)))
  {
    return false;
  }
  // tts
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->tts), &(rhs->tts)))
  {
    return false;
  }
  return true;
}

bool
sana_msgs__msg__Sana__copy(
  const sana_msgs__msg__Sana * input,
  sana_msgs__msg__Sana * output)
{
  if (!input || !output) {
    return false;
  }
  // hotword
  output->hotword = input->hotword;
  // sttcon
  output->sttcon = input->sttcon;
  // cam
  output->cam = input->cam;
  // arrive
  output->arrive = input->arrive;
  // stt
  if (!rosidl_runtime_c__String__copy(
      &(input->stt), &(output->stt)))
  {
    return false;
  }
  // intent
  if (!rosidl_runtime_c__String__copy(
      &(input->intent), &(output->intent)))
  {
    return false;
  }
  // tts
  if (!rosidl_runtime_c__String__copy(
      &(input->tts), &(output->tts)))
  {
    return false;
  }
  return true;
}

sana_msgs__msg__Sana *
sana_msgs__msg__Sana__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sana_msgs__msg__Sana * msg = (sana_msgs__msg__Sana *)allocator.allocate(sizeof(sana_msgs__msg__Sana), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(sana_msgs__msg__Sana));
  bool success = sana_msgs__msg__Sana__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
sana_msgs__msg__Sana__destroy(sana_msgs__msg__Sana * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    sana_msgs__msg__Sana__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
sana_msgs__msg__Sana__Sequence__init(sana_msgs__msg__Sana__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sana_msgs__msg__Sana * data = NULL;

  if (size) {
    data = (sana_msgs__msg__Sana *)allocator.zero_allocate(size, sizeof(sana_msgs__msg__Sana), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = sana_msgs__msg__Sana__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        sana_msgs__msg__Sana__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
sana_msgs__msg__Sana__Sequence__fini(sana_msgs__msg__Sana__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      sana_msgs__msg__Sana__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

sana_msgs__msg__Sana__Sequence *
sana_msgs__msg__Sana__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  sana_msgs__msg__Sana__Sequence * array = (sana_msgs__msg__Sana__Sequence *)allocator.allocate(sizeof(sana_msgs__msg__Sana__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = sana_msgs__msg__Sana__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
sana_msgs__msg__Sana__Sequence__destroy(sana_msgs__msg__Sana__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    sana_msgs__msg__Sana__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
sana_msgs__msg__Sana__Sequence__are_equal(const sana_msgs__msg__Sana__Sequence * lhs, const sana_msgs__msg__Sana__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!sana_msgs__msg__Sana__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
sana_msgs__msg__Sana__Sequence__copy(
  const sana_msgs__msg__Sana__Sequence * input,
  sana_msgs__msg__Sana__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(sana_msgs__msg__Sana);
    sana_msgs__msg__Sana * data =
      (sana_msgs__msg__Sana *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!sana_msgs__msg__Sana__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          sana_msgs__msg__Sana__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!sana_msgs__msg__Sana__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
