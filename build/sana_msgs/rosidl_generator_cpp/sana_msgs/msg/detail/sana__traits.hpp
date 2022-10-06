// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from sana_msgs:msg/Sana.idl
// generated code does not contain a copyright notice

#ifndef SANA_MSGS__MSG__DETAIL__SANA__TRAITS_HPP_
#define SANA_MSGS__MSG__DETAIL__SANA__TRAITS_HPP_

#include "sana_msgs/msg/detail/sana__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<sana_msgs::msg::Sana>()
{
  return "sana_msgs::msg::Sana";
}

template<>
inline const char * name<sana_msgs::msg::Sana>()
{
  return "sana_msgs/msg/Sana";
}

template<>
struct has_fixed_size<sana_msgs::msg::Sana>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<sana_msgs::msg::Sana>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<sana_msgs::msg::Sana>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SANA_MSGS__MSG__DETAIL__SANA__TRAITS_HPP_
