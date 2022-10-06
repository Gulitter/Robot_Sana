// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from sana_msgs:msg/Sana.idl
// generated code does not contain a copyright notice

#ifndef SANA_MSGS__MSG__DETAIL__SANA__BUILDER_HPP_
#define SANA_MSGS__MSG__DETAIL__SANA__BUILDER_HPP_

#include "sana_msgs/msg/detail/sana__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace sana_msgs
{

namespace msg
{

namespace builder
{

class Init_Sana_tts
{
public:
  explicit Init_Sana_tts(::sana_msgs::msg::Sana & msg)
  : msg_(msg)
  {}
  ::sana_msgs::msg::Sana tts(::sana_msgs::msg::Sana::_tts_type arg)
  {
    msg_.tts = std::move(arg);
    return std::move(msg_);
  }

private:
  ::sana_msgs::msg::Sana msg_;
};

class Init_Sana_intent
{
public:
  explicit Init_Sana_intent(::sana_msgs::msg::Sana & msg)
  : msg_(msg)
  {}
  Init_Sana_tts intent(::sana_msgs::msg::Sana::_intent_type arg)
  {
    msg_.intent = std::move(arg);
    return Init_Sana_tts(msg_);
  }

private:
  ::sana_msgs::msg::Sana msg_;
};

class Init_Sana_stt
{
public:
  explicit Init_Sana_stt(::sana_msgs::msg::Sana & msg)
  : msg_(msg)
  {}
  Init_Sana_intent stt(::sana_msgs::msg::Sana::_stt_type arg)
  {
    msg_.stt = std::move(arg);
    return Init_Sana_intent(msg_);
  }

private:
  ::sana_msgs::msg::Sana msg_;
};

class Init_Sana_arrive
{
public:
  explicit Init_Sana_arrive(::sana_msgs::msg::Sana & msg)
  : msg_(msg)
  {}
  Init_Sana_stt arrive(::sana_msgs::msg::Sana::_arrive_type arg)
  {
    msg_.arrive = std::move(arg);
    return Init_Sana_stt(msg_);
  }

private:
  ::sana_msgs::msg::Sana msg_;
};

class Init_Sana_cam
{
public:
  explicit Init_Sana_cam(::sana_msgs::msg::Sana & msg)
  : msg_(msg)
  {}
  Init_Sana_arrive cam(::sana_msgs::msg::Sana::_cam_type arg)
  {
    msg_.cam = std::move(arg);
    return Init_Sana_arrive(msg_);
  }

private:
  ::sana_msgs::msg::Sana msg_;
};

class Init_Sana_sttcon
{
public:
  explicit Init_Sana_sttcon(::sana_msgs::msg::Sana & msg)
  : msg_(msg)
  {}
  Init_Sana_cam sttcon(::sana_msgs::msg::Sana::_sttcon_type arg)
  {
    msg_.sttcon = std::move(arg);
    return Init_Sana_cam(msg_);
  }

private:
  ::sana_msgs::msg::Sana msg_;
};

class Init_Sana_hotword
{
public:
  Init_Sana_hotword()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Sana_sttcon hotword(::sana_msgs::msg::Sana::_hotword_type arg)
  {
    msg_.hotword = std::move(arg);
    return Init_Sana_sttcon(msg_);
  }

private:
  ::sana_msgs::msg::Sana msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::sana_msgs::msg::Sana>()
{
  return sana_msgs::msg::builder::Init_Sana_hotword();
}

}  // namespace sana_msgs

#endif  // SANA_MSGS__MSG__DETAIL__SANA__BUILDER_HPP_
