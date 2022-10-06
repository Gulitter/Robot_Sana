// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from sana_msgs:msg/Sana.idl
// generated code does not contain a copyright notice

#ifndef SANA_MSGS__MSG__DETAIL__SANA__STRUCT_HPP_
#define SANA_MSGS__MSG__DETAIL__SANA__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__sana_msgs__msg__Sana __attribute__((deprecated))
#else
# define DEPRECATED__sana_msgs__msg__Sana __declspec(deprecated)
#endif

namespace sana_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Sana_
{
  using Type = Sana_<ContainerAllocator>;

  explicit Sana_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->hotword = false;
      this->sttcon = false;
      this->cam = false;
      this->arrive = false;
      this->stt = "";
      this->intent = "";
      this->tts = "";
    }
  }

  explicit Sana_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stt(_alloc),
    intent(_alloc),
    tts(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->hotword = false;
      this->sttcon = false;
      this->cam = false;
      this->arrive = false;
      this->stt = "";
      this->intent = "";
      this->tts = "";
    }
  }

  // field types and members
  using _hotword_type =
    bool;
  _hotword_type hotword;
  using _sttcon_type =
    bool;
  _sttcon_type sttcon;
  using _cam_type =
    bool;
  _cam_type cam;
  using _arrive_type =
    bool;
  _arrive_type arrive;
  using _stt_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _stt_type stt;
  using _intent_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _intent_type intent;
  using _tts_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _tts_type tts;

  // setters for named parameter idiom
  Type & set__hotword(
    const bool & _arg)
  {
    this->hotword = _arg;
    return *this;
  }
  Type & set__sttcon(
    const bool & _arg)
  {
    this->sttcon = _arg;
    return *this;
  }
  Type & set__cam(
    const bool & _arg)
  {
    this->cam = _arg;
    return *this;
  }
  Type & set__arrive(
    const bool & _arg)
  {
    this->arrive = _arg;
    return *this;
  }
  Type & set__stt(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->stt = _arg;
    return *this;
  }
  Type & set__intent(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->intent = _arg;
    return *this;
  }
  Type & set__tts(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->tts = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    sana_msgs::msg::Sana_<ContainerAllocator> *;
  using ConstRawPtr =
    const sana_msgs::msg::Sana_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<sana_msgs::msg::Sana_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<sana_msgs::msg::Sana_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      sana_msgs::msg::Sana_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<sana_msgs::msg::Sana_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      sana_msgs::msg::Sana_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<sana_msgs::msg::Sana_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<sana_msgs::msg::Sana_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<sana_msgs::msg::Sana_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__sana_msgs__msg__Sana
    std::shared_ptr<sana_msgs::msg::Sana_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__sana_msgs__msg__Sana
    std::shared_ptr<sana_msgs::msg::Sana_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Sana_ & other) const
  {
    if (this->hotword != other.hotword) {
      return false;
    }
    if (this->sttcon != other.sttcon) {
      return false;
    }
    if (this->cam != other.cam) {
      return false;
    }
    if (this->arrive != other.arrive) {
      return false;
    }
    if (this->stt != other.stt) {
      return false;
    }
    if (this->intent != other.intent) {
      return false;
    }
    if (this->tts != other.tts) {
      return false;
    }
    return true;
  }
  bool operator!=(const Sana_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Sana_

// alias to use template instance with default allocator
using Sana =
  sana_msgs::msg::Sana_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace sana_msgs

#endif  // SANA_MSGS__MSG__DETAIL__SANA__STRUCT_HPP_
