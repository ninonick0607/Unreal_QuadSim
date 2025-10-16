// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rosflight_msgs:srv/SetSimState.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__SRV__DETAIL__SET_SIM_STATE__STRUCT_HPP_
#define ROSFLIGHT_MSGS__SRV__DETAIL__SET_SIM_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'state'
#include "rosflight_msgs/msg/detail/sim_state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__rosflight_msgs__srv__SetSimState_Request __attribute__((deprecated))
#else
# define DEPRECATED__rosflight_msgs__srv__SetSimState_Request __declspec(deprecated)
#endif

namespace rosflight_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetSimState_Request_
{
  using Type = SetSimState_Request_<ContainerAllocator>;

  explicit SetSimState_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : state(_init)
  {
    (void)_init;
  }

  explicit SetSimState_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : state(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _state_type =
    rosflight_msgs::msg::SimState_<ContainerAllocator>;
  _state_type state;

  // setters for named parameter idiom
  Type & set__state(
    const rosflight_msgs::msg::SimState_<ContainerAllocator> & _arg)
  {
    this->state = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rosflight_msgs::srv::SetSimState_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const rosflight_msgs::srv::SetSimState_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rosflight_msgs::srv::SetSimState_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rosflight_msgs::srv::SetSimState_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rosflight_msgs::srv::SetSimState_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rosflight_msgs::srv::SetSimState_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rosflight_msgs::srv::SetSimState_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rosflight_msgs::srv::SetSimState_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rosflight_msgs::srv::SetSimState_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rosflight_msgs::srv::SetSimState_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rosflight_msgs__srv__SetSimState_Request
    std::shared_ptr<rosflight_msgs::srv::SetSimState_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rosflight_msgs__srv__SetSimState_Request
    std::shared_ptr<rosflight_msgs::srv::SetSimState_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetSimState_Request_ & other) const
  {
    if (this->state != other.state) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetSimState_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetSimState_Request_

// alias to use template instance with default allocator
using SetSimState_Request =
  rosflight_msgs::srv::SetSimState_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rosflight_msgs


#ifndef _WIN32
# define DEPRECATED__rosflight_msgs__srv__SetSimState_Response __attribute__((deprecated))
#else
# define DEPRECATED__rosflight_msgs__srv__SetSimState_Response __declspec(deprecated)
#endif

namespace rosflight_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SetSimState_Response_
{
  using Type = SetSimState_Response_<ContainerAllocator>;

  explicit SetSimState_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit SetSimState_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rosflight_msgs::srv::SetSimState_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const rosflight_msgs::srv::SetSimState_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rosflight_msgs::srv::SetSimState_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rosflight_msgs::srv::SetSimState_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rosflight_msgs::srv::SetSimState_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rosflight_msgs::srv::SetSimState_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rosflight_msgs::srv::SetSimState_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rosflight_msgs::srv::SetSimState_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rosflight_msgs::srv::SetSimState_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rosflight_msgs::srv::SetSimState_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rosflight_msgs__srv__SetSimState_Response
    std::shared_ptr<rosflight_msgs::srv::SetSimState_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rosflight_msgs__srv__SetSimState_Response
    std::shared_ptr<rosflight_msgs::srv::SetSimState_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetSimState_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetSimState_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetSimState_Response_

// alias to use template instance with default allocator
using SetSimState_Response =
  rosflight_msgs::srv::SetSimState_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rosflight_msgs

namespace rosflight_msgs
{

namespace srv
{

struct SetSimState
{
  using Request = rosflight_msgs::srv::SetSimState_Request;
  using Response = rosflight_msgs::srv::SetSimState_Response;
};

}  // namespace srv

}  // namespace rosflight_msgs

#endif  // ROSFLIGHT_MSGS__SRV__DETAIL__SET_SIM_STATE__STRUCT_HPP_
