// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from rosflight_msgs:srv/ParamSet.idl
// generated code does not contain a copyright notice

#ifndef ROSFLIGHT_MSGS__SRV__DETAIL__PARAM_SET__STRUCT_HPP_
#define ROSFLIGHT_MSGS__SRV__DETAIL__PARAM_SET__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__rosflight_msgs__srv__ParamSet_Request __attribute__((deprecated))
#else
# define DEPRECATED__rosflight_msgs__srv__ParamSet_Request __declspec(deprecated)
#endif

namespace rosflight_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ParamSet_Request_
{
  using Type = ParamSet_Request_<ContainerAllocator>;

  explicit ParamSet_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->value = 0.0;
    }
  }

  explicit ParamSet_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->value = 0.0;
    }
  }

  // field types and members
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _value_type =
    double;
  _value_type value;

  // setters for named parameter idiom
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__value(
    const double & _arg)
  {
    this->value = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rosflight_msgs::srv::ParamSet_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const rosflight_msgs::srv::ParamSet_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rosflight_msgs::srv::ParamSet_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rosflight_msgs::srv::ParamSet_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rosflight_msgs::srv::ParamSet_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rosflight_msgs::srv::ParamSet_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rosflight_msgs::srv::ParamSet_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rosflight_msgs::srv::ParamSet_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rosflight_msgs::srv::ParamSet_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rosflight_msgs::srv::ParamSet_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rosflight_msgs__srv__ParamSet_Request
    std::shared_ptr<rosflight_msgs::srv::ParamSet_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rosflight_msgs__srv__ParamSet_Request
    std::shared_ptr<rosflight_msgs::srv::ParamSet_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ParamSet_Request_ & other) const
  {
    if (this->name != other.name) {
      return false;
    }
    if (this->value != other.value) {
      return false;
    }
    return true;
  }
  bool operator!=(const ParamSet_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ParamSet_Request_

// alias to use template instance with default allocator
using ParamSet_Request =
  rosflight_msgs::srv::ParamSet_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rosflight_msgs


#ifndef _WIN32
# define DEPRECATED__rosflight_msgs__srv__ParamSet_Response __attribute__((deprecated))
#else
# define DEPRECATED__rosflight_msgs__srv__ParamSet_Response __declspec(deprecated)
#endif

namespace rosflight_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ParamSet_Response_
{
  using Type = ParamSet_Response_<ContainerAllocator>;

  explicit ParamSet_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->exists = false;
    }
  }

  explicit ParamSet_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->exists = false;
    }
  }

  // field types and members
  using _exists_type =
    bool;
  _exists_type exists;

  // setters for named parameter idiom
  Type & set__exists(
    const bool & _arg)
  {
    this->exists = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    rosflight_msgs::srv::ParamSet_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const rosflight_msgs::srv::ParamSet_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<rosflight_msgs::srv::ParamSet_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<rosflight_msgs::srv::ParamSet_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      rosflight_msgs::srv::ParamSet_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<rosflight_msgs::srv::ParamSet_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      rosflight_msgs::srv::ParamSet_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<rosflight_msgs::srv::ParamSet_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<rosflight_msgs::srv::ParamSet_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<rosflight_msgs::srv::ParamSet_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__rosflight_msgs__srv__ParamSet_Response
    std::shared_ptr<rosflight_msgs::srv::ParamSet_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__rosflight_msgs__srv__ParamSet_Response
    std::shared_ptr<rosflight_msgs::srv::ParamSet_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ParamSet_Response_ & other) const
  {
    if (this->exists != other.exists) {
      return false;
    }
    return true;
  }
  bool operator!=(const ParamSet_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ParamSet_Response_

// alias to use template instance with default allocator
using ParamSet_Response =
  rosflight_msgs::srv::ParamSet_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace rosflight_msgs

namespace rosflight_msgs
{

namespace srv
{

struct ParamSet
{
  using Request = rosflight_msgs::srv::ParamSet_Request;
  using Response = rosflight_msgs::srv::ParamSet_Response;
};

}  // namespace srv

}  // namespace rosflight_msgs

#endif  // ROSFLIGHT_MSGS__SRV__DETAIL__PARAM_SET__STRUCT_HPP_
