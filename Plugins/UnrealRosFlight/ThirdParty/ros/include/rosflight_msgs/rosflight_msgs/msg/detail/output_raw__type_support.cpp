// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from rosflight_msgs:msg/OutputRaw.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "rosflight_msgs/msg/detail/output_raw__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace rosflight_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void OutputRaw_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) rosflight_msgs::msg::OutputRaw(_init);
}

void OutputRaw_fini_function(void * message_memory)
{
  auto typed_message = static_cast<rosflight_msgs::msg::OutputRaw *>(message_memory);
  typed_message->~OutputRaw();
}

size_t size_function__OutputRaw__values(const void * untyped_member)
{
  (void)untyped_member;
  return 14;
}

const void * get_const_function__OutputRaw__values(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<float, 14> *>(untyped_member);
  return &member[index];
}

void * get_function__OutputRaw__values(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<float, 14> *>(untyped_member);
  return &member[index];
}

void fetch_function__OutputRaw__values(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__OutputRaw__values(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__OutputRaw__values(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__OutputRaw__values(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember OutputRaw_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(rosflight_msgs::msg::OutputRaw, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "values",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    14,  // array size
    false,  // is upper bound
    offsetof(rosflight_msgs::msg::OutputRaw, values),  // bytes offset in struct
    nullptr,  // default value
    size_function__OutputRaw__values,  // size() function pointer
    get_const_function__OutputRaw__values,  // get_const(index) function pointer
    get_function__OutputRaw__values,  // get(index) function pointer
    fetch_function__OutputRaw__values,  // fetch(index, &value) function pointer
    assign_function__OutputRaw__values,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers OutputRaw_message_members = {
  "rosflight_msgs::msg",  // message namespace
  "OutputRaw",  // message name
  2,  // number of fields
  sizeof(rosflight_msgs::msg::OutputRaw),
  OutputRaw_message_member_array,  // message members
  OutputRaw_init_function,  // function to initialize message memory (memory has to be allocated)
  OutputRaw_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t OutputRaw_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &OutputRaw_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace rosflight_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<rosflight_msgs::msg::OutputRaw>()
{
  return &::rosflight_msgs::msg::rosidl_typesupport_introspection_cpp::OutputRaw_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, rosflight_msgs, msg, OutputRaw)() {
  return &::rosflight_msgs::msg::rosidl_typesupport_introspection_cpp::OutputRaw_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
