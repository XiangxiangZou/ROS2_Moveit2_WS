// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_robot_interfaces:msg/PoseCmd.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "my_robot_interfaces/msg/pose_cmd.hpp"


#ifndef MY_ROBOT_INTERFACES__MSG__DETAIL__POSE_CMD__BUILDER_HPP_
#define MY_ROBOT_INTERFACES__MSG__DETAIL__POSE_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "my_robot_interfaces/msg/detail/pose_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace my_robot_interfaces
{

namespace msg
{

namespace builder
{

class Init_PoseCmd_cartesian_path
{
public:
  explicit Init_PoseCmd_cartesian_path(::my_robot_interfaces::msg::PoseCmd & msg)
  : msg_(msg)
  {}
  ::my_robot_interfaces::msg::PoseCmd cartesian_path(::my_robot_interfaces::msg::PoseCmd::_cartesian_path_type arg)
  {
    msg_.cartesian_path = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_robot_interfaces::msg::PoseCmd msg_;
};

class Init_PoseCmd_yaw
{
public:
  explicit Init_PoseCmd_yaw(::my_robot_interfaces::msg::PoseCmd & msg)
  : msg_(msg)
  {}
  Init_PoseCmd_cartesian_path yaw(::my_robot_interfaces::msg::PoseCmd::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_PoseCmd_cartesian_path(msg_);
  }

private:
  ::my_robot_interfaces::msg::PoseCmd msg_;
};

class Init_PoseCmd_pitch
{
public:
  explicit Init_PoseCmd_pitch(::my_robot_interfaces::msg::PoseCmd & msg)
  : msg_(msg)
  {}
  Init_PoseCmd_yaw pitch(::my_robot_interfaces::msg::PoseCmd::_pitch_type arg)
  {
    msg_.pitch = std::move(arg);
    return Init_PoseCmd_yaw(msg_);
  }

private:
  ::my_robot_interfaces::msg::PoseCmd msg_;
};

class Init_PoseCmd_roll
{
public:
  explicit Init_PoseCmd_roll(::my_robot_interfaces::msg::PoseCmd & msg)
  : msg_(msg)
  {}
  Init_PoseCmd_pitch roll(::my_robot_interfaces::msg::PoseCmd::_roll_type arg)
  {
    msg_.roll = std::move(arg);
    return Init_PoseCmd_pitch(msg_);
  }

private:
  ::my_robot_interfaces::msg::PoseCmd msg_;
};

class Init_PoseCmd_z
{
public:
  explicit Init_PoseCmd_z(::my_robot_interfaces::msg::PoseCmd & msg)
  : msg_(msg)
  {}
  Init_PoseCmd_roll z(::my_robot_interfaces::msg::PoseCmd::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_PoseCmd_roll(msg_);
  }

private:
  ::my_robot_interfaces::msg::PoseCmd msg_;
};

class Init_PoseCmd_y
{
public:
  explicit Init_PoseCmd_y(::my_robot_interfaces::msg::PoseCmd & msg)
  : msg_(msg)
  {}
  Init_PoseCmd_z y(::my_robot_interfaces::msg::PoseCmd::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_PoseCmd_z(msg_);
  }

private:
  ::my_robot_interfaces::msg::PoseCmd msg_;
};

class Init_PoseCmd_x
{
public:
  Init_PoseCmd_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PoseCmd_y x(::my_robot_interfaces::msg::PoseCmd::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_PoseCmd_y(msg_);
  }

private:
  ::my_robot_interfaces::msg::PoseCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_robot_interfaces::msg::PoseCmd>()
{
  return my_robot_interfaces::msg::builder::Init_PoseCmd_x();
}

}  // namespace my_robot_interfaces

#endif  // MY_ROBOT_INTERFACES__MSG__DETAIL__POSE_CMD__BUILDER_HPP_
