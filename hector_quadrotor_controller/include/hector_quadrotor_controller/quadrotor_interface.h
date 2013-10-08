//=================================================================================================
// Copyright (c) 2013, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef HECTOR_QUADROTOR_CONTROLLER_QUADROTOR_INTERFACE_H
#define HECTOR_QUADROTOR_CONTROLLER_QUADROTOR_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/Imu.h>
#include <hector_uav_msgs/MotorStatus.h>
#include <hector_uav_msgs/MotorCommand.h>
#include <nav_msgs/Path.h>

namespace hector_quadrotor_controller {

using namespace hardware_interface;

class QuadrotorInterface : public HardwareInterface
{
public:
  QuadrotorInterface();
  QuadrotorInterface(
    geometry_msgs::Pose &pose,
    geometry_msgs::Twist &twist,
    sensor_msgs::Imu &imu,
    hector_uav_msgs::MotorStatus &motor_status,
    geometry_msgs::Pose &pose_command,
    geometry_msgs::Twist &twist_command,
    geometry_msgs::Wrench &wrench_command,
    hector_uav_msgs::MotorCommand &motor_command,
    nav_msgs::Path &trajectory_command
  );
  template <typename CommandHandleType> CommandHandleType getHandle() const;

  void setPoseCommand(const geometry_msgs::Pose &command)
  {
    *pose_command_ = command;
  }

  void setTwistCommand(const geometry_msgs::Twist &command)
  {
    *twist_command_ = command;
  }

  void setTrajectoryCommand(const nav_msgs::Path &trajectory)
  {
    *trajectory_command_ = trajectory;
  }

private:
  geometry_msgs::Pose *pose_;
  geometry_msgs::Twist *twist_;
  sensor_msgs::Imu *imu_;
  hector_uav_msgs::MotorStatus *motor_status_;

  geometry_msgs::Pose *pose_command_;
  geometry_msgs::Twist *twist_command_;
  geometry_msgs::Wrench *wrench_command_;
  hector_uav_msgs::MotorCommand *motor_command_;
  nav_msgs::Path *trajectory_command_;
};
template <typename CommandHandleType> CommandHandleType QuadrotorInterface::getHandle() const { return CommandHandleType(); }

class PoseHandle
{
public:
  PoseHandle() : pose_(0) {}
  PoseHandle(geometry_msgs::Pose& pose) : pose_(&pose) {}

  static std::string getName() { return "pose"; }
  const geometry_msgs::Pose& getPose() const { return *pose_; }

  void getEulerRPY(double &roll, double &pitch, double &yaw) const;
  double getYaw() const;

protected:
  geometry_msgs::Pose *pose_;
};
template<> inline PoseHandle QuadrotorInterface::getHandle<PoseHandle>() const { return PoseHandle(*pose_); }

class VelocityHandle
{
public:
  VelocityHandle() : twist_(0) {}
  VelocityHandle(geometry_msgs::Twist& twist) : twist_(&twist) {}

  static std::string getName() { return "velocity"; }
  const geometry_msgs::Twist& getTwist() const { return *twist_; }

protected:
  geometry_msgs::Twist *twist_;
};
template<> inline VelocityHandle QuadrotorInterface::getHandle<VelocityHandle>() const { return VelocityHandle(*twist_); }

class StateHandle : public PoseHandle, public VelocityHandle
{
public:
  StateHandle() {}
  StateHandle(geometry_msgs::Pose& pose, geometry_msgs::Twist& twist) : PoseHandle(pose), VelocityHandle(twist) {}
  static std::string getName() { return "state"; }
};
template<> inline StateHandle QuadrotorInterface::getHandle<StateHandle>() const { return StateHandle(*pose_, *twist_); }

class PoseCommandHandle : public PoseHandle
{
public:
  PoseCommandHandle() : command_(0) {}
  PoseCommandHandle(const PoseHandle& state, geometry_msgs::Pose& command) : PoseHandle(state), command_(&command) {}

  void setCommand(const geometry_msgs::Pose& command) { *command_ = command; }
  void getCommand(geometry_msgs::Pose& command) const { command = *command_; }
  const geometry_msgs::Pose& getCommand() const { return *command_; }

protected:
  geometry_msgs::Pose *command_;
};
template<> inline PoseCommandHandle QuadrotorInterface::getHandle<PoseCommandHandle>() const { return PoseCommandHandle(PoseHandle(*pose_), *pose_command_); }

class HorizontalPositionCommandHandle : public PoseCommandHandle
{
public:
  HorizontalPositionCommandHandle() {}
  HorizontalPositionCommandHandle(const PoseCommandHandle& other) : PoseCommandHandle(other) {}
  HorizontalPositionCommandHandle(const PoseHandle& state, geometry_msgs::Pose& command) : PoseCommandHandle(state, command) {}

  static std::string getName() { return "pose.position.xy"; }
  void setCommand(double x, double y)
  {
    command_->position.x = x;
    command_->position.y = y;
  }
  void getCommand(geometry_msgs::Point& command) const {
    command.x = command_->position.x;
    command.y = command_->position.y;
  }
  void getCommand(double &x, double &y) const {
    x = command_->position.x;
    y = command_->position.y;
  }

  void getError(double &x, double &y) const;
};
template<> inline HorizontalPositionCommandHandle QuadrotorInterface::getHandle<HorizontalPositionCommandHandle>() const { return HorizontalPositionCommandHandle(PoseHandle(*pose_), *pose_command_); }

class HeightCommandHandle : public PoseCommandHandle
{
public:
  HeightCommandHandle() {}
  HeightCommandHandle(const PoseCommandHandle& other) : PoseCommandHandle(other) {}
  HeightCommandHandle(const PoseHandle& state, geometry_msgs::Pose& command) : PoseCommandHandle(state, command) {}

  static std::string getName() { return "pose.position.z"; }
  void setCommand(double command)
  {
    command_->position.z = command;
  }
  double getCommand() const {
    return command_->position.z;
  }

  double getError() const;
};
template<> inline HeightCommandHandle QuadrotorInterface::getHandle<HeightCommandHandle>() const { return HeightCommandHandle(PoseHandle(*pose_), *pose_command_); }

class HeadingCommandHandle : public PoseCommandHandle
{
public:
  HeadingCommandHandle() {}
  HeadingCommandHandle(const PoseCommandHandle& other) : PoseCommandHandle(other) {}
  HeadingCommandHandle(const PoseHandle& state, geometry_msgs::Pose& command) : PoseCommandHandle(state, command) {}

  std::string getName() { return "pose.orientation.yaw"; }
  void setCommand(double command)
  {
    command_->orientation.x = 0.0;
    command_->orientation.y = 0.0;
    command_->orientation.z = sin(command/2.);
    command_->orientation.w = cos(command/2.);
  }
  double getCommand() const;
  double getError() const;
};
template<> inline HeadingCommandHandle QuadrotorInterface::getHandle<HeadingCommandHandle>() const { return HeadingCommandHandle(PoseHandle(*pose_), *pose_command_); }

class VelocityCommandHandle : public VelocityHandle
{
public:
  VelocityCommandHandle() : command_(0) {}
  VelocityCommandHandle(const VelocityHandle& state, geometry_msgs::Twist& command) : VelocityHandle(state), command_(&command) {}

  void setCommand(const geometry_msgs::Twist& command) { *command_ = command; }
  const geometry_msgs::Twist& getCommand() const { return *command_; }

protected:
  geometry_msgs::Twist *command_;
};
template<> inline VelocityCommandHandle QuadrotorInterface::getHandle<VelocityCommandHandle>() const { return VelocityCommandHandle(VelocityHandle(*twist_), *twist_command_); }

class HorizontalVelocityCommandHandle : public VelocityCommandHandle
{
public:
  HorizontalVelocityCommandHandle() {}
  HorizontalVelocityCommandHandle(const VelocityCommandHandle& other) : VelocityCommandHandle(other) {}
  HorizontalVelocityCommandHandle(const VelocityHandle& state, geometry_msgs::Twist& command) : VelocityCommandHandle(state, command) {}

  static std::string getName() { return "velocity.linear.xy"; }
  void setCommand(double x, double y)
  {
    command_->linear.x = x;
    command_->linear.y = y;
  }
  void getCommand(double &x, double &y) const {
    x = command_->linear.x;
    y = command_->linear.y;
  }
};
template<> inline HorizontalVelocityCommandHandle QuadrotorInterface::getHandle<HorizontalVelocityCommandHandle>() const { return HorizontalVelocityCommandHandle(VelocityHandle(*twist_), *twist_command_); }

class VerticalVelocityCommandHandle : public VelocityCommandHandle
{
public:
  VerticalVelocityCommandHandle() {}
  VerticalVelocityCommandHandle(const VelocityCommandHandle& other) : VelocityCommandHandle(other) {}
  VerticalVelocityCommandHandle(const VelocityHandle& state, geometry_msgs::Twist& command) : VelocityCommandHandle(state, command) {}

  static std::string getName() { return "velocity.linear.z"; }
  void setCommand(double command)
  {
    command_->linear.z = command;
  }
  double getCommand() const {
    return command_->linear.z;
  }
};
template<> inline VerticalVelocityCommandHandle QuadrotorInterface::getHandle<VerticalVelocityCommandHandle>() const { return VerticalVelocityCommandHandle(VelocityHandle(*twist_), *twist_command_); }

class AngularVelocityCommandHandle : public VelocityCommandHandle
{
public:
  AngularVelocityCommandHandle() {}
  AngularVelocityCommandHandle(const VelocityCommandHandle& other) : VelocityCommandHandle(other) {}
  AngularVelocityCommandHandle(const VelocityHandle& state, geometry_msgs::Twist& command) : VelocityCommandHandle(state, command) {}

  static std::string getName() { return "velocity.angular.z"; }
  void setCommand(double command)
  {
    command_->angular.z = command;
  }
  double getCommand() const {
    return command_->angular.z;
  }
};
template<> inline AngularVelocityCommandHandle QuadrotorInterface::getHandle<AngularVelocityCommandHandle>() const { return AngularVelocityCommandHandle(VelocityHandle(*twist_), *twist_command_); }

} // namespace hector_quadrotor_controller

#endif // HECTOR_QUADROTOR_CONTROLLER_QUADROTOR_INTERFACE_H
