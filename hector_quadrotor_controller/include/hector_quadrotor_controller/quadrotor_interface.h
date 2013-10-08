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

namespace hector_quadrotor_controller {

using namespace hardware_interface;

class QuadrotorInterface : public HardwareInterface
{
public:
  QuadrotorInterface();
  QuadrotorInterface(geometry_msgs::Pose &pose_, geometry_msgs::Twist &twist_, geometry_msgs::Pose &pose_command_, geometry_msgs::Twist &velocity_command_);
  template <typename HandleType> HandleType getHandle() const;

private:
  geometry_msgs::Pose *pose_;
  geometry_msgs::Twist *twist_;
  geometry_msgs::Pose *pose_command_;
  geometry_msgs::Twist *velocity_command_;
};
template <typename HandleType> HandleType QuadrotorInterface::getHandle() const { return HandleType(); }

class PoseStateHandle
{
public:
  PoseStateHandle() : pose_(0) {}
  PoseStateHandle(geometry_msgs::Pose& pose) : pose_(&pose) {}

  static std::string getName() { return "pose"; }
  const geometry_msgs::Pose& getPose() const { return *pose_; }

  void getEulerRPY(double &roll, double &pitch, double &yaw) const;
  double getYaw() const;

protected:
  geometry_msgs::Pose *pose_;
};
template<> inline PoseStateHandle QuadrotorInterface::getHandle<PoseStateHandle>() const { return PoseStateHandle(*pose_); }

class PoseHandle : public PoseStateHandle
{
public:
  PoseHandle() : command_(0) {}
  PoseHandle(const PoseStateHandle& state, geometry_msgs::Pose& command) : PoseStateHandle(state), command_(&command) {}

  void setCommand(const geometry_msgs::Pose& command) { *command_ = command; }
  void getCommand(geometry_msgs::Pose& command) const { command = *command_; }
  const geometry_msgs::Pose& getCommand() const { return *command_; }

protected:
  geometry_msgs::Pose *command_;
};
template<> inline PoseHandle QuadrotorInterface::getHandle<PoseHandle>() const { return PoseHandle(PoseStateHandle(*pose_), *pose_command_); }

class HorizontalPositionHandle : public PoseHandle
{
public:
  HorizontalPositionHandle() {}
  HorizontalPositionHandle(const PoseStateHandle& state, geometry_msgs::Pose& command) : PoseHandle(state, command) {}

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
};
template<> inline HorizontalPositionHandle QuadrotorInterface::getHandle<HorizontalPositionHandle>() const { return HorizontalPositionHandle(PoseStateHandle(*pose_), *pose_command_); }

class HeightHandle : public PoseHandle
{
public:
  HeightHandle() {}
  HeightHandle(const PoseStateHandle& state, geometry_msgs::Pose& command) : PoseHandle(state, command) {}

  static std::string getName() { return "pose.position.z"; }
  void setCommand(double command)
  {
    command_->position.z = command;
  }
  double getCommand() const {
    return command_->position.z;
  }
};
template<> inline HeightHandle QuadrotorInterface::getHandle<HeightHandle>() const { return HeightHandle(PoseStateHandle(*pose_), *pose_command_); }

class HeadingHandle : public PoseHandle
{
public:
  HeadingHandle() {}
  HeadingHandle(const PoseStateHandle& state, geometry_msgs::Pose& command) : PoseHandle(state, command) {}

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
template<> inline HeadingHandle QuadrotorInterface::getHandle<HeadingHandle>() const { return HeadingHandle(PoseStateHandle(*pose_), *pose_command_); }

class VelocityStateHandle
{
public:
  VelocityStateHandle() : twist_(0) {}
  VelocityStateHandle(geometry_msgs::Twist& twist) : twist_(&twist) {}

  static std::string getName() { return "velocity"; }
  const geometry_msgs::Twist& getTwist() const { return *twist_; }

protected:
  geometry_msgs::Twist *twist_;
};
template<> inline VelocityStateHandle QuadrotorInterface::getHandle<VelocityStateHandle>() const { return VelocityStateHandle(*twist_); }

class VelocityHandle : public VelocityStateHandle
{
public:
  VelocityHandle() : command_(0) {}
  VelocityHandle(const VelocityStateHandle& state, geometry_msgs::Twist& command) : VelocityStateHandle(state), command_(&command) {}

  void setCommand(const geometry_msgs::Twist& command) { *command_ = command; }
  const geometry_msgs::Twist& getCommand() const { return *command_; }

protected:
  geometry_msgs::Twist *command_;
};
template<> inline VelocityHandle QuadrotorInterface::getHandle<VelocityHandle>() const { return VelocityHandle(VelocityStateHandle(*twist_), *velocity_command_); }

class HorizontalVelocityHandle : public VelocityHandle
{
public:
  HorizontalVelocityHandle() {}
  HorizontalVelocityHandle(const VelocityStateHandle& state, geometry_msgs::Twist& command) : VelocityHandle(state, command) {}

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
template<> inline HorizontalVelocityHandle QuadrotorInterface::getHandle<HorizontalVelocityHandle>() const { return HorizontalVelocityHandle(VelocityStateHandle(*twist_), *velocity_command_); }

class VerticalVelocityHandle : public VelocityHandle
{
public:
  VerticalVelocityHandle() {}
  VerticalVelocityHandle(const VelocityStateHandle& state, geometry_msgs::Twist& command) : VelocityHandle(state, command) {}

  static std::string getName() { return "velocity.linear.z"; }
  void setCommand(double command)
  {
    command_->linear.z = command;
  }
  double getCommand() const {
    return command_->linear.z;
  }
};
template<> inline VerticalVelocityHandle QuadrotorInterface::getHandle<VerticalVelocityHandle>() const { return VerticalVelocityHandle(VelocityStateHandle(*twist_), *velocity_command_); }

class AngularVelocityHandle : public VelocityHandle
{
public:
  AngularVelocityHandle() {}
  AngularVelocityHandle(const VelocityStateHandle& state, geometry_msgs::Twist& command) : VelocityHandle(state, command) {}

  static std::string getName() { return "velocity.angular.z"; }
  void setCommand(double command)
  {
    command_->angular.z = command;
  }
  double getCommand() const {
    return command_->angular.z;
  }
};
template<> inline AngularVelocityHandle QuadrotorInterface::getHandle<AngularVelocityHandle>() const { return AngularVelocityHandle(VelocityStateHandle(*twist_), *velocity_command_); }

class StateHandle : public PoseStateHandle, public VelocityStateHandle
{
public:
  StateHandle() {}
  StateHandle(geometry_msgs::Pose& pose, geometry_msgs::Twist& twist) : PoseStateHandle(pose), VelocityStateHandle(twist) {}
  static std::string getName() { return "state"; }
};
template<> inline StateHandle QuadrotorInterface::getHandle<StateHandle>() const { return StateHandle(*pose_, *twist_); }

} // namespace hector_quadrotor_controller

#endif // HECTOR_QUADROTOR_CONTROLLER_QUADROTOR_INTERFACE_H
