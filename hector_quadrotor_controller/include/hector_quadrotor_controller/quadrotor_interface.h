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
  virtual ~QuadrotorInterface();

  template <typename HandleType> HandleType getHandle() { return HandleType(this); }

  virtual void setPoseCommand(const geometry_msgs::Pose &command)     { if (getPoseCommand())       *getPoseCommand() = command; }
  virtual void setTwistCommand(const geometry_msgs::Twist &command)   { if (getTwistCommand())      *getTwistCommand() = command; }
  virtual void setTrajectoryCommand(const nav_msgs::Path &trajectory) { if (getTrajectoryCommand()) *getTrajectoryCommand() = trajectory; }

  virtual geometry_msgs::Pose *getPose()                   { return 0; }
  virtual geometry_msgs::Twist *getTwist()                 { return 0; }
  virtual sensor_msgs::Imu *getSensorImu()                 { return 0; }
  virtual hector_uav_msgs::MotorStatus *getMotorStatus()   { return 0; }

  virtual nav_msgs::Path *getTrajectoryCommand()           { return &trajectory_command_; }
  virtual geometry_msgs::Pose *getPoseCommand()            { return &pose_command_; }
  virtual geometry_msgs::Twist *getTwistCommand()          { return &twist_command_; }
  virtual geometry_msgs::Wrench *getWrenchCommand()        { return &wrench_command_; }
  virtual hector_uav_msgs::MotorCommand *getMotorCommand() { return &motor_command_; }

private:
  nav_msgs::Path trajectory_command_;
  geometry_msgs::Pose pose_command_;
  geometry_msgs::Twist twist_command_;
  geometry_msgs::Wrench wrench_command_;
  hector_uav_msgs::MotorCommand motor_command_;
};

class PoseHandle
{
public:
  PoseHandle() : interface_(0) {}
  PoseHandle(QuadrotorInterface *interface) : interface_(interface) {}

  static std::string getName() { return "pose"; }
  const geometry_msgs::Pose& pose() const { return *(interface_->getPose()); }

  void getEulerRPY(double &roll, double &pitch, double &yaw) const;
  double getYaw() const;

protected:
  QuadrotorInterface *interface_;
};

class TwistHandle
{
public:
  TwistHandle() : interface_(0) {}
  TwistHandle(QuadrotorInterface *interface) : interface_(interface) {}

  static std::string getName() { return "twist"; }
  const geometry_msgs::Twist& twist() const { return *(interface_->getTwist()); }

protected:
  QuadrotorInterface *interface_;
};

class StateHandle : public PoseHandle, public TwistHandle
{
public:
  StateHandle() {}
  StateHandle(QuadrotorInterface *interface) : PoseHandle(interface), TwistHandle(interface) {}
  static std::string getName() { return "state"; }
};

class PoseCommandHandle : public PoseHandle
{
public:
  PoseCommandHandle() {}
  PoseCommandHandle(QuadrotorInterface *interface) : PoseHandle(interface) {}

  bool available() const { return interface_->getPoseCommand(); }

  void setCommand(const geometry_msgs::Pose& command) { *(interface_->getPoseCommand()) = command; }
  bool getCommand(geometry_msgs::Pose& command) const {
    if (!available()) return false;
    command = *(interface_->getPoseCommand());
    return true;
  }
  const geometry_msgs::Pose& getCommand() const { return *(interface_->getPoseCommand()); }
};

class HorizontalPositionCommandHandle : public PoseCommandHandle
{
public:
  HorizontalPositionCommandHandle() {}
  HorizontalPositionCommandHandle(const PoseCommandHandle& other) : PoseCommandHandle(other) {}
  HorizontalPositionCommandHandle(QuadrotorInterface *interface) : PoseCommandHandle(interface) {}

  static std::string getName() { return "pose.position.xy"; }
  void setCommand(double x, double y)
  {
    interface_->getPoseCommand()->position.x = x;
    interface_->getPoseCommand()->position.y = y;
  }
  void getCommand(geometry_msgs::Point& command) const {
    command.x = interface_->getPoseCommand()->position.x;
    command.y = interface_->getPoseCommand()->position.y;
  }
  void getCommand(double &x, double &y) const {
    x = interface_->getPoseCommand()->position.x;
    y = interface_->getPoseCommand()->position.y;
  }

  void getError(double &x, double &y) const;
};

class HeightCommandHandle : public PoseCommandHandle
{
public:
  HeightCommandHandle() {}
  HeightCommandHandle(const PoseCommandHandle& other) : PoseCommandHandle(other) {}
  HeightCommandHandle(QuadrotorInterface *interface) : PoseCommandHandle(interface) {}

  static std::string getName() { return "pose.position.z"; }
  void setCommand(double command)
  {
    interface_->getPoseCommand()->position.z = command;
  }
  double getCommand() const {
    return (interface_->getPoseCommand())->position.z;
  }

  double getError() const;
};

class HeadingCommandHandle : public PoseCommandHandle
{
public:
  HeadingCommandHandle() {}
  HeadingCommandHandle(const PoseCommandHandle& other) : PoseCommandHandle(other) {}
  HeadingCommandHandle(QuadrotorInterface *interface) : PoseCommandHandle(interface) {}

  std::string getName() { return "pose.orientation.yaw"; }
  void setCommand(double command)
  {
    interface_->getPoseCommand()->orientation.x = 0.0;
    interface_->getPoseCommand()->orientation.y = 0.0;
    interface_->getPoseCommand()->orientation.z = sin(command/2.);
    interface_->getPoseCommand()->orientation.w = cos(command/2.);
  }
  double getCommand() const;
  double getError() const;
};

class TwistCommandHandle : public TwistHandle
{
public:
  TwistCommandHandle() {}
  TwistCommandHandle(QuadrotorInterface *interface) : TwistHandle(interface) {}

  bool available() const { return interface_->getTwistCommand(); }

  void setCommand(const geometry_msgs::Twist& command) { *(interface_->getTwistCommand()) = command; }
  bool getCommand(geometry_msgs::Twist& command) const {
    if (!available()) return false;
    command = *(interface_->getTwistCommand());
    return true;
  }
  const geometry_msgs::Twist& getCommand() const { return *(interface_->getTwistCommand()); }
};

class HorizontalVelocityCommandHandle : public TwistCommandHandle
{
public:
  HorizontalVelocityCommandHandle() {}
  HorizontalVelocityCommandHandle(const TwistCommandHandle& other) : TwistCommandHandle(other) {}
  HorizontalVelocityCommandHandle(QuadrotorInterface *interface) : TwistCommandHandle(interface) {}

  static std::string getName() { return "twist.linear.xy"; }
  void setCommand(double x, double y)
  {
    interface_->getTwistCommand()->linear.x = x;
    interface_->getTwistCommand()->linear.y = y;
  }
  void getCommand(double &x, double &y) const {
    x = interface_->getTwistCommand()->linear.x;
    y = interface_->getTwistCommand()->linear.y;
  }
};

class VerticalVelocityCommandHandle : public TwistCommandHandle
{
public:
  VerticalVelocityCommandHandle() {}
  VerticalVelocityCommandHandle(const TwistCommandHandle& other) : TwistCommandHandle(other) {}
  VerticalVelocityCommandHandle(QuadrotorInterface *interface) : TwistCommandHandle(interface) {}

  static std::string getName() { return "twist.linear.z"; }
  void setCommand(double command)
  {
    if (!interface_->getTwistCommand()) return;
    interface_->getTwistCommand()->linear.z = command;
  }
  double getCommand() const {
    return interface_->getTwistCommand()->linear.z;
  }
};

class AngularVelocityCommandHandle : public TwistCommandHandle
{
public:
  AngularVelocityCommandHandle() {}
  AngularVelocityCommandHandle(const TwistCommandHandle& other) : TwistCommandHandle(other) {}
  AngularVelocityCommandHandle(QuadrotorInterface *interface) : TwistCommandHandle(interface) {}

  static std::string getName() { return "twist.angular.z"; }
  void setCommand(double command)
  {
    interface_->getTwistCommand()->angular.z = command;
  }
  double getCommand() const {
    return interface_->getTwistCommand()->angular.z;
  }
};

class WrenchCommandHandle
{
public:
  WrenchCommandHandle() : interface_(0) {}
  WrenchCommandHandle(QuadrotorInterface *interface) : interface_(interface) {}

  static std::string getName() { return "wrench"; }
  bool available() const { return interface_->getWrenchCommand(); }

  void setCommand(const geometry_msgs::Wrench& command) { *(interface_->getWrenchCommand()) = command; }
  bool getCommand(geometry_msgs::Wrench& command) const {
    if (!available()) return false;
    command = *(interface_->getWrenchCommand());
    return true;
  }
  const geometry_msgs::Wrench& getCommand() const { return *(interface_->getWrenchCommand()); }

protected:
  QuadrotorInterface *interface_;
};

class MotorCommandHandle
{
public:
  MotorCommandHandle() : interface_(0) {}
  MotorCommandHandle(QuadrotorInterface *interface) : interface_(interface) {}

  static std::string getName() { return "motor"; }
  bool available() const { return interface_->getMotorCommand(); }

  void setCommand(const hector_uav_msgs::MotorCommand& command) { *(interface_->getMotorCommand()) = command; }
  bool getCommand(hector_uav_msgs::MotorCommand& command) const {
    if (!available()) return false;
    command = *(interface_->getMotorCommand());
    return true;
  }
  const hector_uav_msgs::MotorCommand& getCommand() const { return *(interface_->getMotorCommand()); }

protected:
  QuadrotorInterface *interface_;
};

} // namespace hector_quadrotor_controller

#endif // HECTOR_QUADROTOR_CONTROLLER_QUADROTOR_INTERFACE_H
