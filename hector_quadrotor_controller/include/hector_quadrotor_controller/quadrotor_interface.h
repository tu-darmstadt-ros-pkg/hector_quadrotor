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

namespace hector_quadrotor_controller {

using namespace hardware_interface;

using geometry_msgs::Pose;
using geometry_msgs::Point;
using geometry_msgs::Quaternion;
using geometry_msgs::Twist;
using geometry_msgs::Vector3;
using geometry_msgs::Wrench;
using sensor_msgs::Imu;
using hector_uav_msgs::MotorStatus;
using hector_uav_msgs::MotorCommand;

class CommandHandle;

class QuadrotorInterface : public HardwareInterface
{
public:
  QuadrotorInterface();
  virtual ~QuadrotorInterface();

  template <typename HandleType> HandleType getHandle() { return HandleType(this); }

  virtual Pose *getPose()                 { return 0; }
  virtual Twist *getTwist()               { return 0; }
  virtual Imu *getSensorImu()             { return 0; }
  virtual MotorStatus *getMotorStatus()   { return 0; }

  virtual Pose *getPoseCommand()          { return &pose_command_; }
  virtual Twist *getTwistCommand()        { return &twist_command_; }
  virtual Wrench *getWrenchCommand()      { return &wrench_command_; }
  virtual MotorCommand *getMotorCommand() { return &motor_command_; }

  bool enabled(const CommandHandle *handle) const;
  bool start(const CommandHandle *handle);
  void stop(const CommandHandle *handle);

private:
  Pose pose_command_;
  Twist twist_command_;
  Wrench wrench_command_;
  MotorCommand motor_command_;

  std::map<std::string, const CommandHandle *> enabled_;
};

class PoseHandle
{
public:
  PoseHandle() : pose_(0) {}
  PoseHandle(QuadrotorInterface *interface) : pose_(interface->getPose()) {}
  PoseHandle(const Pose *pose) : pose_(pose) {}

  const Pose& pose() const { return *pose_; }

  void getEulerRPY(double &roll, double &pitch, double &yaw) const;
  double getYaw() const;

protected:
  const Pose *pose_;
};

class TwistHandle
{
public:
  TwistHandle() : twist_(0) {}
  TwistHandle(QuadrotorInterface *interface) : twist_(interface->getTwistCommand()) {}
  TwistHandle(const Twist *twist) : twist_(twist) {}

  const Twist& twist() const { return *twist_; }

protected:
  const Twist *twist_;
};

class StateHandle : public PoseHandle, public TwistHandle
{
public:
  StateHandle() {}
  StateHandle(QuadrotorInterface *interface) : PoseHandle(interface), TwistHandle(interface) {}
  StateHandle(const Pose *pose, const Twist *twist) : PoseHandle(pose), TwistHandle(twist) {}
};

class CommandHandle
{
public:
  CommandHandle() : interface_(0) {}
  CommandHandle(QuadrotorInterface *interface) : interface_(interface) {}
  virtual ~CommandHandle() {
  }

  virtual std::string getName() const = 0;
  virtual bool available() const = 0;

  bool enabled() { return interface_->enabled(this); }
  bool start()   { return interface_->start(this); }
  void stop()    { interface_->stop(this); }

private:
  QuadrotorInterface *interface_;
};

class PoseCommandHandle : public PoseHandle, public CommandHandle
{
public:
  PoseCommandHandle() : command_(0) {}
  PoseCommandHandle(QuadrotorInterface *interface) : PoseHandle(interface), CommandHandle(interface), command_(interface->getPoseCommand()) {}
  PoseCommandHandle(const PoseHandle &pose, Pose* command) : PoseHandle(pose), command_(command) {}

  std::string getName() const { return "pose"; }
  bool available() const { return command_ != 0; }

  const Pose& getCommand() const { return *command_; }
  void setCommand(const Pose& command) { *command_ = command; }

  bool update(Pose& command) const {
    if (!available()) return false;
    command = *command_;
    return true;
  }

protected:
  Pose *command_;
};

class HorizontalPositionCommandHandle : public PoseCommandHandle
{
public:
  HorizontalPositionCommandHandle() : command_(0) {}
  HorizontalPositionCommandHandle(const PoseCommandHandle& other) : PoseCommandHandle(other), command_(&(PoseCommandHandle::command_->position)) {}
  HorizontalPositionCommandHandle(QuadrotorInterface *interface) : PoseCommandHandle(interface), command_(&(PoseCommandHandle::command_->position)) {}
  HorizontalPositionCommandHandle(const PoseHandle &pose, Point* command) : PoseCommandHandle(pose, 0), command_(command) {}

  std::string getName() const { return "pose.position.xy"; }
  bool available() const { return command_ != 0; }

  const Point& getCommand() const { return *command_; }
  void getCommand(double &x, double &y) const {
    x = command_->x;
    y = command_->y;
  }
  void setCommand(double x, double y)
  {
    command_->x = x;
    command_->y = y;
  }

  bool update(Pose& command) const {
    if (!available()) return false;
    getCommand(command.position.x, command.position.y);
    return true;
  }

  void getError(double &x, double &y) const;

protected:
  Point *command_;
};

class HeightCommandHandle : public PoseCommandHandle
{
public:
  HeightCommandHandle() : command_(0) {}
  HeightCommandHandle(const PoseCommandHandle& other) : PoseCommandHandle(other), command_(&(PoseCommandHandle::command_->position.z)) {}
  HeightCommandHandle(QuadrotorInterface *interface) : PoseCommandHandle(interface), command_(&(PoseCommandHandle::command_->position.z)) {}
  HeightCommandHandle(const PoseHandle &pose, double *command) : PoseCommandHandle(pose, 0), command_(command) {}

  std::string getName() const { return "pose.position.z"; }
  bool available() const { return command_ != 0; }

  double getCommand() const { return *command_; }
  void setCommand(double command) { *command_ = command; }

  bool update(Pose& command) const {
    if (!available()) return false;
    command.position.z = getCommand();
    return true;
  }

  double getError() const;

protected:
  double *command_;
};

class HeadingCommandHandle : public PoseCommandHandle
{
public:
  HeadingCommandHandle() : command_(0) {}
  HeadingCommandHandle(const PoseCommandHandle& other) : PoseCommandHandle(other), command_(0), quaternion_(&(PoseCommandHandle::command_->orientation)) {}
  HeadingCommandHandle(QuadrotorInterface *interface) : PoseCommandHandle(interface), command_(0), quaternion_(&(PoseCommandHandle::command_->orientation)) {}
  HeadingCommandHandle(const PoseHandle &pose, double *command) : PoseCommandHandle(pose, 0), command_(command), quaternion_(0) {}
  HeadingCommandHandle(const PoseHandle &pose, Quaternion *quaternion) : PoseCommandHandle(pose, 0), command_(0), quaternion_(quaternion) {}

  std::string getName() const { return "pose.orientation.yaw"; }
  bool available() const { return (command_ != 0) || (quaternion_ != 0); }

  double getCommand() const;
  void setCommand(double command);

  bool update(Pose& command) const;
  double getError() const;

protected:
  double *command_;
  Quaternion *quaternion_;
};

class TwistCommandHandle : public TwistHandle, public CommandHandle
{
public:
  TwistCommandHandle() : command_(0) {}
  TwistCommandHandle(QuadrotorInterface *interface) : TwistHandle(interface), CommandHandle(interface), command_(interface->getTwistCommand()) {}
  TwistCommandHandle(const TwistHandle &twist, Twist* command) : TwistHandle(twist), command_(command) {}

  std::string getName() const { return "twist"; }
  bool available() const { return command_ != 0; }

  const Twist& getCommand() const { return *command_; }
  void setCommand(const Twist& command) { *command_ = command; }

  bool update(Twist& command) const {
    if (!available()) return false;
    command = *command_;
    return true;
  }

protected:
  Twist *command_;
};

class HorizontalVelocityCommandHandle : public TwistCommandHandle
{
public:
  HorizontalVelocityCommandHandle() : command_(0) {}
  HorizontalVelocityCommandHandle(const TwistCommandHandle& other) : TwistCommandHandle(other), command_(&(TwistCommandHandle::command_->linear)) {}
  HorizontalVelocityCommandHandle(QuadrotorInterface *interface) : TwistCommandHandle(interface), command_(&(TwistCommandHandle::command_->linear)) {}
  HorizontalVelocityCommandHandle(const TwistHandle &twist, Vector3* command) : TwistCommandHandle(twist, 0), command_(command) {}

  std::string getName() const { return "twist.position.xy"; }
  bool available() const { return command_ != 0; }

  const Vector3& getCommand() const { return *command_; }
  void getCommand(double &x, double &y) const {
    x = command_->x;
    y = command_->y;
  }
  void setCommand(double x, double y)
  {
    command_->x = x;
    command_->y = y;
  }

  bool update(Twist& command) const {
    if (!available()) return false;
    getCommand(command.linear.x, command.linear.y);
    return true;
  }

protected:
  Vector3 *command_;
};

class VerticalVelocityCommandHandle : public TwistCommandHandle
{
public:
  VerticalVelocityCommandHandle() : command_(0) {}
  VerticalVelocityCommandHandle(const TwistCommandHandle& other) : TwistCommandHandle(other), command_(&(TwistCommandHandle::command_->linear.z)) {}
  VerticalVelocityCommandHandle(QuadrotorInterface *interface) : TwistCommandHandle(interface), command_(&(TwistCommandHandle::command_->linear.z)) {}
  VerticalVelocityCommandHandle(const TwistHandle &twist, double* command) : TwistCommandHandle(twist, 0), command_(command) {}

  std::string getName() const { return "twist.linear.z"; }
  bool available() const { return command_ != 0; }

  double getCommand() const { return *command_; }
  void setCommand(double command) { *command_ = command; }

  bool update(Twist& command) const {
    if (!available()) return false;
    command.linear.z = getCommand();
    return true;
  }

protected:
  double *command_;
};

class AngularVelocityCommandHandle : public TwistCommandHandle
{
public:
  AngularVelocityCommandHandle() : command_(0) {}
  AngularVelocityCommandHandle(const TwistCommandHandle& other) : TwistCommandHandle(other), command_(&(TwistCommandHandle::command_->angular.z)) {}
  AngularVelocityCommandHandle(QuadrotorInterface *interface) : TwistCommandHandle(interface), command_(&(TwistCommandHandle::command_->angular.z)) {}
  AngularVelocityCommandHandle(const TwistHandle &twist, double* command) : TwistCommandHandle(twist, 0), command_(command) {}

  std::string getName() const { return "twist.angular.z"; }
  bool available() const { return command_ != 0; }

  double getCommand() const { return *command_; }
  void setCommand(double command) { *command_ = command; }

  bool update(Twist& command) const {
    if (!available()) return false;
    command.linear.z = getCommand();
    return true;
  }

protected:
  double *command_;
};

class WrenchCommandHandle : public CommandHandle
{
public:
  WrenchCommandHandle() : command_(0) {}
  WrenchCommandHandle(QuadrotorInterface *interface) : CommandHandle(interface), command_(interface->getWrenchCommand()) {}

  std::string getName() const { return "wrench"; }
  bool available() const { return command_ != 0; }

  void setCommand(const Wrench& command) { *command_ = command; }
  const Wrench& getCommand() const { return *command_; }

  bool update(Wrench& command) const {
    if (!available()) return false;
    command = *command_;
    return true;
  }

protected:
  Wrench *command_;
};

class MotorCommandHandle : public CommandHandle
{
public:
  MotorCommandHandle() : command_(0) {}
  MotorCommandHandle(QuadrotorInterface *interface) : CommandHandle(interface), command_(interface->getMotorCommand()) {}

  std::string getName() const { return "motor"; }
  bool available() const { return command_ != 0; }

  void setCommand(const MotorCommand& command) { *command_ = command; }
  const MotorCommand& getCommand() const { return *command_; }

  bool update(MotorCommand& command) const {
    if (!available()) return false;
    command = *command_;
    return true;
  }

protected:
  MotorCommand *command_;
};

} // namespace hector_quadrotor_controller

#endif // HECTOR_QUADROTOR_CONTROLLER_QUADROTOR_INTERFACE_H
