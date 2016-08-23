//=================================================================================================
// Copyright (c) 2012-2016, Institute of Flight Systems and Automatic Control,
// Technische Universität Darmstadt.
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of hector_quadrotor nor the names of its contributors
//       may be used to endorse or promote products derived from this software
//       without specific prior written permission.

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

#ifndef HECTOR_QUADROTOR_INTERFACE_HANDLES_H
#define HECTOR_QUADROTOR_INTERFACE_HANDLES_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Wrench.h>
#include <sensor_msgs/Imu.h>
#include <hector_uav_msgs/MotorStatus.h>
#include <hector_uav_msgs/MotorCommand.h>
#include <hector_uav_msgs/AttitudeCommand.h>
#include <hector_uav_msgs/YawrateCommand.h>
#include <hector_uav_msgs/ThrustCommand.h>

namespace hector_quadrotor_interface {

class QuadrotorInterface;

using geometry_msgs::Pose;
using geometry_msgs::Point;
using geometry_msgs::Quaternion;
using geometry_msgs::Accel;
using geometry_msgs::Twist;
using geometry_msgs::Vector3;
using geometry_msgs::Wrench;
using sensor_msgs::Imu;
using hector_uav_msgs::MotorStatus;
using hector_uav_msgs::MotorCommand;
using hector_uav_msgs::AttitudeCommand;
using hector_uav_msgs::YawrateCommand;
using hector_uav_msgs::ThrustCommand;

template <class Derived, typename T>
class Handle_
{
public:
  typedef T ValueType;
  typedef Handle_<Derived, T> Base;

  Handle_(const std::string& name, const std::string& field = std::string()) : interface_(0), name_(name), field_(field), value_(0) {}
  Handle_(QuadrotorInterface *interface, const std::string& name, const std::string& field = std::string()) : interface_(interface), name_(name), field_(field), value_(0) {}
  Handle_(QuadrotorInterface *interface, const ValueType *source, const std::string& name, const std::string& field = std::string()) : interface_(interface), name_(name), field_(field) { *this = source; }
  virtual ~Handle_() {}

  virtual const std::string& getName() const { return name_; }
  virtual const std::string& getField() const { return field_; }

  virtual bool connected() const { return get(); }
  virtual void reset() { value_ = 0; }

  Derived& operator=(const ValueType *source) { value_ = source; return static_cast<Derived &>(*this); }
  const ValueType *get() const { return value_; }
  const ValueType &operator*() const { return *value_; }

protected:
  QuadrotorInterface *interface_;
  const std::string name_;
  const std::string field_;
  const ValueType *value_;
};

class PoseHandle : public Handle_<PoseHandle, Pose>
{
public:
  using Base::operator=;

  PoseHandle() : Base("pose") {}
  PoseHandle(QuadrotorInterface *interface) : Base(interface, "pose") {}
  PoseHandle(QuadrotorInterface *interface, const Pose *pose) : Base(interface, pose, "pose") {}
  virtual ~PoseHandle() {}

  const ValueType& pose() const { return *get(); }

  void getEulerRPY(double &roll, double &pitch, double &yaw) const;
  double getYaw() const;
  Vector3 toBody(const Vector3& nav) const;
  Vector3 fromBody(const Vector3& nav) const;
};
typedef boost::shared_ptr<PoseHandle> PoseHandlePtr;

class TwistHandle : public Handle_<TwistHandle, Twist>
{
public:
  using Base::operator=;

  TwistHandle() : Base("twist") {}
  TwistHandle(QuadrotorInterface *interface) : Base(interface, "twist") {}
  TwistHandle(QuadrotorInterface *interface, const Twist *twist) : Base(interface, twist, "twist") {}
  virtual ~TwistHandle() {}

  const ValueType& twist() const { return *get(); }
};
typedef boost::shared_ptr<TwistHandle> TwistHandlePtr;

class AccelerationHandle : public Handle_<AccelerationHandle, Accel>
{
public:
  using Base::operator=;

  AccelerationHandle() : Base("acceleration") {}
  AccelerationHandle(QuadrotorInterface *interface) : Base(interface, "acceleration") {}
  AccelerationHandle(QuadrotorInterface *interface, const Accel *acceleration) : Base(interface, acceleration, "acceleration") {}
  virtual ~AccelerationHandle() {}

  const ValueType& acceleration() const { return *get(); }
};
typedef boost::shared_ptr<AccelerationHandle> AccelerationHandlePtr;

class StateHandle : public PoseHandle, public TwistHandle
{
public:
  StateHandle() {}
  StateHandle(QuadrotorInterface *interface, const Pose *pose, const Twist *twist) : PoseHandle(interface, pose), TwistHandle(interface, twist) {}
  virtual ~StateHandle() {}

  virtual bool connected() const { return PoseHandle::connected() && TwistHandle::connected(); }
};
typedef boost::shared_ptr<StateHandle> StateHandlePtr;

class ImuHandle : public Handle_<ImuHandle, Imu>
{
public:
  using Base::operator=;

  ImuHandle() : Base("imu") {}
  ImuHandle(QuadrotorInterface *interface, const Imu *imu) : Base(interface, imu, "imu") {}
  virtual ~ImuHandle() {}

  const ValueType& imu() const { return *get(); }
};
typedef boost::shared_ptr<ImuHandle> ImuHandlePtr;

class MotorStatusHandle : public Handle_<MotorStatusHandle, MotorStatus>
{
public:
  using Base::operator=;

  MotorStatusHandle() : Base("motor_status") {}
  MotorStatusHandle(QuadrotorInterface *interface, const MotorStatus *motor_status) : Base(interface, motor_status, "motor_status") {}
  virtual ~MotorStatusHandle() {}

  const ValueType& motorStatus() const { return *get(); }
};
typedef boost::shared_ptr<MotorStatusHandle> MotorStatusHandlePtr;

class CommandHandle
{
public:
  CommandHandle() : interface_(0), preempted_(false), new_value_(false) {}
  CommandHandle(QuadrotorInterface *interface, const std::string& name, const std::string& field) : interface_(interface), name_(name), field_(field), preempted_(false), new_value_(false) {}
  virtual ~CommandHandle() {}

  virtual const std::string& getName() const { return name_; }
  virtual const std::string& getField() const { return field_; }
  virtual bool connected() const = 0;
  virtual void reset() {}

  void *get() const { return 0; }

  bool enabled();
  bool start();
  void stop();
  void disconnect();

  bool preempt();
  void setPreempted();
  bool preempted();

  template <typename T> T* ownData(T* data) { my_.reset(data); return data; }

  template <typename Derived> bool connectFrom(const Derived& output) {
    Derived *me = dynamic_cast<Derived *>(this);
    if (!me) return false;
    ROS_DEBUG("Connected output port '%s (%p)' to input port '%s (%p)'", output.getName().c_str(), &output, me->getName().c_str(), me);
    return (*me = output.get()).connected();
  }

  template <typename Derived> bool connectTo(Derived& input) const {
    const Derived *me = dynamic_cast<const Derived *>(this);
    if (!me) return false;
    ROS_DEBUG("Connected output port '%s (%p)' to input port '%s (%p)'", me->getName().c_str(), me, input.getName().c_str(), &input);
    return (input = me->get()).connected();
  }

private:
  QuadrotorInterface *interface_;
  const std::string name_;
  const std::string field_;
  boost::shared_ptr<void> my_;

  bool preempted_;

protected:
  mutable bool new_value_;
  bool wasNew() const { bool old = new_value_; new_value_ = false; return old; }
};
typedef boost::shared_ptr<CommandHandle> CommandHandlePtr;

namespace internal {
  template <class Derived> struct FieldAccessor {
    static typename Derived::ValueType *get(void *) { return 0; }
  };
}

template <class Derived, typename T, class Parent = CommandHandle>
class CommandHandle_ : public Parent
{
public:
  typedef T ValueType;
  typedef CommandHandle_<Derived, T, Parent> Base;

  CommandHandle_() : command_(0) {}
  CommandHandle_(const Parent &other) : Parent(other), command_(0) {}
  CommandHandle_(QuadrotorInterface *interface, const std::string& name, const std::string& field = std::string()) : Parent(interface, name, field), command_(0) {}
  virtual ~CommandHandle_() {}

  virtual bool connected() const { return get(); }
  virtual void reset() { command_ = 0; Parent::reset(); }

  using Parent::operator=;
  Derived& operator=(ValueType *source) { command_ = source; return static_cast<Derived &>(*this); }
  ValueType *get() const { return command_ ? command_ : internal::FieldAccessor<Derived>::get(Parent::get()); }
  ValueType &operator*() const { return *command_; }

  ValueType& command() { return *get(); }
  const ValueType& getCommand() const { this->new_value_ = false; return *get(); }
  void setCommand(const ValueType& command) { this->new_value_ = true; *get() = command; }
  bool getCommand(ValueType& command) const { command = *getCommand(); return this->wasNew(); }

  bool update(ValueType& command) const {
    if (!connected()) return false;
    command = getCommand();
    return true;
  }

protected:
  ValueType *command_;
};

class PoseCommandHandle : public CommandHandle_<PoseCommandHandle, Pose>
{
public:
  using Base::operator=;

  PoseCommandHandle() {}
  PoseCommandHandle(QuadrotorInterface *interface, const std::string& name, const std::string& field = std::string()) : Base(interface, name, field) {}
  PoseCommandHandle(Pose* command) { *this = command; }
  virtual ~PoseCommandHandle() {}
};
typedef boost::shared_ptr<PoseCommandHandle> PoseCommandHandlePtr;

class HorizontalPositionCommandHandle : public CommandHandle_<HorizontalPositionCommandHandle, Point, PoseCommandHandle>
{
public:
  using Base::operator=;

  HorizontalPositionCommandHandle() {}
  HorizontalPositionCommandHandle(const PoseCommandHandle& other) : Base(other) {}
  HorizontalPositionCommandHandle(QuadrotorInterface *interface, const std::string& name) : Base(interface, name, "position.xy") {}
  HorizontalPositionCommandHandle(Point* command) { *this = command; }
  virtual ~HorizontalPositionCommandHandle() {}

  using Base::getCommand;
  virtual bool getCommand(double &x, double &y) const {
    x = get()->x;
    y = get()->y;
    return wasNew();
  }

  using Base::setCommand;
  virtual void setCommand(double x, double y)
  {
    this->new_value_ = true;
    get()->x = x;
    get()->y = y;
  }

  using Base::update;
  bool update(Pose& command) const {
    if (!connected()) return false;
    getCommand(command.position.x, command.position.y);
    return true;
  }

  void getError(const PoseHandle &pose, double &x, double &y) const;
};
typedef boost::shared_ptr<HorizontalPositionCommandHandle> HorizontalPositionCommandHandlePtr;

namespace internal {
  template <> struct FieldAccessor<HorizontalPositionCommandHandle> {
    static Point *get(Pose *pose) { return &(pose->position); }
  };
}

class HeightCommandHandle : public CommandHandle_<HeightCommandHandle, double, PoseCommandHandle>
{
public:
  using Base::operator=;

  HeightCommandHandle() {}
  HeightCommandHandle(const PoseCommandHandle& other) : Base(other) {}
  HeightCommandHandle(QuadrotorInterface *interface, const std::string& name) : Base(interface, name, "position.z") {}
  HeightCommandHandle(double *command) { *this = command; }
  virtual ~HeightCommandHandle() {}

  using Base::update;
  bool update(Pose& command) const {
    if (!connected()) return false;
    command.position.z = getCommand();
    return true;
  }

  double getError(const PoseHandle &pose) const;
};
typedef boost::shared_ptr<HeightCommandHandle> HeightCommandHandlePtr;

namespace internal {
  template <> struct FieldAccessor<HeightCommandHandle> {
    static double *get(Pose *pose) { return &(pose->position.z); }
  };
}

class HeadingCommandHandle : public CommandHandle_<HeadingCommandHandle, Quaternion, PoseCommandHandle>
{
public:
  using Base::operator=;

  HeadingCommandHandle() {}
  HeadingCommandHandle(const PoseCommandHandle& other) : Base(other), scalar_(0) {}
  HeadingCommandHandle(QuadrotorInterface *interface, const std::string& name) : Base(interface, name, "orientation.yaw"), scalar_(0) {}
  HeadingCommandHandle(Quaternion *command) { *this = command; }
  virtual ~HeadingCommandHandle() {}

  using Base::getCommand;
  double getCommand() const;

  using Base::setCommand;
  void setCommand(double command);

  using Base::update;
  bool update(Pose& command) const;

  double getError(const PoseHandle &pose) const;

protected:
  double *scalar_;
};
typedef boost::shared_ptr<HeadingCommandHandle> HeadingCommandHandlePtr;

namespace internal {
  template <> struct FieldAccessor<HeadingCommandHandle> {
    static Quaternion *get(Pose *pose) { return &(pose->orientation); }
  };
}

class TwistCommandHandle : public CommandHandle_<TwistCommandHandle, Twist>
{
public:

  using Base::operator=;

  TwistCommandHandle() {}
  TwistCommandHandle(QuadrotorInterface *interface, const std::string& name, const std::string& field = std::string()) : Base(interface, name, field) {}
  TwistCommandHandle(Twist* command) { *this = command; }
  virtual ~TwistCommandHandle() {}
};
typedef boost::shared_ptr<TwistCommandHandle> TwistCommandHandlePtr;

class AccelCommandHandle : public CommandHandle_<AccelCommandHandle, Accel>
{
public:

  using Base::operator=;

  AccelCommandHandle() {}
  AccelCommandHandle(QuadrotorInterface *interface, const std::string& name, const std::string& field = std::string()) : Base(interface, name, field) {}
  AccelCommandHandle(Accel* command) { *this = command; }
  virtual ~AccelCommandHandle() {}
};
typedef boost::shared_ptr<AccelCommandHandle> AccelCommandHandlePtr;

class HorizontalVelocityCommandHandle : public CommandHandle_<HorizontalVelocityCommandHandle, Vector3, TwistCommandHandle>
{
public:
  using Base::operator=;

  HorizontalVelocityCommandHandle() {}
  HorizontalVelocityCommandHandle(const TwistCommandHandle& other) : Base(other) {}
  HorizontalVelocityCommandHandle(QuadrotorInterface *interface, const std::string& name) : Base(interface, name, "linear.xy") {}
  HorizontalVelocityCommandHandle(Vector3* command) { *this = command; }
  virtual ~HorizontalVelocityCommandHandle() {}

  using Base::getCommand;
  bool getCommand(double &x, double &y) const {
    x = get()->x;
    y = get()->y;
    return true;
  }

  using Base::setCommand;
  void setCommand(double x, double y)
  {
    get()->x = x;
    get()->y = y;
  }

  using Base::update;
  bool update(Twist& command) const {
    if (!connected()) return false;
    getCommand(command.linear.x, command.linear.y);
    return true;
  }
};
typedef boost::shared_ptr<HorizontalVelocityCommandHandle> HorizontalVelocityCommandHandlePtr;

namespace internal {
  template <> struct FieldAccessor<HorizontalVelocityCommandHandle> {
    static Vector3 *get(Twist *twist) { return &(twist->linear); }
  };
}

class VerticalVelocityCommandHandle : public CommandHandle_<VerticalVelocityCommandHandle, double, TwistCommandHandle>
{
public:
  using Base::operator=;

  VerticalVelocityCommandHandle() {}
  VerticalVelocityCommandHandle(const TwistCommandHandle& other) : Base(other) {}
  VerticalVelocityCommandHandle(QuadrotorInterface *interface, const std::string& name) : Base(interface, name, "linear.z") {}
  VerticalVelocityCommandHandle(double* command) { *this = command; }
  virtual ~VerticalVelocityCommandHandle() {}

  using Base::update;
  bool update(Twist& command) const {
    if (!connected()) return false;
    command.linear.z = *get();
    return true;
  }
};
typedef boost::shared_ptr<VerticalVelocityCommandHandle> VerticalVelocityCommandHandlePtr;

namespace internal {
  template <> struct FieldAccessor<VerticalVelocityCommandHandle> {
    static double *get(Twist *twist) { return &(twist->linear.z); }
  };
}

class AngularVelocityCommandHandle : public CommandHandle_<AngularVelocityCommandHandle, double, TwistCommandHandle>
{
public:
  using Base::operator=;

  AngularVelocityCommandHandle() {}
  AngularVelocityCommandHandle(const TwistCommandHandle& other) : Base(other) {}
  AngularVelocityCommandHandle(QuadrotorInterface *interface, const std::string& name) : Base(interface, name, "angular.z") {}
  AngularVelocityCommandHandle(double* command) { *this = command; }
  virtual ~AngularVelocityCommandHandle() {}

  using Base::update;
  bool update(Twist& command) const {
    if (!connected()) return false;
    command.linear.z = *get();
    return true;
  }
};
typedef boost::shared_ptr<AngularVelocityCommandHandle> AngularVelocityCommandHandlePtr;

namespace internal {
  template <> struct FieldAccessor<AngularVelocityCommandHandle> {
    static double *get(Twist *twist) { return &(twist->angular.z); }
  };
}

class WrenchCommandHandle : public CommandHandle_<WrenchCommandHandle, Wrench>
{
public:
  using Base::operator=;

  WrenchCommandHandle() {}
  WrenchCommandHandle(QuadrotorInterface *interface, const std::string& name) : Base(interface, name) {}
  virtual ~WrenchCommandHandle() {}
};
typedef boost::shared_ptr<WrenchCommandHandle> WrenchCommandHandlePtr;

class MotorCommandHandle : public CommandHandle_<MotorCommandHandle, MotorCommand>
{
public:
  using Base::operator=;

  MotorCommandHandle() {}
  MotorCommandHandle(QuadrotorInterface *interface, const std::string& name) : Base(interface, name) {}
  virtual ~MotorCommandHandle() {}
};
typedef boost::shared_ptr<MotorCommandHandle> MotorCommandHandlePtr;

class AttitudeCommandHandle : public CommandHandle_<AttitudeCommandHandle, AttitudeCommand>
{
public:
  using Base::operator=;

  AttitudeCommandHandle() {}
  AttitudeCommandHandle(QuadrotorInterface *interface, const std::string& name) : Base(interface, name) {}
  virtual ~AttitudeCommandHandle() {}
};
typedef boost::shared_ptr<AttitudeCommandHandle> AttitudeCommandHandlePtr;

class YawrateCommandHandle : public CommandHandle_<YawrateCommandHandle, YawrateCommand>
{
public:
  using Base::operator=;

  YawrateCommandHandle() {}
  YawrateCommandHandle(QuadrotorInterface *interface, const std::string& name) : Base(interface, name) {}
  virtual ~YawrateCommandHandle() {}
};
typedef boost::shared_ptr<YawrateCommandHandle> YawrateCommandHandlePtr;

class ThrustCommandHandle : public CommandHandle_<ThrustCommandHandle, ThrustCommand>
{
public:
  using Base::operator=;

  ThrustCommandHandle() {}
  ThrustCommandHandle(QuadrotorInterface *interface, const std::string& name) : Base(interface, name) {}
  virtual ~ThrustCommandHandle() {}
};
typedef boost::shared_ptr<ThrustCommandHandle> ThrustCommandHandlePtr;

} // namespace hector_quadrotor_interface

#endif // HECTOR_QUADROTOR_INTERFACE_HANDLES_H
