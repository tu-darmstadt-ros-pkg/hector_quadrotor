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

#include <map>
#include <list>

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

  virtual Pose *getPose()                 { return 0; }
  virtual Twist *getTwist()               { return 0; }
  virtual Imu *getSensorImu()             { return 0; }
  virtual MotorStatus *getMotorStatus()   { return 0; }

  template <typename HandleType> boost::shared_ptr<HandleType> addInput(const std::string& controller = std::string())
  {
    boost::shared_ptr<HandleType> input(new HandleType(this, controller));
    inputs_[&typeid(HandleType)].push_back(input);

    // connect to primary output
    if (outputs_.count(&typeid(HandleType))) {
      HandleList& outputs = outputs_.at(&typeid(HandleType));
      if (!outputs.empty()) {
          HandleType& output = *boost::shared_static_cast<HandleType>(outputs.front());
          if (output.getController().empty() || (output.getController() != input->getController())) {
              output.connectTo(*input);
          }
      }
    }

    return input;
  }

  template <typename HandleType> boost::shared_ptr<HandleType> addOutput(const std::string& controller = std::string())
  {
    boost::shared_ptr<HandleType> output(new HandleType(this, controller));
    outputs_[&typeid(HandleType)].push_front(output);

    *output = output->ownData(new typename HandleType::ValueType());

    // connect matching unconnected inputs
    if (inputs_.count(&typeid(HandleType))) {
      HandleList& inputs = inputs_.at(&typeid(HandleType));
      for(HandleList::iterator it = inputs.begin(); it != inputs.end(); ++it) {
        HandleType& input = *boost::shared_static_cast<HandleType>(*it);
        if (!input.getController().empty() && (input.getController() == output->getController())) continue;
        if (!input.connected()) input.connectFrom(*output);
      }
    }

    return output;
  }

  template <typename HandleType> boost::shared_ptr<HandleType> getOutput() const
  {
    if (!outputs_.count(&typeid(HandleType))) return boost::shared_ptr<HandleType>();
    return boost::shared_static_cast<HandleType>(outputs_.at(&typeid(HandleType)).front());
  }

  template <typename HandleType> boost::shared_ptr<HandleType> getInput() const
  {
    if (!inputs_.count(&typeid(HandleType))) return boost::shared_ptr<HandleType>();
    return boost::shared_static_cast<HandleType>(inputs_[&typeid(HandleType)].front());
  }

  template <typename HandleType> typename HandleType::ValueType const* getCommand() const
  {
    boost::shared_ptr<HandleType> output = getOutput<HandleType>();
    if (!output || !output->connected()) return 0;
    return &(output->command());
  }

  virtual const Pose *getPoseCommand() const;
  virtual const Twist *getTwistCommand() const;
  virtual const Wrench *getWrenchCommand() const;
  virtual const MotorCommand *getMotorCommand() const;

  bool enabled(const CommandHandle *handle) const;
  bool start(const CommandHandle *handle);
  void stop(const CommandHandle *handle);

private:
  typedef std::list< boost::shared_ptr<CommandHandle> > HandleList;
  std::map<const std::type_info *, HandleList> inputs_;
  std::map<const std::type_info *, HandleList> outputs_;
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
  TwistHandle(QuadrotorInterface *interface) : twist_(interface->getTwist()) {}
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
  CommandHandle(QuadrotorInterface *interface, const std::string& controller) : interface_(interface), controller_(controller) {}
  virtual ~CommandHandle() {}

  virtual std::string getName() const = 0;
  virtual bool connected() const = 0;

  bool enabled() { return interface_->enabled(this); }
  bool start()   { return interface_->start(this); }
  void stop()    { interface_->stop(this); }

  template <typename T> T* ownData(T* data) { my_.reset(data); return data; }

  template <typename Derived> bool connectFrom(const Derived& output) {
    Derived *me = dynamic_cast<Derived *>(this);
    if (!me) return false;
    ROS_DEBUG("Connected output port '%s/%s (%p)' to input port '%s/%s (%p)'", output.controller_.c_str(), output.getName().c_str(), &output, me->controller_.c_str(), me->getName().c_str(), me);
    return (*me = output.get()).connected();
  }

  template <typename Derived> bool connectTo(Derived& input) const {
    const Derived *me = dynamic_cast<const Derived *>(this);
    if (!me) return false;
    ROS_DEBUG("Connected output port '%s/%s (%p)' to input port '%s/%s (%p)'", me->controller_.c_str(), me->getName().c_str(), me, input.controller_.c_str(), input.getName().c_str(), &input);
    return (input = me->get()).connected();
  }

  const std::string& getController() const { return controller_; }

private:
  QuadrotorInterface *interface_;
  std::string controller_;
  boost::shared_ptr<void> my_;
};

class PoseCommandHandle : public PoseHandle, public CommandHandle
{
public:
  typedef Pose ValueType;

  PoseCommandHandle() : command_(0) {}
  PoseCommandHandle(QuadrotorInterface *interface, const std::string& controller) : PoseHandle(interface), CommandHandle(interface, controller), command_(0) {}
  PoseCommandHandle(const PoseHandle &pose, Pose* command) : PoseHandle(pose) { *this = command; }

  std::string getName() const { return "pose"; }
  bool connected() const { return get(); }

  using CommandHandle::operator=;
  PoseCommandHandle& operator=(Pose *source) { command_ = source; return *this; }
  ValueType *get() const { return command_; }

  ValueType& command() { return *get(); }
  const ValueType& getCommand() const { return *get(); }
  void setCommand(const ValueType& command) { *get() = command; }

  bool update(Pose& command) const {
    if (!connected()) return false;
    command = *command_;
    return true;
  }

protected:
  ValueType *command_;
};

class HorizontalPositionCommandHandle : public PoseCommandHandle
{
public:
  typedef Point ValueType;

  HorizontalPositionCommandHandle() : command_(0) {}
  HorizontalPositionCommandHandle(const PoseCommandHandle& other) : PoseCommandHandle(other), command_(0) {}
  HorizontalPositionCommandHandle(QuadrotorInterface *interface, const std::string& controller) : PoseCommandHandle(interface, controller), command_(0) {}
  HorizontalPositionCommandHandle(const PoseHandle &pose, Point* command) : PoseCommandHandle(pose, 0) { *this = command; }

  std::string getName() const { return "pose.position.xy"; }
  bool connected() const { return get(); }

  HorizontalPositionCommandHandle& operator=(Pose *source) { PoseCommandHandle::operator=(source); command_ = 0; return *this; }
  HorizontalPositionCommandHandle& operator=(Point *source) { command_ = source; return *this; }
  ValueType *get() const { return command_ ? command_ : (PoseCommandHandle::get() ? &(PoseCommandHandle::get()->position) : 0); }

  ValueType& command() { return *get(); }
  const ValueType& getCommand() const { return *get(); }
  void getCommand(double &x, double &y) const {
    x = get()->x;
    y = get()->y;
  }
  void setCommand(double x, double y)
  {
    get()->x = x;
    get()->y = y;
  }

  bool update(Pose& command) const {
    if (!connected()) return false;
    getCommand(command.position.x, command.position.y);
    return true;
  }

  void getError(double &x, double &y) const;

protected:
  ValueType *command_;
};

class HeightCommandHandle : public PoseCommandHandle
{
public:
  typedef double ValueType;

  HeightCommandHandle() : command_(0) {}
  HeightCommandHandle(const PoseCommandHandle& other) : PoseCommandHandle(other), command_(0) {}
  HeightCommandHandle(QuadrotorInterface *interface, const std::string& controller) : PoseCommandHandle(interface, controller), command_(0) {}
  HeightCommandHandle(const PoseHandle &pose, double *command) : PoseCommandHandle(pose, 0) { *this = command; }

  std::string getName() const { return "pose.position.z"; }
  bool connected() const { return get(); }

  HeightCommandHandle& operator=(Pose *source) { PoseCommandHandle::operator=(source); command_ = 0; return *this; }
  HeightCommandHandle& operator=(double *source) { command_ = source; return *this; }
  ValueType *get() const { return command_ ? command_ : (PoseCommandHandle::get() ? &(PoseCommandHandle::get()->position.z) : 0); }

  ValueType& command() { return *get(); }
  double getCommand() const { return *get(); }
  void setCommand(double command) { *get() = command; }

  bool update(Pose& command) const {
    if (!connected()) return false;
    command.position.z = getCommand();
    return true;
  }

  double getError() const;

protected:
  ValueType *command_;
};

class HeadingCommandHandle : public PoseCommandHandle
{
public:
  typedef Quaternion ValueType;

  HeadingCommandHandle() : command_(0) {}
  HeadingCommandHandle(const PoseCommandHandle& other) : PoseCommandHandle(other), command_(0), scalar_(0) {}
  HeadingCommandHandle(QuadrotorInterface *interface, const std::string& controller) : PoseCommandHandle(interface, controller), command_(0), scalar_(0) {}
  // HeadingCommandHandle(const PoseHandle &pose, double *command) : PoseCommandHandle(pose, 0) { *this = value_; }
  HeadingCommandHandle(const PoseHandle &pose, Quaternion *quaternion) : PoseCommandHandle(pose, 0) { *this = command_; }

  std::string getName() const { return "pose.orientation.yaw"; }
  bool connected() const { return (command_ != 0) || get(); }

  HeadingCommandHandle& operator=(Pose *source) { PoseCommandHandle::operator=(source); command_ = 0; return *this; }
  // HeadingCommandHandle& operator=(double *source) { command_ = 0; scalar_ = source; return *this; }
  HeadingCommandHandle& operator=(Quaternion *source) { command_ = source; scalar_ = 0; return *this; }
  ValueType *get() const { return command_ ? command_ : (PoseCommandHandle::get() ? &(PoseCommandHandle::get()->orientation) : 0); }

  ValueType& command() { return *get(); }
  double getCommand() const;
  void setCommand(double command);

  bool update(Pose& command) const;
  double getError() const;

protected:
  ValueType *command_;
  double *scalar_;
};

class TwistCommandHandle : public TwistHandle, public CommandHandle
{
public:
  typedef Twist ValueType;

  TwistCommandHandle() : command_(0) {}
  TwistCommandHandle(QuadrotorInterface *interface, const std::string& controller) : TwistHandle(interface), CommandHandle(interface, controller), command_(0) {}
  TwistCommandHandle(const TwistHandle &twist, Twist* command) : TwistHandle(twist) { *this = command; }

  std::string getName() const { return "twist"; }
  bool connected() const { return get(); }

  TwistCommandHandle& operator=(Twist *source) { command_ = source; return *this; }
  ValueType *get() const { return command_; }

  ValueType& command() { return *get(); }
  const Twist& getCommand() const { return *get(); }
  void setCommand(const Twist& command) { *get() = command; }

  bool update(Twist& command) const {
    if (!connected()) return false;
    command = *get();
    return true;
  }

protected:
  ValueType *command_;
};

class HorizontalVelocityCommandHandle : public TwistCommandHandle
{
public:
  typedef Vector3 ValueType;

  HorizontalVelocityCommandHandle() : command_(0) {}
  HorizontalVelocityCommandHandle(const TwistCommandHandle& other) : TwistCommandHandle(other), command_(0) {}
  HorizontalVelocityCommandHandle(QuadrotorInterface *interface, const std::string& controller) : TwistCommandHandle(interface, controller), command_(0) {}
  HorizontalVelocityCommandHandle(const TwistHandle &twist, Vector3* command) : TwistCommandHandle(twist, 0) { *this = command; }

  std::string getName() const { return "twist.position.xy"; }
  bool connected() const { return get(); }

  HorizontalVelocityCommandHandle& operator=(Twist *source) { TwistCommandHandle::operator=(source); command_ = 0; return *this; }
  HorizontalVelocityCommandHandle& operator=(Vector3 *source) { command_ = source; return *this; }
  ValueType *get() const { return command_ ? command_ : (TwistCommandHandle::get() ? &(TwistCommandHandle::get()->linear) : 0); }

  ValueType& command() { return *get(); }
  const Vector3& getCommand() const { return *get(); }
  void getCommand(double &x, double &y) const {
    x = get()->x;
    y = get()->y;
  }
  void setCommand(double x, double y)
  {
    get()->x = x;
    get()->y = y;
  }

  bool update(Twist& command) const {
    if (!connected()) return false;
    getCommand(command.linear.x, command.linear.y);
    return true;
  }

protected:
  ValueType *command_;
};

class VerticalVelocityCommandHandle : public TwistCommandHandle
{
public:
  typedef double ValueType;

  VerticalVelocityCommandHandle() : command_(0) {}
  VerticalVelocityCommandHandle(const TwistCommandHandle& other) : TwistCommandHandle(other), command_(0) {}
  VerticalVelocityCommandHandle(QuadrotorInterface *interface, const std::string& controller) : TwistCommandHandle(interface, controller), command_(0) {}
  VerticalVelocityCommandHandle(const TwistHandle &twist, double* command) : TwistCommandHandle(twist, 0) { *this = command; }

  std::string getName() const { return "twist.linear.z"; }
  bool connected() const { return get(); }

  VerticalVelocityCommandHandle& operator=(Twist *source) { TwistCommandHandle::operator=(source); command_ = 0; return *this; }
  VerticalVelocityCommandHandle& operator=(double *source) { command_ = source; return *this; }
  ValueType *get() const { return command_ ? command_ : (TwistCommandHandle::get() ? &(TwistCommandHandle::get()->linear.z) : 0); }

  ValueType& command() { return *get(); }
  double getCommand() const { return *get(); }
  void setCommand(double command) { *get() = command; }

  bool update(Twist& command) const {
    if (!connected()) return false;
    command.linear.z = getCommand();
    return true;
  }

protected:
  ValueType *command_;
};

class AngularVelocityCommandHandle : public TwistCommandHandle
{
public:
  typedef double ValueType;

  AngularVelocityCommandHandle() : command_(0) {}
  AngularVelocityCommandHandle(const TwistCommandHandle& other) : TwistCommandHandle(other), command_(0) {}
  AngularVelocityCommandHandle(QuadrotorInterface *interface, const std::string& controller) : TwistCommandHandle(interface, controller), command_(0) {}
  AngularVelocityCommandHandle(const TwistHandle &twist, double* command) : TwistCommandHandle(twist, 0) { *this = command; }

  std::string getName() const { return "twist.angular.z"; }
  bool connected() const { return get(); }

  AngularVelocityCommandHandle& operator=(Twist *source) { TwistCommandHandle::operator=(source); command_ = 0; return *this; }
  AngularVelocityCommandHandle& operator=(double *source) { command_ = source; return *this; }
  ValueType *get() const { return command_ ? command_ : (TwistCommandHandle::get() ? &(TwistCommandHandle::get()->angular.z) : 0); }

  ValueType& command() { return *get(); }
  double getCommand() const { return *get(); }
  void setCommand(double command) { *get() = command; }

  bool update(Twist& command) const {
    if (!connected()) return false;
    command.linear.z = getCommand();
    return true;
  }

protected:
  ValueType *command_;
};

class WrenchCommandHandle : public CommandHandle
{
public:
  typedef Wrench ValueType;

  WrenchCommandHandle() : command_(0) {}
  WrenchCommandHandle(QuadrotorInterface *interface, const std::string& controller) : CommandHandle(interface, controller), command_(0) {}

  std::string getName() const { return "wrench"; }
  bool connected() const { return get(); }

  WrenchCommandHandle& operator=(Wrench *source) { command_ = source; return *this; }
  ValueType *get() const { return command_; }

  ValueType& command() { return *get(); }
  void setCommand(const Wrench& command) { *get() = command; }
  const Wrench& getCommand() const { return *get(); }

  bool update(Wrench& command) const {
    if (!connected()) return false;
    command = *get();
    return true;
  }

protected:
  ValueType *command_;
};

class MotorCommandHandle : public CommandHandle
{
public:
  typedef MotorCommand ValueType;

  MotorCommandHandle() : command_(0) {}
  MotorCommandHandle(QuadrotorInterface *interface, const std::string& controller) : CommandHandle(interface, controller), command_(0) {}

  std::string getName() const { return "motor"; }
  bool connected() const { return get(); }

  MotorCommandHandle& operator=(MotorCommand *source) { command_ = source; return *this; }
  ValueType *get() const { return command_; }

  ValueType& command() { return *get(); }
  void setCommand(const MotorCommand& command) { *get() = command; }
  const MotorCommand& getCommand() const { return *get(); }

  bool update(MotorCommand& command) const {
    if (!connected()) return false;
    command = *get();
    return true;
  }

protected:
  ValueType *command_;
};

} // namespace hector_quadrotor_controller

#endif // HECTOR_QUADROTOR_CONTROLLER_QUADROTOR_INTERFACE_H
