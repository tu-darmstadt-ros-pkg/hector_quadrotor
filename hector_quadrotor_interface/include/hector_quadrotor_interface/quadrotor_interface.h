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

#ifndef HECTOR_QUADROTOR_INTERFACE_QUADROTOR_INTERFACE_H
#define HECTOR_QUADROTOR_INTERFACE_QUADROTOR_INTERFACE_H

#include <hardware_interface/internal/hardware_resource_manager.h>

#include <hector_quadrotor_interface/handles.h>

#include <map>
#include <list>

namespace hector_quadrotor_interface
{

class QuadrotorInterface : public hardware_interface::HardwareInterface
{
public:
  QuadrotorInterface();

  virtual ~QuadrotorInterface();

  // TODO template
  void registerPose(geometry_msgs::Pose *pose){
    pose_ = PoseHandlePtr(new PoseHandle(this, pose));
  }

  void registerTwist(geometry_msgs::Twist *twist){
    twist_ = TwistHandlePtr(new TwistHandle(this, twist));
  }

  void registerAccel(geometry_msgs::Accel *accel){
    accel_ = AccelerationHandlePtr(new AccelerationHandle(this, accel));
  }
  void registerSensorImu(sensor_msgs::Imu *imu){
    imu_ = ImuHandlePtr(new ImuHandle(this, imu));
  }

  void registerMotorStatus(hector_uav_msgs::MotorStatus *motor_status){
    motor_status_ = MotorStatusHandlePtr(new MotorStatusHandle(this, motor_status));
  }

  PoseHandlePtr getPose()
  {
    return pose_;
  }

  TwistHandlePtr getTwist()
  {
    return twist_;
  }

  AccelerationHandlePtr getAccel()
  {
    return accel_;
  }

  ImuHandlePtr getSensorImu()
  {
    return imu_;
  }

  MotorStatusHandlePtr getMotorStatus()
  {
    return motor_status_;
  }

  PoseHandlePtr pose_;
  TwistHandlePtr twist_;
  AccelerationHandlePtr accel_;
  ImuHandlePtr imu_;
  MotorStatusHandlePtr motor_status_;

  template<typename HandleType>
  boost::shared_ptr<HandleType> getHandle()
  {
    return boost::shared_ptr<HandleType>(new HandleType(this));
  }

  template<typename HandleType>
  boost::shared_ptr<HandleType> addInput(const std::string &name)
  {
    boost::shared_ptr<HandleType> input = getInput<HandleType>(name);
    if (input) { return input; }

    // create new input handle
    input.reset(new HandleType(this, name));
    inputs_[name] = input;

    // connect to outputs of same name
    if (outputs_.count(name))
    {
      boost::shared_ptr<HandleType> output = boost::dynamic_pointer_cast<HandleType>(outputs_.at(name));
      if (output) output->connectTo(*input);
    }

    return input;
  }

  template<typename HandleType>
  boost::shared_ptr<HandleType> addOutput(const std::string &name)
  {
    boost::shared_ptr<HandleType> output = getOutput<HandleType>(name);
    if (output) { return output; }

    // create new output handle
    output.reset(new HandleType(this, name));
    outputs_[name] = output;
    *output = output->ownData(new typename HandleType::ValueType());

    //claim resource
    claim(name);

    // connect to inputs of same name
    if (inputs_.count(name))
    {
      boost::shared_ptr<HandleType> input = boost::dynamic_pointer_cast<HandleType>(inputs_.at(name));
      if (input) output->connectTo(*input);
    }

    return output;
  }

  template<typename HandleType>
  boost::shared_ptr<HandleType> getOutput(const std::string &name) const
  {
    if (!outputs_.count(name)) { return boost::shared_ptr<HandleType>(); }
    return boost::dynamic_pointer_cast<HandleType>(outputs_.at(name));
  }

  template<typename HandleType>
  boost::shared_ptr<HandleType> getInput(const std::string &name) const
  {
    if (!inputs_.count(name)) { return boost::shared_ptr<HandleType>(); }
    return boost::dynamic_pointer_cast<HandleType>(inputs_.at(name));
  }

  template<typename HandleType>
  typename HandleType::ValueType const *getCommand(const std::string &name) const
  {
    boost::shared_ptr<HandleType> output = getOutput<HandleType>(name);
    if (!output || !output->connected())
    { return 0; }
    return &(output->command());
  }

  const Pose *getPoseCommand() const;
  const Twist *getTwistCommand() const;
  const Wrench *getWrenchCommand() const;
  const MotorCommand *getMotorCommand() const;

  bool enabled(const CommandHandle *handle) const;
  bool start(const CommandHandle *handle);
  bool stop(const CommandHandle *handle);
  void disconnect(const CommandHandle *handle);

  bool preempt(const CommandHandle *handle);

private:
  std::map<std::string, CommandHandlePtr> inputs_;
  std::map<std::string, CommandHandlePtr> outputs_;
  std::map<std::string, const CommandHandle *> enabled_;
};

} // namespace hector_quadrotor_interface

#endif // HECTOR_QUADROTOR_INTERFACE_QUADROTOR_INTERFACE_H
