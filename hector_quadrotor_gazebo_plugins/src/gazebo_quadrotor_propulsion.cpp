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

#include <hector_quadrotor_gazebo_plugins/gazebo_quadrotor_propulsion.h>
#include <hector_quadrotor_model/helpers.h>

#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#include <rosgraph_msgs/Clock.h>
#include <geometry_msgs/WrenchStamped.h>

#if (GAZEBO_MAJOR_VERSION >= 8)
namespace hector_quadrotor_model {

template <typename Message, typename T> static inline void toVector(const Message& msg, ignition::math::Vector3<T>& vector)
{
  vector.X() = msg.x;
  vector.Y() = msg.y;
  vector.Z() = msg.z;
}

template <typename Message, typename T> static inline void fromVector(const ignition::math::Vector3<T>& vector, Message& msg)
{
  msg.x = vector.X();
  msg.y = vector.Y();
  msg.z = vector.Z();
}

template <typename Message, typename T> static inline void toQuaternion(const Message& msg, ignition::math::Quaternion<T>& quaternion)
{
  quaternion.W() = msg.w;
  quaternion.X() = msg.x;
  quaternion.Y() = msg.y;
  quaternion.Z() = msg.z;
}

template <typename Message, typename T> static inline void fromQuaternion(const ignition::math::Quaternion<T>& quaternion, Message& msg)
{
  msg.w = quaternion.W();
  msg.x = quaternion.X();
  msg.y = quaternion.Y();
  msg.z = quaternion.Z();
}

} // namespace hector_quadrotor_model
#endif

namespace gazebo {

using namespace common;
using namespace hector_quadrotor_model;

GazeboQuadrotorPropulsion::GazeboQuadrotorPropulsion()
  : node_handle_(0)
{
}

GazeboQuadrotorPropulsion::~GazeboQuadrotorPropulsion()
{
#if (GAZEBO_MAJOR_VERSION < 8)
  event::Events::DisconnectWorldUpdateBegin(updateConnection);
#endif
  updateConnection.reset();

  if (node_handle_) {
    node_handle_->shutdown();
    if (callback_queue_thread_.joinable())
      callback_queue_thread_.join();
    delete node_handle_;
  }
}

//////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboQuadrotorPropulsion::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  world = _model->GetWorld();
  link = _model->GetLink();

  // default parameters
  namespace_.clear();
  param_namespace_ = "quadrotor_propulsion";
  trigger_topic_ = "quadro/trigger";
  command_topic_ = "command/motor";
  pwm_topic_ = "motor_pwm";
  wrench_topic_ = "propulsion/wrench";
  supply_topic_ = "supply";
  status_topic_ = "motor_status";
  control_tolerance_ = ros::Duration();
  control_delay_ = ros::Duration();
  frame_id_ = link->GetName();

  // load parameters
  if (_sdf->HasElement("robotNamespace"))   namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  if (_sdf->HasElement("paramNamespace"))   param_namespace_ = _sdf->GetElement("paramNamespace")->Get<std::string>();
  if (_sdf->HasElement("triggerTopic"))     trigger_topic_ = _sdf->GetElement("triggerTopic")->Get<std::string>();
  if (_sdf->HasElement("topicName"))        command_topic_ = _sdf->GetElement("topicName")->Get<std::string>();
  if (_sdf->HasElement("pwmTopicName"))     pwm_topic_ = _sdf->GetElement("pwmTopicName")->Get<std::string>();
  if (_sdf->HasElement("wrenchTopic"))      wrench_topic_ = _sdf->GetElement("wrenchTopic")->Get<std::string>();
  if (_sdf->HasElement("supplyTopic"))      supply_topic_ = _sdf->GetElement("supplyTopic")->Get<std::string>();
  if (_sdf->HasElement("statusTopic"))      status_topic_ = _sdf->GetElement("statusTopic")->Get<std::string>();
  if (_sdf->HasElement("frameId"))          frame_id_= _sdf->GetElement("frameId")->Get<std::string>();

  if (_sdf->HasElement("voltageTopicName")) {
    gzwarn << "[quadrotor_propulsion] Plugin parameter 'voltageTopicName' is deprecated! Plese change your config to use "
           << "'topicName' for MotorCommand messages or 'pwmTopicName' for MotorPWM messages." << std::endl;
    pwm_topic_ = _sdf->GetElement("voltageTopicName")->Get<std::string>();
  }

  // set control timing parameters
  controlTimer.Load(world, _sdf, "control");
  motorStatusTimer.Load(world, _sdf, "motorStatus");
  if (_sdf->HasElement("controlTolerance")) control_tolerance_.fromSec(_sdf->GetElement("controlTolerance")->Get<double>());
  if (_sdf->HasElement("controlDelay"))     control_delay_.fromSec(_sdf->GetElement("controlDelay")->Get<double>());

  // set initial supply voltage
  if (_sdf->HasElement("supplyVoltage"))    model_.setInitialSupplyVoltage(_sdf->GetElement("supplyVoltage")->Get<double>());

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  node_handle_ = new ros::NodeHandle(namespace_);

  // get model parameters
  if (!model_.configure(ros::NodeHandle(*node_handle_, param_namespace_))) {
    gzwarn << "[quadrotor_propulsion] Could not properly configure the propulsion plugin. Make sure you loaded the parameter file." << std::endl;
    return;
  }

  // publish trigger
  if (!trigger_topic_.empty())
  {
    ros::AdvertiseOptions ops;
    ops.callback_queue = &callback_queue_;
    ops.init<rosgraph_msgs::Clock>(trigger_topic_, 10);
    trigger_publisher_ = node_handle_->advertise(ops);
  }

  // subscribe voltage command
  if (!command_topic_.empty())
  {
    ros::SubscribeOptions ops;
    ops.callback_queue = &callback_queue_;
    ops.init<hector_uav_msgs::MotorCommand>(command_topic_, 1, boost::bind(&QuadrotorPropulsion::addCommandToQueue, &model_, _1));
    command_subscriber_ = node_handle_->subscribe(ops);
  }

  // subscribe pwm command
  if (!pwm_topic_.empty())
  {
    ros::SubscribeOptions ops;
    ops.callback_queue = &callback_queue_;
    ops.init<hector_uav_msgs::MotorPWM>(pwm_topic_, 1, boost::bind(&QuadrotorPropulsion::addPWMToQueue, &model_, _1));
    pwm_subscriber_ = node_handle_->subscribe(ops);
  }

  // advertise wrench
  if (!wrench_topic_.empty())
  {
    ros::AdvertiseOptions ops;
    ops.callback_queue = &callback_queue_;
    ops.init<geometry_msgs::WrenchStamped>(wrench_topic_, 10);
    wrench_publisher_ = node_handle_->advertise(ops);
  }

  // advertise and latch supply
  if (!supply_topic_.empty())
  {
    ros::AdvertiseOptions ops;
    ops.callback_queue = &callback_queue_;
    ops.latch = true;
    ops.init<hector_uav_msgs::Supply>(supply_topic_, 10);
    supply_publisher_ = node_handle_->advertise(ops);
    supply_publisher_.publish(model_.getSupply());
  }

  // advertise motor_status
  if (!status_topic_.empty())
  {
    ros::AdvertiseOptions ops;
    ops.callback_queue = &callback_queue_;
    ops.init<hector_uav_msgs::MotorStatus>(status_topic_, 10);
    motor_status_publisher_ = node_handle_->advertise(ops);
  }

  // callback_queue_thread_ = boost::thread( boost::bind( &GazeboQuadrotorPropulsion::QueueThread,this ) );

  Reset();

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboQuadrotorPropulsion::Update, this));
}

//////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboQuadrotorPropulsion::Update()
{
  // Get simulator time
#if (GAZEBO_MAJOR_VERSION >= 8)
  Time current_time = world->SimTime();
#else
  Time current_time = world->GetSimTime();
#endif
  Time dt = current_time - last_time_;
  last_time_ = current_time;
  if (dt <= 0.0) return;

  // Send trigger
  bool trigger = controlTimer.getUpdatePeriod() > 0.0 ? controlTimer.update() : false;
  if (trigger && trigger_publisher_) {
    rosgraph_msgs::Clock clock;
    clock.clock = ros::Time(current_time.sec, current_time.nsec);
    trigger_publisher_.publish(clock);

    ROS_DEBUG_STREAM_NAMED("quadrotor_propulsion", "Sent a trigger message at t = " << current_time.Double() << " (dt = " << (current_time - last_trigger_time_).Double() << ")");
    last_trigger_time_ = current_time;
  }

  // Get new commands/state
  callback_queue_.callAvailable();

  // Process input queue
  model_.processQueue(ros::Time(current_time.sec, current_time.nsec), control_tolerance_, control_delay_, (model_.getMotorStatus().on && trigger) ? ros::WallDuration(1.0) : ros::WallDuration(), &callback_queue_);

  // fill input vector u for propulsion model
  geometry_msgs::Twist twist;
#if (GAZEBO_MAJOR_VERSION >= 8)
  fromVector(link->RelativeLinearVel(), twist.linear);
  fromVector(link->RelativeAngularVel(), twist.angular);
#else
  fromVector(link->GetRelativeLinearVel(), twist.linear);
  fromVector(link->GetRelativeAngularVel(), twist.angular);
#endif
  model_.setTwist(twist);

  // update the model
  model_.update(dt.Double());

  // get wrench from model
#if (GAZEBO_MAJOR_VERSION >= 8)
  ignition::math::Vector3d force, torque;
#else
  math::Vector3 force, torque;
#endif
  toVector(model_.getWrench().force, force);
  toVector(model_.getWrench().torque, torque);

  // publish wrench
  if (wrench_publisher_) {
    geometry_msgs::WrenchStamped wrench_msg;
    wrench_msg.header.stamp = ros::Time(current_time.sec, current_time.nsec);
    wrench_msg.header.frame_id = frame_id_;
    wrench_msg.wrench = model_.getWrench();
    wrench_publisher_.publish(wrench_msg);
  }

  // publish motor status
  if (motor_status_publisher_ && motorStatusTimer.update() /* && current_time >= last_motor_status_time_ + control_period_ */) {
    hector_uav_msgs::MotorStatus motor_status = model_.getMotorStatus();
    motor_status.header.stamp = ros::Time(current_time.sec, current_time.nsec);
    motor_status_publisher_.publish(motor_status);
    last_motor_status_time_ = current_time;
  }

  // publish supply
  if (supply_publisher_ && current_time >= last_supply_time_ + 1.0) {
    supply_publisher_.publish(model_.getSupply());
    last_supply_time_ = current_time;
  }

  // set force and torque in gazebo
  link->AddRelativeForce(force);
#if (GAZEBO_MAJOR_VERSION >= 8)
  link->AddRelativeTorque(torque - link->GetInertial()->CoG().Cross(force));
#else
  link->AddRelativeTorque(torque - link->GetInertial()->GetCoG().Cross(force));
#endif
}

//////////////////////////////////////////////////////////////////////////////
// Reset the controller
void GazeboQuadrotorPropulsion::Reset()
{
  model_.reset();
  last_time_ = Time();
  last_trigger_time_ = Time();
  last_motor_status_time_ = Time();
  last_supply_time_ = Time();
}

//////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void GazeboQuadrotorPropulsion::QueueThread()
{
  static const double timeout = 0.01;

  while (node_handle_->ok())
  {
    callback_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboQuadrotorPropulsion)

} // namespace gazebo
