//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
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

#include <hector_quadrotor_controller/quadrotor_propulsion.h>
#include "common/Events.hh"
#include "physics/physics.h"

extern "C" {
  #include "quadrotorPropulsion/quadrotorPropulsion.h"
  #include "quadrotorPropulsion/quadrotorPropulsion_initialize.h"
}

#include <boost/array.hpp>

namespace gazebo {

using namespace common;
using namespace math;

template <typename T> static inline T checknan(const T& value, const std::string& text = "") {
  if (!(value == value)) {
    if (!text.empty()) std::cerr << text << " contains **!?* Nan values!" << std::endl;
    return T();
  }
  return value;
}

// extern void quadrotorPropulsion(const real_T xin[4], const real_T uin[10], const PropulsionParameters parameter, real_T dt, real_T y[6], real_T xpred[4]);
struct GazeboQuadrotorPropulsion::PropulsionModel {
  PropulsionParameters parameters_;
  boost::array<real_T,4>  x;
  boost::array<real_T,4>  x_pred;
  boost::array<real_T,10> u;
  boost::array<real_T,14> y;
};

GazeboQuadrotorPropulsion::GazeboQuadrotorPropulsion()
{
  // initialize propulsion model
  quadrotorPropulsion_initialize();
  propulsion_model_ = new PropulsionModel;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboQuadrotorPropulsion::~GazeboQuadrotorPropulsion()
{
  event::Events::DisconnectWorldUpdateStart(updateConnection);
  node_handle_->shutdown();
  callback_queue_thread_.join();
  delete node_handle_;

  delete propulsion_model_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboQuadrotorPropulsion::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  world = _model->GetWorld();
  link = _model->GetLink();

  // load parameters
  if (!_sdf->HasElement("robotNamespace"))
    namespace_.clear();
  else
    namespace_ = _sdf->GetElement("robotNamespace")->GetValueString() + "/";

  if (!_sdf->HasElement("paramNamespace"))
    param_namespace_ = "~/quadrotor_propulsion/";
  else
    param_namespace_ = _sdf->GetElement("paramNamespace")->GetValueString() + "/";

  if (!_sdf->HasElement("voltageTopicName"))
    voltage_topic_ = "motor_pwm";
  else
    voltage_topic_ = _sdf->GetElement("voltageTopicName")->GetValueString();

  if (!_sdf->HasElement("wrenchTopic"))
    wrench_topic_ = "wrench_out";
  else
    wrench_topic_ = _sdf->GetElement("wrenchTopic")->GetValueString();

  if (!_sdf->HasElement("supplyTopic"))
    supply_topic_ = "supply";
  else
    supply_topic_ = _sdf->GetElement("supplyTopic")->GetValueString();

  if (!_sdf->HasElement("statusTopic"))
    status_topic_ = "motor_status";
  else
    status_topic_ = _sdf->GetElement("statusTopic")->GetValueString();

  // set control timing parameters
  control_rate_ = 0.0;
  if (_sdf->HasElement("controlRate")) control_rate_ = _sdf->GetElement("controlRate")->GetValueDouble();
  control_period_ = (control_rate_ > 0) ? (1.0 / control_rate_) : 0.0;
  control_tolerance_ = Time();
  if (_sdf->HasElement("controlTolerance")) control_tolerance_ = _sdf->GetElement("controlTolerance")->GetValueDouble();
  control_delay_ = Time();
  if (_sdf->HasElement("controlDelay")) control_delay_ = _sdf->GetElement("controlDelay")->GetValueDouble();

  // set initial supply voltage
  supply_.voltage.resize(1);
  supply_.voltage[0] = 14.8;
  if (_sdf->HasElement("supplyVoltage")) supply_.voltage[0] = _sdf->GetElement("supplyVoltage")->GetValueDouble();

  // start ros node
  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  node_handle_ = new ros::NodeHandle(namespace_);

  // subscribe command
  if (!voltage_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<hector_uav_msgs::MotorPWM>(
      voltage_topic_, 1,
      boost::bind(&GazeboQuadrotorPropulsion::CommandCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    voltage_subscriber_ = node_handle_->subscribe(ops);
  }

  // publish wrench
  if (!wrench_topic_.empty())
  {
    ros::AdvertiseOptions ops = ros::AdvertiseOptions::create<geometry_msgs::Wrench>(
      wrench_topic_, 10,
      ros::SubscriberStatusCallback(), ros::SubscriberStatusCallback(),
      ros::VoidConstPtr(), &callback_queue_);
    wrench_publisher_ = node_handle_->advertise(ops);
  }

  // publish and latch supply
  if (!supply_topic_.empty())
  {
    ros::AdvertiseOptions ops = ros::AdvertiseOptions::create<hector_uav_msgs::Supply>(
      supply_topic_, 10,
      ros::SubscriberStatusCallback(), ros::SubscriberStatusCallback(),
      ros::VoidConstPtr(), &callback_queue_);
    ops.latch = true;
    supply_publisher_ = node_handle_->advertise(ops);

    supply_publisher_.publish(supply_);
  }

  // publish motor_status
  if (!status_topic_.empty())
  {
    ros::AdvertiseOptions ops = ros::AdvertiseOptions::create<hector_uav_msgs::MotorStatus>(
      status_topic_, 10,
      ros::SubscriberStatusCallback(), ros::SubscriberStatusCallback(),
      ros::VoidConstPtr(), &callback_queue_);
    motor_status_publisher_ = node_handle_->advertise(ops);
  }

  callback_queue_thread_ = boost::thread( boost::bind( &GazeboQuadrotorPropulsion::QueueThread,this ) );

  Reset();

  // get model parameters
  ros::NodeHandle param(param_namespace_);
  param.getParam("k_t",  propulsion_model_->parameters_.k_t);
  param.getParam("CT0s", propulsion_model_->parameters_.CT0s);
  param.getParam("CT1s", propulsion_model_->parameters_.CT1s);
  param.getParam("CT2s", propulsion_model_->parameters_.CT2s);
  param.getParam("J_M",  propulsion_model_->parameters_.J_M);
  param.getParam("l_m",  propulsion_model_->parameters_.l_m);
  param.getParam("Psi",  propulsion_model_->parameters_.Psi);
  param.getParam("R_A",  propulsion_model_->parameters_.R_A);

  std::cout << "Loaded the following quadrotor propulsion model parameters from namespace " << param.getNamespace() << ":" << std::endl;
  std::cout << "k_t = " << propulsion_model_->parameters_.k_t << std::endl;
  std::cout << "CT2s = " << propulsion_model_->parameters_.CT2s << std::endl;
  std::cout << "CT1s = " << propulsion_model_->parameters_.CT1s << std::endl;
  std::cout << "CT0s = " << propulsion_model_->parameters_.CT0s << std::endl;
  std::cout << "Psi = "  << propulsion_model_->parameters_.Psi << std::endl;
  std::cout << "J_M = "  << propulsion_model_->parameters_.J_M << std::endl;
  std::cout << "R_A = "  << propulsion_model_->parameters_.R_A << std::endl;
  std::cout << "l_m = "  << propulsion_model_->parameters_.l_m << std::endl;

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection = event::Events::ConnectWorldUpdateStart(
      boost::bind(&GazeboQuadrotorPropulsion::Update, this));
}

////////////////////////////////////////////////////////////////////////////////
// Callbacks
void GazeboQuadrotorPropulsion::CommandCallback(const hector_uav_msgs::MotorPWMConstPtr& command)
{
  boost::mutex::scoped_lock lock(command_mutex_);
  motor_status_.on = true;
  new_motor_voltages_.push_back(command);
  ROS_DEBUG_STREAM("Received motor command valid at " << command->header.stamp);
  command_condition_.notify_all();
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboQuadrotorPropulsion::Update()
{
  Vector3 force, torque;
  boost::mutex::scoped_lock lock(command_mutex_);

  // Get simulator time
  Time current_time = world->GetSimTime();
  Time dt = current_time - last_time_;
  last_time_ = current_time;
  if (dt <= 0.0) return;

  // Get new commands/state
  // callback_queue_.callAvailable();

  while(1) {
    if (!new_motor_voltages_.empty()) {
      hector_uav_msgs::MotorPWMConstPtr new_motor_voltage = new_motor_voltages_.front();
      Time new_time = Time(new_motor_voltage->header.stamp.sec, new_motor_voltage->header.stamp.nsec);
      if (new_time == Time() || (new_time >= current_time - control_delay_ - dt - control_tolerance_ && new_time <= current_time - control_delay_ + control_tolerance_)) {
        motor_voltage_ = new_motor_voltage;
        new_motor_voltages_.pop_front();
        last_control_time_ = current_time;
        ROS_DEBUG_STREAM_NAMED("quadrotor_propulsion", "Using motor command valid at t = " << new_time.Double() << " for simulation step at t = " << current_time.Double() << " (dt = " << (current_time - new_time).Double() << ")");
        break;
      } else if (new_time < current_time - control_delay_ - control_tolerance_) {
        ROS_DEBUG_NAMED("quadrotor_propulsion", "command received was too old: %f s", (new_time - current_time).Double());
        new_motor_voltages_.pop_front();
        continue;
      }
    }

    if (new_motor_voltages_.empty() && motor_status_.on &&  control_period_ > 0 && current_time >= last_control_time_ + control_period_ + control_tolerance_) {
      ROS_DEBUG_NAMED("quadrotor_propulsion", "Waiting for command at simulation step t = %f s... last update was %f s ago", current_time.Double(), (current_time - last_control_time_).Double());
      if (command_condition_.timed_wait(lock, (ros::Duration(control_period_.sec, control_period_.nsec) * 100.0).toBoost())) continue;
      ROS_ERROR_NAMED("quadrotor_propulsion", "command timed out.");
      motor_status_.on = false;
    }

    break;
  }

  // fill input vector u for propulsion model
  velocity = link->GetRelativeLinearVel();
  rate = link->GetRelativeAngularVel();
  propulsion_model_->u[0] = velocity.x;
  propulsion_model_->u[1] = -velocity.y;
  propulsion_model_->u[2] = -velocity.z;
  propulsion_model_->u[3] = rate.x;
  propulsion_model_->u[4] = -rate.y;
  propulsion_model_->u[5] = -rate.z;
  if (motor_voltage_ && motor_voltage_->pwm.size() >= 4) {
    propulsion_model_->u[6] = motor_voltage_->pwm[0] * supply_.voltage[0] / 255.0;
    propulsion_model_->u[7] = motor_voltage_->pwm[1] * supply_.voltage[0] / 255.0;
    propulsion_model_->u[8] = motor_voltage_->pwm[2] * supply_.voltage[0] / 255.0;
    propulsion_model_->u[9] = motor_voltage_->pwm[3] * supply_.voltage[0] / 255.0;
  } else {
    propulsion_model_->u[6] = 0.0;
    propulsion_model_->u[7] = 0.0;
    propulsion_model_->u[8] = 0.0;
    propulsion_model_->u[9] = 0.0;
  }

//  std::cout << "u = [ ";
//  for(std::size_t i = 0; i < propulsion_model_->u.size(); ++i)
//    std::cout << propulsion_model_->u[i] << " ";
//  std::cout << "]" << std::endl;

  checknan(propulsion_model_->u, "propulsion model input");
  checknan(propulsion_model_->x, "propulsion model state");

  // update propulsion model
  quadrotorPropulsion(propulsion_model_->x.data(), propulsion_model_->u.data(), propulsion_model_->parameters_, dt.Double(), propulsion_model_->y.data(), propulsion_model_->x_pred.data());
  propulsion_model_->x = propulsion_model_->x_pred;
  force  = force  + checknan(Vector3(propulsion_model_->y[0], -propulsion_model_->y[1], propulsion_model_->y[2]), "propulsion model force");
  torque = torque + checknan(Vector3(propulsion_model_->y[3], -propulsion_model_->y[4], -propulsion_model_->y[5]), "propulsion model torque");

//  std::cout << "y = [ ";
//  for(std::size_t i = 0; i < propulsion_model_->y.size(); ++i)
//    std::cout << propulsion_model_->y[i] << " ";
//  std::cout << "]" << std::endl;

  // publish wrench
  if (wrench_publisher_) {
    wrench_.force.x = force.x;
    wrench_.force.y = force.y;
    wrench_.force.z = force.z;
    wrench_.torque.x = torque.x;
    wrench_.torque.y = torque.y;
    wrench_.torque.z = torque.z;
    wrench_publisher_.publish(wrench_);
  }

  // set force and torque in gazebo
  link->AddRelativeForce(force);
  link->AddRelativeTorque(torque - link->GetInertial()->GetCoG().GetCrossProd(force));

  // publish motor status
  if (motor_status_publisher_ && current_time >= last_motor_status_time_ + control_period_) {
    motor_status_.header.stamp = ros::Time(current_time.sec, current_time.nsec);

    motor_status_.voltage.resize(4);
    motor_status_.voltage[0] = propulsion_model_->u[6];
    motor_status_.voltage[1] = propulsion_model_->u[7];
    motor_status_.voltage[2] = propulsion_model_->u[8];
    motor_status_.voltage[3] = propulsion_model_->u[9];

    motor_status_.frequency.resize(4);
    motor_status_.frequency[0] = propulsion_model_->y[6];
    motor_status_.frequency[1] = propulsion_model_->y[7];
    motor_status_.frequency[2] = propulsion_model_->y[8];
    motor_status_.frequency[3] = propulsion_model_->y[9];
    motor_status_.running = motor_status_.frequency[0] > 1.0 && motor_status_.frequency[1] > 1.0 && motor_status_.frequency[2] > 1.0 && motor_status_.frequency[3] > 1.0;

    motor_status_.current.resize(4);
    motor_status_.current[0] = propulsion_model_->y[10];
    motor_status_.current[1] = propulsion_model_->y[11];
    motor_status_.current[2] = propulsion_model_->y[12];
    motor_status_.current[3] = propulsion_model_->y[13];
    motor_status_publisher_.publish(motor_status_);
    last_motor_status_time_ = current_time;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Reset the controller
void GazeboQuadrotorPropulsion::Reset()
{
  propulsion_model_->x.assign(0.0);
  propulsion_model_->x_pred.assign(0.0);
  last_control_time_ = Time();
  last_motor_status_time_ = Time();
  new_motor_voltages_.clear();
  motor_voltage_.reset();
}

////////////////////////////////////////////////////////////////////////////////
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
