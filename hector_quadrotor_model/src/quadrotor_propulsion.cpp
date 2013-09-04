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

#include <hector_quadrotor_model/quadrotor_propulsion.h>
#include <hector_quadrotor_model/helpers.h>

#include <ros/node_handle.h>
#include <ros/callback_queue.h>

#include <boost/array.hpp>

extern "C" {
  #include "quadrotorPropulsion/quadrotorPropulsion.h"
  #include "quadrotorPropulsion/quadrotorPropulsion_initialize.h"
}

namespace hector_quadrotor_model {

// extern void quadrotorPropulsion(const real_T xin[4], const real_T uin[10], const PropulsionParameters parameter, real_T dt, real_T y[6], real_T xpred[4]);
struct QuadrotorPropulsion::PropulsionModel {
  PropulsionParameters parameters_;
  boost::array<real_T,4>  x;
  boost::array<real_T,4>  x_pred;
  boost::array<real_T,10> u;
  boost::array<real_T,14> y;
};

QuadrotorPropulsion::QuadrotorPropulsion()
{
  // initialize propulsion model
  quadrotorPropulsion_initialize();
  propulsion_model_ = new PropulsionModel;
}

QuadrotorPropulsion::~QuadrotorPropulsion()
{
  delete propulsion_model_;
}

inline void QuadrotorPropulsion::f(const real_T xin[4], const real_T uin[10], real_T dt, real_T y[14], real_T xpred[4]) const
{
  ::quadrotorPropulsion(xin, uin, propulsion_model_->parameters_, dt, y, xpred);
}

void QuadrotorPropulsion::configure(const std::string &ns)
{
  ros::NodeHandle param(ns);

  // get model parameters
  param.getParam("k_m",     propulsion_model_->parameters_.k_m);
  param.getParam("k_t",     propulsion_model_->parameters_.k_t);
  param.getParam("CT0s",    propulsion_model_->parameters_.CT0s);
  param.getParam("CT1s",    propulsion_model_->parameters_.CT1s);
  param.getParam("CT2s",    propulsion_model_->parameters_.CT2s);
  param.getParam("J_M",     propulsion_model_->parameters_.J_M);
  param.getParam("l_m",     propulsion_model_->parameters_.l_m);
  param.getParam("Psi",     propulsion_model_->parameters_.Psi);
  param.getParam("R_A",     propulsion_model_->parameters_.R_A);
  param.getParam("alpha_m", propulsion_model_->parameters_.alpha_m);
  param.getParam("beta_m",  propulsion_model_->parameters_.beta_m);

#ifndef NDEBUG
  std::cout << "Loaded the following quadrotor propulsion model parameters from namespace " << param.getNamespace() << ":" << std::endl;
  std::cout << "k_m     = " << propulsion_model_->parameters_.k_m << std::endl;
  std::cout << "k_t     = " << propulsion_model_->parameters_.k_t << std::endl;
  std::cout << "CT2s    = " << propulsion_model_->parameters_.CT2s << std::endl;
  std::cout << "CT1s    = " << propulsion_model_->parameters_.CT1s << std::endl;
  std::cout << "CT0s    = " << propulsion_model_->parameters_.CT0s << std::endl;
  std::cout << "Psi     = " << propulsion_model_->parameters_.Psi << std::endl;
  std::cout << "J_M     = " << propulsion_model_->parameters_.J_M << std::endl;
  std::cout << "R_A     = " << propulsion_model_->parameters_.R_A << std::endl;
  std::cout << "l_m     = " << propulsion_model_->parameters_.l_m << std::endl;
  std::cout << "alpha_m = " << propulsion_model_->parameters_.alpha_m << std::endl;
  std::cout << "beta_m  = " << propulsion_model_->parameters_.beta_m << std::endl;
#endif

  initial_voltage_ = 14.8;
  param.getParam("supply_voltage", initial_voltage_);
  reset();
}

void QuadrotorPropulsion::reset()
{
  boost::mutex::scoped_lock lock(mutex_);

  propulsion_model_->x.assign(0.0);
  propulsion_model_->x_pred.assign(0.0);
  propulsion_model_->u.assign(0.0);
  propulsion_model_->y.assign(0.0);

  wrench_ = geometry_msgs::Wrench();

  motor_status_ = hector_uav_msgs::MotorStatus();
  motor_status_.voltage.resize(4);
  motor_status_.frequency.resize(4);
  motor_status_.current.resize(4);

  supply_ = hector_uav_msgs::Supply();
  supply_.voltage.resize(1);
  supply_.voltage[0] = initial_voltage_;

  last_command_time_ = ros::Time();

  command_queue_ = std::queue<hector_uav_msgs::MotorPWMConstPtr>(); // .clear();
}

void QuadrotorPropulsion::setVoltage(const hector_uav_msgs::MotorPWM& voltage)
{
  boost::mutex::scoped_lock lock(mutex_);
  last_command_time_ = voltage.header.stamp;

  if (motor_status_.on && voltage.pwm.size() >= 4) {
    propulsion_model_->u[6] = voltage.pwm[0] * supply_.voltage[0] / 255.0;
    propulsion_model_->u[7] = voltage.pwm[1] * supply_.voltage[0] / 255.0;
    propulsion_model_->u[8] = voltage.pwm[2] * supply_.voltage[0] / 255.0;
    propulsion_model_->u[9] = voltage.pwm[3] * supply_.voltage[0] / 255.0;
  } else {
    propulsion_model_->u[6] = 0.0;
    propulsion_model_->u[7] = 0.0;
    propulsion_model_->u[8] = 0.0;
    propulsion_model_->u[9] = 0.0;
  }
}

void QuadrotorPropulsion::setTwist(const geometry_msgs::Twist &twist)
{
  boost::mutex::scoped_lock lock(mutex_);
  propulsion_model_->u[0] = twist.linear.x;
  propulsion_model_->u[1] = -twist.linear.y;
  propulsion_model_->u[2] = -twist.linear.z;
  propulsion_model_->u[3] = twist.angular.x;
  propulsion_model_->u[4] = -twist.angular.y;
  propulsion_model_->u[5] = -twist.angular.z;
}

void QuadrotorPropulsion::addVoltageToQueue(const hector_uav_msgs::MotorPWMConstPtr& command)
{
  boost::mutex::scoped_lock lock(command_queue_mutex_);

  if (!motor_status_.on) {
    ROS_WARN_NAMED("quadrotor_propulsion", "Received new motor command. Enabled motors.");
    engage();
  }

  ROS_DEBUG_STREAM_NAMED("quadrotor_propulsion", "Received motor command valid at " << command->header.stamp);
  command_queue_.push(command);
  command_condition_.notify_all();
}

bool QuadrotorPropulsion::processQueue(const ros::Time &timestamp, const ros::Duration &tolerance, const ros::Duration &delay, const ros::WallDuration &wait, ros::CallbackQueue *callback_queue) {
  boost::mutex::scoped_lock lock(command_queue_mutex_);
  bool new_command = false;

  ros::Time min(timestamp), max(timestamp);
  try {
    min = timestamp - delay - tolerance /* - ros::Duration(dt) */;
  } catch (std::runtime_error &e) {}

  try {
    max = timestamp - delay + tolerance;
  } catch (std::runtime_error &e) {}

  do {

    if (!command_queue_.empty()) {
      hector_uav_msgs::MotorPWMConstPtr new_motor_voltage = command_queue_.front();
      ros::Time new_time = new_motor_voltage->header.stamp;

      if (new_time.isZero() || (new_time >= min && new_time <= max)) {
        setVoltage(*new_motor_voltage);
        command_queue_.pop();
        new_command = true;

        ROS_DEBUG_STREAM_NAMED("quadrotor_propulsion", "Using motor command valid at t = " << new_time.toSec() << "s for simulation step at t = " << timestamp.toSec() << "s (dt = " << (timestamp - new_time).toSec() << "s)");

      // new motor command is too old:
      } else if (new_time < min) {
        ROS_DEBUG_NAMED("quadrotor_propulsion", "Command received was %fs too old. Discarding.", (new_time - timestamp).toSec());
        command_queue_.pop();
        continue;

      // new motor command is too new:
      } else {
        // do nothing
      }

    } else {
      if (!motor_status_.on || wait.isZero()) break;

      ROS_DEBUG_NAMED("quadrotor_propulsion", "Waiting for command at simulation step t = %fs... last update was %fs ago", timestamp.toSec(), (timestamp - last_command_time_).toSec());
      if (!callback_queue) {
        if (command_condition_.timed_wait(lock, wait.toBoost())) continue;
      } else {
        lock.unlock();
        callback_queue->callAvailable(wait);
        lock.lock();
        if (!command_queue_.empty()) continue;
      }

      ROS_ERROR_NAMED("quadrotor_propulsion", "Command timed out. Disabled motors.");
      shutdown();
    }

  } while(false);

  return new_command;
}

void QuadrotorPropulsion::update(double dt)
{
  if (dt <= 0.0) return;

//  std::cout << "u = [ ";
//  for(std::size_t i = 0; i < propulsion_model_->u.size(); ++i)
//    std::cout << propulsion_model_->u[i] << " ";
//  std::cout << "]" << std::endl;

  checknan(propulsion_model_->u, "propulsion model input");
  checknan(propulsion_model_->x, "propulsion model state");

  // update propulsion model
  f(propulsion_model_->x.data(), propulsion_model_->u.data(), dt, propulsion_model_->y.data(), propulsion_model_->x_pred.data());
  propulsion_model_->x = propulsion_model_->x_pred;

  checknan(propulsion_model_->y, "propulsion model output");

  //  std::cout << "y = [ ";
  //  for(std::size_t i = 0; i < propulsion_model_->y.size(); ++i)
  //    std::cout << propulsion_model_->y[i] << " ";
  //  std::cout << "]" << std::endl;

  wrench_.force.x  =  propulsion_model_->y[0];
  wrench_.force.y  = -propulsion_model_->y[1];
  wrench_.force.z  =  propulsion_model_->y[2];
  wrench_.torque.x =  propulsion_model_->y[3];
  wrench_.torque.y = -propulsion_model_->y[4];
  wrench_.torque.z = -propulsion_model_->y[5];

  motor_status_.voltage[0] = propulsion_model_->u[6];
  motor_status_.voltage[1] = propulsion_model_->u[7];
  motor_status_.voltage[2] = propulsion_model_->u[8];
  motor_status_.voltage[3] = propulsion_model_->u[9];

  motor_status_.frequency[0] = propulsion_model_->y[6];
  motor_status_.frequency[1] = propulsion_model_->y[7];
  motor_status_.frequency[2] = propulsion_model_->y[8];
  motor_status_.frequency[3] = propulsion_model_->y[9];
  motor_status_.running = motor_status_.frequency[0] > 1.0 && motor_status_.frequency[1] > 1.0 && motor_status_.frequency[2] > 1.0 && motor_status_.frequency[3] > 1.0;

  motor_status_.current[0] = propulsion_model_->y[10];
  motor_status_.current[1] = propulsion_model_->y[11];
  motor_status_.current[2] = propulsion_model_->y[12];
  motor_status_.current[3] = propulsion_model_->y[13];
}

void QuadrotorPropulsion::engage()
{
  motor_status_.on = true;
}

void QuadrotorPropulsion::shutdown()
{
  motor_status_.on = false;
}

} // namespace hector_quadrotor_model
