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

#include <hector_quadrotor_model/quadrotor_propulsion.h>
#include <hector_quadrotor_model/helpers.h>

#include <ros/node_handle.h>
#include <ros/callback_queue.h>

#include <boost/array.hpp>

#include "matlab_helpers.h"

//extern "C" {
//  #include "quadrotorPropulsion/quadrotorPropulsion.h"
//  #include "quadrotorPropulsion/quadrotorPropulsion_initialize.h"
//}

namespace hector_quadrotor_model {

struct PropulsionParameters
{
    real_T k_m;
    real_T k_t;
    real_T CT2s;
    real_T CT1s;
    real_T CT0s;
    real_T Psi;
    real_T J_M;
    real_T R_A;
    real_T alpha_m;
    real_T beta_m;
    real_T l_m;

    PropulsionParameters()
      : k_m(0.0)
      , k_t(0.0)
      , CT2s(0.0)
      , CT1s(0.0)
      , CT0s(0.0)
      , Psi(0.0)
      , J_M(std::numeric_limits<real_T>::infinity())
      , R_A(std::numeric_limits<real_T>::infinity())
      , alpha_m(0.0)
      , beta_m(0.0)
      , l_m(0.0)
    {}
};

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
  // quadrotorPropulsion_initialize();
  propulsion_model_ = new PropulsionModel;
}

QuadrotorPropulsion::~QuadrotorPropulsion()
{
  delete propulsion_model_;
}

/*
 * quadrotorPropulsion.c
 *
 * Code generation for function 'quadrotorPropulsion'
 *
 * C source code generated on: Sun Nov  3 13:34:35 2013
 *
 */

/* Include files */
//#include "rt_nonfinite.h"
//#include "motorspeed.h"
//#include "quadrotorPropulsion.h"
//#include "quadrotorPropulsion_rtwutil.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */

/* Function Definitions */
void quadrotorPropulsion(const real_T xin[4], const real_T uin[10], const
  PropulsionParameters parameter, real_T dt, real_T y[14], real_T xpred[4])
{
  real_T v_1[4];
  int32_T i;
  real_T F_m[4];
  real_T U[4];
  real_T M_e[4];
  real_T I[4];
  real_T F[3];
  real_T b_F_m;
  real_T temp;
  real_T d0;

  /*  initialize vectors */
  for (i = 0; i < 4; i++) {
    xpred[i] = xin[i];

    /*  motorspeed */
    v_1[i] = 0.0;
  }

  memset(&y[0], 0, 14U * sizeof(real_T));
  for (i = 0; i < 4; i++) {
    F_m[i] = 0.0;
    U[i] = 0.0;
    M_e[i] = 0.0;
    I[i] = 0.0;
  }

  for (i = 0; i < 3; i++) {
    F[i] = 0.0;
  }

  /*  Input variables */
  U[0] = uin[6];
  U[1] = uin[7];
  U[2] = uin[8];
  U[3] = uin[9];

  /*  Constants */
  v_1[0] = -uin[2] + parameter.l_m * uin[4];
  v_1[1] = -uin[2] - parameter.l_m * uin[3];
  v_1[2] = -uin[2] - parameter.l_m * uin[4];
  v_1[3] = -uin[2] + parameter.l_m * uin[3];

  /*  calculate thrust for all 4 rotors */
  for (i = 0; i < 4; i++) {
    /*  if the flow speed at infinity is negative */
    if (v_1[i] < 0.0) {
      b_F_m = (parameter.CT2s * rt_powd_snf(v_1[i], 2.0) + parameter.CT1s *
               v_1[i] * xin[i]) + parameter.CT0s * rt_powd_snf(xin[i], 2.0);

      /*  if the flow speed at infinity is positive */
    } else {
      b_F_m = (-parameter.CT2s * rt_powd_snf(v_1[i], 2.0) + parameter.CT1s *
               v_1[i] * xin[i]) + parameter.CT0s * rt_powd_snf(xin[i], 2.0);
    }

    /*  sum up all rotor forces */
    /*  Identification of Roxxy2827-34 motor with 10x4.5 propeller */
    /*  temporarily used Expressions */
    temp = (U[i] * parameter.beta_m - parameter.Psi * xin[i]) / (2.0 *
      parameter.R_A);
    temp += sqrt(rt_powd_snf(temp, 2.0) + U[i] * parameter.alpha_m /
                 parameter.R_A);
    d0 = parameter.Psi * temp;

    /*  electrical torque motor 1-4 */
    /*  new version */
    /*  old version */
    /*  fx   = (Psi/R_A*(U-Psi*omega_m) - M_m)/J_M; */
    /*  A    = -(Psi^2/R_A)/J_M; */
    /*  B(1) =  Psi/(J_M*R_A); */
    /*  B(2) = -1/J_M; */
    /*  system outputs. Use euler solver to predict next time step */
    /*  predicted motor speed */
    /*  electric torque */
    /* y = [M_e I]; */
    /*  system jacobian */
    /*  A       = 1 + dt*A; */
    /*  input jacobian */
    /*  B       = A*B*dt; */
    M_e[i] = d0;
    I[i] = temp;
    xpred[i] = xin[i] + dt * (1.0 / parameter.J_M * (d0 - (parameter.k_t * b_F_m
      + parameter.k_m * xin[i])));
    F_m[i] = b_F_m;
    F[2] += b_F_m;
  }

  /*  System output, i.e. force and torque of quadrotor */
  y[0] = 0.0;
  y[1] = 0.0;
  y[2] = F[2];

  /*  torque for rotating quadrocopter around x-axis is the mechanical torque */
  y[3] = (F_m[3] - F_m[1]) * parameter.l_m;

  /*  torque for rotating quadrocopter around y-axis is the mechanical torque */
  y[4] = (F_m[0] - F_m[2]) * parameter.l_m;

  /*  torque for rotating quadrocopter around z-axis is the electrical torque */
  y[5] = ((-M_e[0] - M_e[2]) + M_e[1]) + M_e[3];

  /*  motor speeds (rad/s) */
  for (i = 0; i < 4; i++) {
    y[i + 6] = xpred[i];
  }

  /*  motor current (A) */
  for (i = 0; i < 4; i++) {
    y[i + 10] = I[i];
  }

  /*  M_e(1:4) / Psi; */
}

/* End of code generation (quadrotorPropulsion.c) */

inline void QuadrotorPropulsion::f(const double xin[4], const double uin[10], double dt, double y[14], double xpred[4]) const
{
  quadrotorPropulsion(xin, uin, propulsion_model_->parameters_, dt, y, xpred);
}

bool QuadrotorPropulsion::configure(const ros::NodeHandle &param)
{
  // get model parameters
  if (!param.getParam("k_m",     propulsion_model_->parameters_.k_m)) return false;
  if (!param.getParam("k_t",     propulsion_model_->parameters_.k_t)) return false;
  if (!param.getParam("CT0s",    propulsion_model_->parameters_.CT0s)) return false;
  if (!param.getParam("CT1s",    propulsion_model_->parameters_.CT1s)) return false;
  if (!param.getParam("CT2s",    propulsion_model_->parameters_.CT2s)) return false;
  if (!param.getParam("J_M",     propulsion_model_->parameters_.J_M)) return false;
  if (!param.getParam("l_m",     propulsion_model_->parameters_.l_m)) return false;
  if (!param.getParam("Psi",     propulsion_model_->parameters_.Psi)) return false;
  if (!param.getParam("R_A",     propulsion_model_->parameters_.R_A)) return false;
  if (!param.getParam("alpha_m", propulsion_model_->parameters_.alpha_m)) return false;
  if (!param.getParam("beta_m",  propulsion_model_->parameters_.beta_m)) return false;

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

  return true;
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

  // We limit the input velocities to +-100. Required for numeric stability in case of collisions,
  // where velocities returned by Gazebo can be very high.
  limit(boost::iterator_range<boost::array<real_T,10>::iterator>(&(propulsion_model_->u[0]), &(propulsion_model_->u[6])), -100.0, 100.0);
}

void QuadrotorPropulsion::addCommandToQueue(const hector_uav_msgs::MotorCommandConstPtr& command)
{
  hector_uav_msgs::MotorPWMPtr pwm(new hector_uav_msgs::MotorPWM);
  pwm->header = command->header;
  pwm->pwm.resize(command->voltage.size());
  for(std::size_t i = 0; i < command->voltage.size(); ++i) {
    int temp = round(command->voltage[i] / supply_.voltage[0] * 255.0);
    if (temp < 0)
      pwm->pwm[i] = 0;
    else if (temp > 255)
      pwm->pwm[i] = 255;
    else
      pwm->pwm[i] = temp;
  }
  addPWMToQueue(pwm);
}

void QuadrotorPropulsion::addPWMToQueue(const hector_uav_msgs::MotorPWMConstPtr& pwm)
{
  boost::mutex::scoped_lock lock(command_queue_mutex_);

  if (!motor_status_.on) {
    ROS_WARN_NAMED("quadrotor_propulsion", "Received new motor command. Enabled motors.");
    engage();
  }

  ROS_DEBUG_STREAM_NAMED("quadrotor_propulsion", "Received motor command valid at " << pwm->header.stamp);
  command_queue_.push(pwm);
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

    while(!command_queue_.empty()) {
      hector_uav_msgs::MotorPWMConstPtr new_motor_voltage = command_queue_.front();
      ros::Time new_time = new_motor_voltage->header.stamp;

      if (new_time.isZero() || (new_time >= min && new_time <= max)) {
        setVoltage(*new_motor_voltage);
        command_queue_.pop();
        new_command = true;

      // new motor command is too old:
      } else if (new_time < min) {
        ROS_DEBUG_NAMED("quadrotor_propulsion", "Command received was %fs too old. Discarding.", (new_time - timestamp).toSec());
        command_queue_.pop();

      // new motor command is too new:
      } else {
        break;
      }
    }

    if (command_queue_.empty() && !new_command) {
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

  if (new_command) {
      ROS_DEBUG_STREAM_NAMED("quadrotor_propulsion", "Using motor command valid at t = " << last_command_time_.toSec() << "s for simulation step at t = " << timestamp.toSec() << "s (dt = " << (timestamp - last_command_time_).toSec() << "s)");
  }

  return new_command;
}

void QuadrotorPropulsion::update(double dt)
{
  if (dt <= 0.0) return;

  ROS_DEBUG_STREAM_NAMED("quadrotor_propulsion", "propulsion.twist:   " << PrintVector<double>(propulsion_model_->u.begin(), propulsion_model_->u.begin() + 6));
  ROS_DEBUG_STREAM_NAMED("quadrotor_propulsion", "propulsion.voltage: " << PrintVector<double>(propulsion_model_->u.begin() + 6, propulsion_model_->u.begin() + 10));
  ROS_DEBUG_STREAM_NAMED("quadrotor_propulsion", "propulsion.x_prior: " << PrintVector<double>(propulsion_model_->x.begin(), propulsion_model_->x.end()));

  checknan(propulsion_model_->u, "propulsion model input");
  checknan(propulsion_model_->x, "propulsion model state");

  // update propulsion model
  f(propulsion_model_->x.data(), propulsion_model_->u.data(), dt, propulsion_model_->y.data(), propulsion_model_->x_pred.data());
  propulsion_model_->x = propulsion_model_->x_pred;

  ROS_DEBUG_STREAM_NAMED("quadrotor_propulsion", "propulsion.x_post:  " << PrintVector<double>(propulsion_model_->x.begin(), propulsion_model_->x.end()));
  ROS_DEBUG_STREAM_NAMED("quadrotor_propulsion", "propulsion.force:   " << PrintVector<double>(propulsion_model_->y.begin() + 0, propulsion_model_->y.begin() + 3) << " " <<
                                                 "propulsion.torque:  " << PrintVector<double>(propulsion_model_->y.begin() + 3, propulsion_model_->y.begin() + 6));

  checknan(propulsion_model_->y, "propulsion model output");

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
