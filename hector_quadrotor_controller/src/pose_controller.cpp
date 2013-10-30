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

#include <hector_quadrotor_controller/pose_controller.h>
#include <limits>

namespace hector_quadrotor_controller {

bool PoseController::init(QuadrotorInterface *interface, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
{
  // get interface handles
  pose_input_   = interface->addInput<PoseCommandHandle>("pose");
  twist_input_  = interface->addInput<TwistCommandHandle>("pose/twist");
  twist_output_ = interface->addOutput<TwistCommandHandle>("twist");
  interface->claim(twist_output_->getName());

  // subscribe to commanded pose and velocity
  ros::SubscribeOptions pose_subscribe_options = ros::SubscribeOptions::create<geometry_msgs::PoseStamped>(
    "command/pose", 1,
    boost::bind(&PoseController::poseCommandCallback, this, _1),
    ros::VoidConstPtr(), 0 // &callback_queue_
  );
  pose_subscriber_ = root_nh.subscribe(pose_subscribe_options);
  ros::SubscribeOptions twist_subscribe_options = ros::SubscribeOptions::create<geometry_msgs::TwistStamped>(
    "command/twist", 1,
    boost::bind(&PoseController::twistCommandCallback, this, _1),
    ros::VoidConstPtr(), 0 // &callback_queue_
  );
  twist_subscriber_ = root_nh.subscribe(twist_subscribe_options);

  // get parameters
  initParameters(parameters_.xy,  ros::NodeHandle(controller_nh, "xy"));
  initParameters(parameters_.z,   ros::NodeHandle(controller_nh, "z"));
  initParameters(parameters_.yaw, ros::NodeHandle(controller_nh, "yaw"));

  return true;
}

void PoseController::initParameters(parameters &param, const ros::NodeHandle &param_nh)
{
  param_nh.getParam("enabled", param.enabled = true);
  param_nh.getParam("k_p", param.k_p = 0.0);
  param_nh.getParam("k_i", param.k_i = 0.0);
  param_nh.getParam("k_d", param.k_d = 0.0);
  param_nh.getParam("limit_i", param.limit_i = std::numeric_limits<double>::quiet_NaN());
  param_nh.getParam("limit_output", param.limit_output = std::numeric_limits<double>::quiet_NaN());
}

void PoseController::reset()
{
  state_.x   = state();
  state_.y   = state();
  state_.x   = state();
  state_.yaw = state();
}

PoseController::state::state()
  : p(std::numeric_limits<double>::quiet_NaN())
  , i(0.0)
  , d(std::numeric_limits<double>::quiet_NaN())
  , derivative(std::numeric_limits<double>::quiet_NaN())
{
}

void PoseController::poseCommandCallback(const geometry_msgs::PoseStampedConstPtr& command)
{
  pose_command_ = *command;
  if (!(pose_input_->connected())) *pose_input_ = &(pose_command_.pose);

  ros::Time start_time = command->header.stamp;
  if (start_time.isZero()) start_time = ros::Time::now();
  if (!isRunning()) this->startRequest(start_time);
}

void PoseController::twistCommandCallback(const geometry_msgs::TwistStampedConstPtr& command)
{
  twist_command_ = *command;
  if (!(twist_input_->connected())) *twist_input_ = &(twist_command_.twist);

  ros::Time start_time = command->header.stamp;
  if (start_time.isZero()) start_time = ros::Time::now();
  if (!isRunning()) this->startRequest(start_time);
}

void PoseController::starting(const ros::Time &time)
{
  reset();
  twist_output_->start();
}

void PoseController::stopping(const ros::Time &time)
{
  twist_output_->stop();
}

void PoseController::update(const ros::Time& time, const ros::Duration& period)
{
  Twist output;

  // execute available callbacks in the callback queue (is this real-time safe?)
//  callback_queue_.callAvailable();

  // return if no pose command is available
  if (!(pose_input_->connected())) {
    return;
  }

  // check command timeout
  // TODO

  // control horizontal position
  double error_n, error_w;
  HorizontalPositionCommandHandle(*pose_input_).getError(error_n, error_w);
  double command_n = updatePID(error_n, twist_input_->twist().linear.x, state_.x, parameters_.xy, period);
  double command_w = updatePID(error_w, twist_input_->twist().linear.y, state_.y, parameters_.xy, period);

  // transform to body coordinates (yaw only)
  double yaw = pose_input_->getYaw();
  output.linear.x =  cos(yaw) * command_n + sin(yaw) * command_w;
  output.linear.y = -sin(yaw) * command_n + cos(yaw) * command_w;

  // control height
  output.linear.z = updatePID(HeightCommandHandle(*pose_input_).getError(), twist_input_->twist().linear.z, state_.z, parameters_.z, period);

  // control yaw angle
  output.angular.z = updatePID(HeadingCommandHandle(*pose_input_).getError(), twist_input_->twist().angular.z, state_.yaw, parameters_.yaw, period);

  // add twist command if available
  if (twist_input_->connected())
  {
    output.linear.x  += twist_input_->getCommand().linear.x;
    output.linear.y  += twist_input_->getCommand().linear.y;
    output.linear.z  += twist_input_->getCommand().linear.z;
    output.angular.x += twist_input_->getCommand().angular.x;
    output.angular.y += twist_input_->getCommand().angular.y;
    output.angular.z += twist_input_->getCommand().angular.z;
  }

  // set twist output
  twist_output_->setCommand(output);
}

template <typename T> inline T& checknan(T& value)
{
  if (std::isnan(value)) value = T();
  return value;
}

double PoseController::updatePID(double error, double derivative, state &state, const parameters &param, const ros::Duration& period)
{
  if (!param.enabled) return 0.0;
  double dt = period.toSec();

  // integral error
  state.i += error * dt;
  if (param.limit_i > 0.0)
  {
    if (state.i >  param.limit_i) state.i =  param.limit_i;
    if (state.i < -param.limit_i) state.i = -param.limit_i;
  }

  // differential error
  if (dt > 0.0 && !std::isnan(state.p) && !std::isnan(state.derivative)) {
    state.d = (error - state.p) / dt + state.derivative - derivative;
  } else {
    state.d = -derivative;
  }
  state.derivative = derivative;

  // proportional error
  state.p = error;

  // calculate output...
  double output = param.k_p * state.p + param.k_i * state.i + param.k_d * state.d;
  int antiwindup = 0;
  if (param.limit_output > 0.0)
  {
    if (output >  param.limit_i) { output =  param.limit_i; antiwindup =  1; }
    if (output < -param.limit_i) { output = -param.limit_i; antiwindup = -1; }
  }
  if (antiwindup && (error * dt * antiwindup > 0.0)) state.i -= error * dt;

  checknan(output);
  return output;
}

} // namespace hector_quadrotor_controller

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hector_quadrotor_controller::PoseController, controller_interface::ControllerBase)
