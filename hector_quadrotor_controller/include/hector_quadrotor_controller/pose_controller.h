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

#ifndef HECTOR_QUADROTOR_CONTROLLER_POSE_CONTROLLER_H
#define HECTOR_QUADROTOR_CONTROLLER_POSE_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hector_quadrotor_controller/quadrotor_interface.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <ros/subscriber.h>

namespace hector_quadrotor_controller {

using namespace controller_interface;

class PoseController : public controller_interface::Controller<QuadrotorInterface>
{
public:
  bool init(QuadrotorInterface* interface, ros::NodeHandle& root_nh, ros::NodeHandle &controller_nh);
  void reset();

  void commandCallback(const geometry_msgs::Pose& command);

  void starting(const ros::Time& time);
  void update(const ros::Time& time, const ros::Duration& period);
  void stopping(const ros::Time& time);

private:
  PoseCommandHandle pose_;
  VelocityCommandHandle velocity_;

  struct parameters {
    bool enabled;
    double k_p;
    double k_i;
    double k_d;
    double limit_i;
    double limit_output;
  };
  void initParameters(parameters &param, const ros::NodeHandle &param_nh);

  struct {
    parameters xy;
    parameters z;
    parameters yaw;
  } parameters_;

  struct state {
    state();
    double p, i, d;
    double derivative;
  };

  struct {
    state x;
    state y;
    state z;
    state yaw;
  } state_;

  double updatePID(double error, double derivative, state &state, const parameters &param, const ros::Duration& period);

  ros::Time start_time_;
  ros::Subscriber subscriber_;
};

} // namespace hector_quadrotor_controller

#endif // HECTOR_QUADROTOR_CONTROLLER_POSE_CONTROLLER_H
