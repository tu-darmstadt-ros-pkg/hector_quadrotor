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

#ifndef HECTOR_QUADROTOR_GAZEBO_PLUGINS_QUADROTOR_PROPULSION_H
#define HECTOR_QUADROTOR_GAZEBO_PLUGINS_QUADROTOR_PROPULSION_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/math/Vector3.hh>

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <hector_quadrotor_model/quadrotor_propulsion.h>
#include <hector_gazebo_plugins/update_timer.h>

#include <boost/thread.hpp>

namespace gazebo
{

class GazeboQuadrotorPropulsion : public ModelPlugin
{
public:
  GazeboQuadrotorPropulsion();
  virtual ~GazeboQuadrotorPropulsion();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Reset();
  virtual void Update();

private:
  /// \brief The parent World
  physics::WorldPtr world;

  /// \brief The link referred to by this plugin
  physics::LinkPtr link;

  hector_quadrotor_model::QuadrotorPropulsion model_;

  ros::NodeHandle* node_handle_;
  ros::CallbackQueue callback_queue_;
  boost::thread callback_queue_thread_;
  void QueueThread();

  ros::Publisher trigger_publisher_;
  ros::Subscriber command_subscriber_;
  ros::Subscriber pwm_subscriber_;
  ros::Publisher wrench_publisher_;
  ros::Publisher supply_publisher_;
  ros::Publisher motor_status_publisher_;

  std::string body_name_;
  std::string namespace_;
  std::string param_namespace_;
  std::string trigger_topic_;
  std::string command_topic_;
  std::string pwm_topic_;
  std::string wrench_topic_;
  std::string supply_topic_;
  std::string status_topic_;
  std::string frame_id_;
  ros::Duration control_delay_;
  ros::Duration control_tolerance_;

  common::Time last_time_;
  common::Time last_trigger_time_;
  common::Time last_motor_status_time_;
  common::Time last_supply_time_;

  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;

  UpdateTimer controlTimer;
  UpdateTimer motorStatusTimer;
};

}

#endif // HECTOR_QUADROTOR_GAZEBO_PLUGINS_QUADROTOR_PROPULSION_H
