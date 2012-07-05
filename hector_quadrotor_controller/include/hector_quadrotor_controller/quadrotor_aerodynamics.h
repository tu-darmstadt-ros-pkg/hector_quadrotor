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

#ifndef HECTOR_GAZEBO_PLUGINS_QUADROTOR_PROPULSION_H
#define HECTOR_GAZEBO_PLUGINS_QUADROTOR_PROPULSION_H

#include <gazebo/Controller.hh>
#include <gazebo/Entity.hh>
#include <gazebo/Model.hh>
#include <gazebo/Body.hh>
#include <gazebo/Param.hh>
#include <gazebo/Time.hh>

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <hector_uav_msgs/MotorStatus.h>
#include <hector_uav_msgs/MotorPWM.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Vector3.h>

#include <boost/thread.hpp>
#include <list>

namespace gazebo
{

class GazeboQuadrotorAerodynamics : public Controller
{
public:
  GazeboQuadrotorAerodynamics(Entity *parent);
  virtual ~GazeboQuadrotorAerodynamics();

protected:
  virtual void LoadChild(XMLConfigNode *node);
  virtual void InitChild();
  virtual void UpdateChild();
  virtual void FiniChild();
  virtual void ResetChild();

private:
  Model *parent_;
  Body *body_;

  ros::NodeHandle* node_handle_;
  ros::CallbackQueue callback_queue_;
  boost::thread callback_queue_thread_;
  void QueueThread();

  ros::Subscriber voltage_subscriber_;
  ros::Subscriber wind_subscriber_;
  ros::Publisher wrench_publisher_;
  ros::Publisher motor_status_publisher_;

  hector_uav_msgs::MotorPWMConstPtr motor_voltage_;
  std::list<hector_uav_msgs::MotorPWMConstPtr> new_motor_voltages_;
  geometry_msgs::Wrench wrench_;
  hector_uav_msgs::MotorStatus motor_status_;
  geometry_msgs::Vector3 wind_;
  void CommandCallback(const hector_uav_msgs::MotorPWMConstPtr&);
  void WindCallback(const geometry_msgs::Vector3ConstPtr&);

  Vector3 velocity, rate;

  ParamT<std::string> *body_name_;
  ParamT<std::string> *namespace_;
  ParamT<std::string> *param_namespace_;
  ParamT<Time> *control_rate_;
  ParamT<std::string> *voltage_topic_;
  ParamT<std::string> *wind_topic_;
  ParamT<std::string> *wrench_topic_;
  ParamT<std::string> *status_topic_;
  ParamT<Time> *control_delay_param_;
  Time control_delay_;
  Time control_tolerance_;


  class PropulsionModel;
  PropulsionModel *propulsion_model_;

  class DragModel;
  DragModel *drag_model_;

  Time control_period_;
  Time last_control_time_;
  Time last_motor_status_time_;

  boost::condition command_condition_;
  boost::mutex command_mutex_;
};

}

#endif // HECTOR_GAZEBO_PLUGINS_QUADROTOR_PROPULSION_H
