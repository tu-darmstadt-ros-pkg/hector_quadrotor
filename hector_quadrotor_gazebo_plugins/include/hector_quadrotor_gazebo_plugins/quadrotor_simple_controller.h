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

#ifndef HECTOR_GAZEBO_PLUGINS_QUADROTOR_SIMPLE_CONTROLLER_H
#define HECTOR_GAZEBO_PLUGINS_QUADROTOR_SIMPLE_CONTROLLER_H

#include <gazebo/Controller.hh>
#include <gazebo/Entity.hh>
#include <gazebo/Model.hh>
#include <gazebo/Body.hh>
#include <gazebo/Param.hh>
#include <gazebo/Time.hh>

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>

namespace gazebo
{

class GazeboQuadrotorSimpleController : public Controller
{
public:
  GazeboQuadrotorSimpleController(Entity *parent);
  virtual ~GazeboQuadrotorSimpleController();

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
  ros::Subscriber velocity_subscriber_;
  ros::Subscriber wrench_subscriber_;

  // void CallbackQueueThread();
  // boost::mutex lock_;
  // boost::thread callback_queue_thread_;

  geometry_msgs::Twist velocity_command_;
  geometry_msgs::Wrench wrench_command_;
  void VelocityCallback(const geometry_msgs::TwistConstPtr&);
  void WrenchCallback(const geometry_msgs::WrenchConstPtr&);

  ParamT<std::string> *body_name_param_;
  std::string body_name_;
  ParamT<std::string> *namespace_param_;
  std::string namespace_;
  ParamT<std::string> *velocity_topic_param_;
  std::string velocity_topic_;
  ParamT<std::string> *wrench_topic_param_;
  std::string wrench_topic_;
  ParamT<double> *max_force_param_;
  double max_force_;

  class PIDController {
  public:
    PIDController(std::vector<Param*>& parameters);
    virtual ~PIDController();
    virtual void LoadChild(XMLConfigNode *node);

    ParamT<double> *gain_p_param_;
    double gain_p;
    ParamT<double> *gain_i_param_;
    double gain_i;
    ParamT<double> *gain_d_param_;
    double gain_d;
    ParamT<double> *time_constant_param_;
    double time_constant;
    ParamT<double> *limit_param_;
    double limit;

    double input;
    double dinput;
    double output;
    double p, i, d;

    double update(double input, double x, double dx, double dt);
    void reset();
  };

  struct Controllers {
    Controllers(std::vector<Param*>& parameters) : roll(parameters), pitch(parameters), yaw(parameters), velocity_x(parameters), velocity_y(parameters), velocity_z(parameters) {}
    PIDController roll;
    PIDController pitch;
    PIDController yaw;
    PIDController velocity_x;
    PIDController velocity_y;
    PIDController velocity_z;
  } controllers_;

  Vector3 inertia;
  double mass;
};

}

#endif // HECTOR_GAZEBO_PLUGINS_QUADROTOR_SIMPLE_CONTROLLER_H
