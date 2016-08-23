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

#ifndef HECTOR_QUADROTOR_MODEL_QUADROTOR_PROPULSION_H
#define HECTOR_QUADROTOR_MODEL_QUADROTOR_PROPULSION_H

#include <hector_uav_msgs/Supply.h>
#include <hector_uav_msgs/MotorStatus.h>
#include <hector_uav_msgs/MotorCommand.h>
#include <hector_uav_msgs/MotorPWM.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Wrench.h>

#include <ros/forwards.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <queue>

#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>

namespace hector_quadrotor_model
{

class QuadrotorPropulsion
{
public:
  QuadrotorPropulsion();
  ~QuadrotorPropulsion();

  bool configure(const ros::NodeHandle &param = ros::NodeHandle("~"));
  void reset();
  void update(double dt);

  void engage();
  void shutdown();

  void setTwist(const geometry_msgs::Twist& twist);
  void setVoltage(const hector_uav_msgs::MotorPWM& command);

  const geometry_msgs::Wrench& getWrench() const { return wrench_; }
  const hector_uav_msgs::Supply& getSupply() const { return supply_; }
  const hector_uav_msgs::MotorStatus& getMotorStatus() const { return motor_status_; }

  void addCommandToQueue(const hector_uav_msgs::MotorCommandConstPtr& command);
  void addPWMToQueue(const hector_uav_msgs::MotorPWMConstPtr& pwm);
  bool processQueue(const ros::Time& timestamp, const ros::Duration& tolerance = ros::Duration(), const ros::Duration& delay = ros::Duration(), const ros::WallDuration &wait = ros::WallDuration(), ros::CallbackQueue *callback_queue = 0);

  void f(const double xin[4], const double uin[10], double dt, double y[14], double xpred[4]) const;

  void setInitialSupplyVoltage(double voltage) { initial_voltage_ = voltage; }

private:
  geometry_msgs::Wrench wrench_;
  hector_uav_msgs::Supply supply_;
  hector_uav_msgs::MotorStatus motor_status_;
  ros::Time last_command_time_;

  double initial_voltage_;

  std::queue<hector_uav_msgs::MotorPWMConstPtr> command_queue_;
  boost::mutex command_queue_mutex_;
  boost::condition command_condition_;

  boost::mutex mutex_;

  class PropulsionModel;
  PropulsionModel *propulsion_model_;
};

}

#endif // HECTOR_QUADROTOR_MODEL_QUADROTOR_PROPULSION_H
