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

#ifndef HECTOR_QUADROTOR_CONTROLLER_QUADROTOR_HARDWARE_GAZEBO_H
#define HECTOR_QUADROTOR_CONTROLLER_QUADROTOR_HARDWARE_GAZEBO_H

#include <gazebo_ros_control/robot_hw_sim.h>
#include <hector_quadrotor_controller/quadrotor_interface.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <hector_uav_msgs/MotorStatus.h>

#include <ros/node_handle.h>
#include <ros/callback_queue.h>

namespace hector_quadrotor_controller_gazebo {

using namespace hector_quadrotor_controller;
using namespace hardware_interface;
using namespace gazebo_ros_control;

class QuadrotorHardwareSim : public RobotHWSim, public QuadrotorInterface
{
public:
  QuadrotorHardwareSim();
  virtual ~QuadrotorHardwareSim();

  virtual const ros::Time &getTimestamp() { return header_.stamp; }

  virtual PoseHandlePtr getPose()                 { return PoseHandlePtr(new PoseHandle(this, &pose_)); }
  virtual TwistHandlePtr getTwist()               { return TwistHandlePtr(new TwistHandle(this, &twist_)); }
  virtual AccelerationHandlePtr getAcceleration() { return AccelerationHandlePtr(new AccelerationHandle(this, &acceleration_)); }
  virtual ImuHandlePtr getSensorImu()             { return ImuHandlePtr(new ImuHandle(this, &imu_)); }
  virtual MotorStatusHandlePtr getMotorStatus()   { return MotorStatusHandlePtr(new MotorStatusHandle(this, &motor_status_)); }

  virtual bool getMassAndInertia(double &mass, double inertia[3]);

  virtual bool initSim(
      const std::string& robot_namespace,
      ros::NodeHandle model_nh,
      gazebo::physics::ModelPtr parent_model,
      const urdf::Model *const urdf_model,
      std::vector<transmission_interface::TransmissionInfo> transmissions);

  virtual void readSim(ros::Time time, ros::Duration period);

  virtual void writeSim(ros::Time time, ros::Duration period);

private:
  void stateCallback(const nav_msgs::OdometryConstPtr &state);
  void imuCallback(const sensor_msgs::ImuConstPtr &imu);
  void motorStatusCallback(const hector_uav_msgs::MotorStatusConstPtr &motor_status);

protected:
  std_msgs::Header header_;
  Pose pose_;
  Twist twist_;
  Vector3 acceleration_;
  Imu imu_;
  MotorStatus motor_status_;
  std::string base_link_frame_, world_frame_;

  WrenchCommandHandlePtr wrench_output_;
  MotorCommandHandlePtr motor_output_;

  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr link_;
  gazebo::physics::PhysicsEnginePtr physics_;

  gazebo::math::Pose gz_pose_;
  gazebo::math::Vector3 gz_velocity_, gz_acceleration_, gz_angular_velocity_;

  ros::CallbackQueue callback_queue_;
  ros::Subscriber subscriber_state_;
  ros::Subscriber subscriber_imu_;
  ros::Subscriber subscriber_motor_status_;
  ros::Publisher publisher_wrench_command_;
  ros::Publisher publisher_motor_command_;
};

} // namespace hector_quadrotor_controller_gazebo

#endif // HECTOR_QUADROTOR_CONTROLLER_QUADROTOR_HARDWARE_GAZEBO_H
