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

#ifndef HECTOR_QUADROTOR_CONTROLLER_QUADROTOR_HARDWARE_GAZEBO_H
#define HECTOR_QUADROTOR_CONTROLLER_QUADROTOR_HARDWARE_GAZEBO_H

#include <ros/node_handle.h>
#include <gazebo_ros_control/robot_hw_sim.h>

#include <hector_quadrotor_interface/quadrotor_interface.h>
#include <hector_quadrotor_interface/helpers.h>
#include <hector_quadrotor_interface/limiters.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <hector_uav_msgs/MotorStatus.h>
#include <hector_uav_msgs/EnableMotors.h>

namespace hector_quadrotor_controller_gazebo
{

using namespace hector_quadrotor_interface;

class QuadrotorHardwareSim : public gazebo_ros_control::RobotHWSim
{
public:
  QuadrotorHardwareSim();
  virtual ~QuadrotorHardwareSim();

  virtual bool initSim(
      const std::string &robot_namespace,
      ros::NodeHandle model_nh,
      gazebo::physics::ModelPtr parent_model,
      const urdf::Model *const urdf_model,
      std::vector<transmission_interface::TransmissionInfo> transmissions);

  virtual void readSim(ros::Time time, ros::Duration period);

  virtual void writeSim(ros::Time time, ros::Duration period);

  bool enableMotorsCallback(hector_uav_msgs::EnableMotors::Request &req, hector_uav_msgs::EnableMotors::Response &res);

private:
  bool enableMotors(bool enable);

  std_msgs::Header header_;
  geometry_msgs::Pose pose_;
  geometry_msgs::Twist twist_;
  geometry_msgs::Accel acceleration_;
  sensor_msgs::Imu imu_;
  hector_uav_msgs::MotorStatus motor_status_;

  QuadrotorInterface interface_;

  WrenchCommandHandlePtr wrench_output_;
  MotorCommandHandlePtr motor_output_;

  hector_quadrotor_interface::WrenchLimiter wrench_limiter_;
  std::string base_link_frame_, world_frame_;

  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr link_;
  gazebo::physics::PhysicsEnginePtr physics_;

  gazebo::math::Pose gz_pose_;
  gazebo::math::Vector3 gz_velocity_, gz_acceleration_, gz_angular_velocity_, gz_angular_acceleration_;

  boost::shared_ptr<hector_quadrotor_interface::ImuSubscriberHelper> imu_sub_helper_;
  boost::shared_ptr<hector_quadrotor_interface::OdomSubscriberHelper> odom_sub_helper_;

  ros::Publisher wrench_command_publisher_;
  ros::Publisher motor_command_publisher_;
  ros::ServiceServer enable_motors_server_;
};

} // namespace hector_quadrotor_controller_gazebo

#endif // HECTOR_QUADROTOR_CONTROLLER_QUADROTOR_HARDWARE_GAZEBO_H
