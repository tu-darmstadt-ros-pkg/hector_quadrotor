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

#include <hector_quadrotor_controller/quadrotor_interface.h>

#include <cmath>

namespace hector_quadrotor_controller {

//QuadrotorInterface::QuadrotorInterface()
//  : pose_(0)
//  , twist_(0)
//  , imu_(0)
//  , motor_status_(0)
//  , pose_command_(0)
//  , twist_command_(0)
//  , wrench_command_(0)
//  , motor_command_(0)
//  , trajectory_command_(0)
//{}

//QuadrotorInterface::QuadrotorInterface(
//    geometry_msgs::Pose &pose,
//    geometry_msgs::Twist &twist,
//    sensor_msgs::Imu &imu,
//    hector_uav_msgs::MotorStatus &motor_status,
//    geometry_msgs::Pose &pose_command,
//    geometry_msgs::Twist &twist_command,
//    geometry_msgs::Wrench &wrench_command,
//    hector_uav_msgs::MotorCommand &motor_command,
//    nav_msgs::Path &trajectory_command
//  )
//  : pose_(&pose)
//  , twist_(&twist)
//  , imu_(&imu)
//  , motor_status_(&motor_status)
//  , pose_command_(&pose_command)
//  , twist_command_(&twist_command)
//  , wrench_command_(&wrench_command)
//  , motor_command_(&motor_command)
//  , trajectory_command_(&trajectory_command)
//{
//}

QuadrotorInterface::QuadrotorInterface()
{}

QuadrotorInterface::~QuadrotorInterface()
{}

void PoseHandle::getEulerRPY(double &roll, double &pitch, double &yaw) const
{
  const geometry_msgs::Quaternion::_w_type& w = pose().orientation.w;
  const geometry_msgs::Quaternion::_x_type& x = pose().orientation.x;
  const geometry_msgs::Quaternion::_y_type& y = pose().orientation.y;
  const geometry_msgs::Quaternion::_z_type& z = pose().orientation.z;
  roll  =  atan2(2.*y*z + 2.*w*x, z*z - y*y - x*x + w*w);
  pitch = -asin(2.*x*z - 2.*w*y);
  yaw   =  atan2(2.*x*y + 2.*w*z, x*x + w*w - z*z - y*y);
}

double PoseHandle::getYaw() const
{
  const geometry_msgs::Quaternion::_w_type& w = pose().orientation.w;
  const geometry_msgs::Quaternion::_x_type& x = pose().orientation.x;
  const geometry_msgs::Quaternion::_y_type& y = pose().orientation.y;
  const geometry_msgs::Quaternion::_z_type& z = pose().orientation.z;
  return atan2(2.*x*y + 2.*w*z, x*x + w*w - z*z - y*y);
}

void HorizontalPositionCommandHandle::getError(double &x, double &y) const
{
  getCommand(x, y);
  x -= pose().position.x;
  y -= pose().position.y;
}

double HeightCommandHandle::getError() const
{
  return getCommand() - pose().position.z;
}

double HeadingCommandHandle::getCommand() const {
  geometry_msgs::Pose *command = interface_->getPoseCommand();
  return atan2(command->orientation.z, command->orientation.w) * 2.;
}

double HeadingCommandHandle::getError() const {
  static const double M_2PI = 2.0 * M_PI;
  double error = getCommand() - getYaw() + M_PI;
  error -= floor(error / M_2PI) * M_2PI;
  return error - M_PI;
}

} // namespace hector_quadrotor_controller
