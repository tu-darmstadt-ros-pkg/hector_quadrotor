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

#include <hector_quadrotor_controller_gazebo/quadrotor_hardware_gazebo.h>

#include <geometry_msgs/WrenchStamped.h>

namespace hector_quadrotor_controller_gazebo
{

QuadrotorHardwareSim::QuadrotorHardwareSim()
{
  this->registerInterface(&interface_);
  interface_.registerAccel(&acceleration_);
  interface_.registerPose(&pose_);
  interface_.registerMotorStatus(&motor_status_);
  interface_.registerSensorImu(&imu_);
  interface_.registerTwist(&twist_);

  wrench_output_ = interface_.addInput<WrenchCommandHandle>("wrench");
  motor_output_ = interface_.addInput<MotorCommandHandle>("motor");
}

QuadrotorHardwareSim::~QuadrotorHardwareSim()
{

}

bool QuadrotorHardwareSim::initSim(
    const std::string &robot_namespace,
    ros::NodeHandle model_nh,
    gazebo::physics::ModelPtr parent_model,
    const urdf::Model *const urdf_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  // store parent model pointer
  model_ = parent_model;
  link_ = model_->GetLink();
#if (GAZEBO_MAJOR_VERSION >= 8)
  physics_ = model_->GetWorld()->Physics();
#else
  physics_ = model_->GetWorld()->GetPhysicsEngine();
#endif

  model_nh.param<std::string>("world_frame", world_frame_, "world");
  model_nh.param<std::string>("base_link_frame", base_link_frame_, "base_link");

  // subscribe state
  std::string state_topic;
  model_nh.getParam("state_topic", state_topic);
  if (!state_topic.empty())
  {
    odom_sub_helper_ = boost::make_shared<OdomSubscriberHelper>(model_nh, state_topic, boost::ref(pose_),
                                                                boost::ref(twist_), boost::ref(acceleration_),
                                                                boost::ref(header_));
    gzlog << "[hector_quadrotor_controller_gazebo] Using topic '" << state_topic << "' as state input for control" <<
    std::endl;
  }
  else
  {
    gzlog << "[hector_quadrotor_controller_gazebo] Using ground truth from Gazebo as state input for control" <<
    std::endl;
  }

  // subscribe imu
  std::string imu_topic;
  model_nh.getParam("imu_topic", imu_topic);
  if (!imu_topic.empty())
  {
    imu_sub_helper_ = boost::make_shared<ImuSubscriberHelper>(model_nh, imu_topic, boost::ref(imu_));
    gzlog << "[hector_quadrotor_controller_gazebo] Using topic '" << imu_topic << "' as imu input for control" <<
    std::endl;
  }
  else
  {
    gzlog << "[hector_quadrotor_controller_gazebo] Using ground truth from Gazebo as imu input for control" <<
    std::endl;
  }

  motor_status_.on = true;
  motor_status_.header.frame_id = base_link_frame_;

  enable_motors_server_ = model_nh.advertiseService("enable_motors", &QuadrotorHardwareSim::enableMotorsCallback, this);

  wrench_limiter_.init(model_nh, "wrench_limits");

  wrench_command_publisher_ = model_nh.advertise<geometry_msgs::WrenchStamped>("command/wrench", 1);
  motor_command_publisher_ = model_nh.advertise<geometry_msgs::WrenchStamped>("command/motor", 1);

  return true;
}

void QuadrotorHardwareSim::readSim(ros::Time time, ros::Duration period)
{
  // read state from Gazebo
  const double acceleration_time_constant = 0.1;
#if (GAZEBO_MAJOR_VERSION >= 8)
  gz_acceleration_ = ((link_->WorldLinearVel() - gz_velocity_) + acceleration_time_constant * gz_acceleration_) /
                     (period.toSec() + acceleration_time_constant);
  gz_angular_acceleration_ =
      ((link_->WorldLinearVel() - gz_angular_velocity_) + acceleration_time_constant * gz_angular_acceleration_) /
      (period.toSec() + acceleration_time_constant);

  gz_pose_ = link_->WorldPose();
  gz_velocity_ = link_->WorldLinearVel();
  gz_angular_velocity_ = link_->WorldAngularVel();
#else
  gz_acceleration_ = ((link_->GetWorldLinearVel() - gz_velocity_) + acceleration_time_constant * gz_acceleration_) /
                     (period.toSec() + acceleration_time_constant);
  gz_angular_acceleration_ =
      ((link_->GetWorldLinearVel() - gz_angular_velocity_) + acceleration_time_constant * gz_angular_acceleration_) /
      (period.toSec() + acceleration_time_constant);

  gz_pose_ = link_->GetWorldPose();
  gz_velocity_ = link_->GetWorldLinearVel();
  gz_angular_velocity_ = link_->GetWorldAngularVel();
#endif

  // Use when Gazebo patches accel = 0 bug
//    gz_acceleration_ = link_->GetWorldLinearAccel();
//    gz_angular_acceleration_ = link_->GetWorldAngularAccel();

  if (!odom_sub_helper_)
  {
    header_.frame_id = world_frame_;
    header_.stamp = time;
#if (GAZEBO_MAJOR_VERSION >= 8)
    pose_.position.x = gz_pose_.Pos().X();
    pose_.position.y = gz_pose_.Pos().Y();
    pose_.position.z = gz_pose_.Pos().Z();
    pose_.orientation.w = gz_pose_.Rot().W();
    pose_.orientation.x = gz_pose_.Rot().X();
    pose_.orientation.y = gz_pose_.Rot().Y();
    pose_.orientation.z = gz_pose_.Rot().Z();
    twist_.linear.x = gz_velocity_.X();
    twist_.linear.y = gz_velocity_.Y();
    twist_.linear.z = gz_velocity_.Z();
    twist_.angular.x = gz_angular_velocity_.X();
    twist_.angular.y = gz_angular_velocity_.Y();
    twist_.angular.z = gz_angular_velocity_.Z();
    acceleration_.linear.x = gz_acceleration_.X();
    acceleration_.linear.y = gz_acceleration_.Y();
    acceleration_.linear.z = gz_acceleration_.Z();
    acceleration_.angular.x = gz_angular_acceleration_.X();
    acceleration_.angular.y = gz_angular_acceleration_.Y();
    acceleration_.angular.z = gz_angular_acceleration_.Z();
#else
    pose_.position.x = gz_pose_.pos.x;
    pose_.position.y = gz_pose_.pos.y;
    pose_.position.z = gz_pose_.pos.z;
    pose_.orientation.w = gz_pose_.rot.w;
    pose_.orientation.x = gz_pose_.rot.x;
    pose_.orientation.y = gz_pose_.rot.y;
    pose_.orientation.z = gz_pose_.rot.z;
    twist_.linear.x = gz_velocity_.x;
    twist_.linear.y = gz_velocity_.y;
    twist_.linear.z = gz_velocity_.z;
    twist_.angular.x = gz_angular_velocity_.x;
    twist_.angular.y = gz_angular_velocity_.y;
    twist_.angular.z = gz_angular_velocity_.z;
    acceleration_.linear.x = gz_acceleration_.x;
    acceleration_.linear.y = gz_acceleration_.y;
    acceleration_.linear.z = gz_acceleration_.z;
    acceleration_.angular.x = gz_angular_acceleration_.x;
    acceleration_.angular.y = gz_angular_acceleration_.y;
    acceleration_.angular.z = gz_angular_acceleration_.z;
#endif
  }

  if (!imu_sub_helper_)
  {
#if (GAZEBO_MAJOR_VERSION >= 8)
    imu_.orientation.w = gz_pose_.Rot().W();
    imu_.orientation.x = gz_pose_.Rot().X();
    imu_.orientation.y = gz_pose_.Rot().Y();
    imu_.orientation.z = gz_pose_.Rot().Z();

    ignition::math::Vector3d gz_angular_velocity_body = gz_pose_.Rot().RotateVectorReverse(gz_angular_velocity_);
    imu_.angular_velocity.x = gz_angular_velocity_body.X();
    imu_.angular_velocity.y = gz_angular_velocity_body.Y();
    imu_.angular_velocity.z = gz_angular_velocity_body.Z();

    ignition::math::Vector3d gz_linear_acceleration_body = gz_pose_.Rot().RotateVectorReverse(
        gz_acceleration_ - model_->GetWorld()->Gravity());
    imu_.linear_acceleration.x = gz_linear_acceleration_body.X();
    imu_.linear_acceleration.y = gz_linear_acceleration_body.Y();
    imu_.linear_acceleration.z = gz_linear_acceleration_body.Z();
#else
    imu_.orientation.w = gz_pose_.rot.w;
    imu_.orientation.x = gz_pose_.rot.x;
    imu_.orientation.y = gz_pose_.rot.y;
    imu_.orientation.z = gz_pose_.rot.z;

    gazebo::math::Vector3 gz_angular_velocity_body = gz_pose_.rot.RotateVectorReverse(gz_angular_velocity_);
    imu_.angular_velocity.x = gz_angular_velocity_body.x;
    imu_.angular_velocity.y = gz_angular_velocity_body.y;
    imu_.angular_velocity.z = gz_angular_velocity_body.z;

    gazebo::math::Vector3 gz_linear_acceleration_body = gz_pose_.rot.RotateVectorReverse(
        gz_acceleration_ - physics_->GetGravity());
    imu_.linear_acceleration.x = gz_linear_acceleration_body.x;
    imu_.linear_acceleration.y = gz_linear_acceleration_body.y;
    imu_.linear_acceleration.z = gz_linear_acceleration_body.z;
#endif
  }
}

void QuadrotorHardwareSim::writeSim(ros::Time time, ros::Duration period)
{
  bool result_written = false;

  if (motor_output_->connected() && motor_output_->enabled()) {
    motor_command_publisher_.publish(motor_output_->getCommand());
    result_written = true;
  }

  if (wrench_output_->connected() && wrench_output_->enabled()) {
    geometry_msgs::WrenchStamped wrench;
    wrench.header.stamp = time;
    wrench.header.frame_id = base_link_frame_;

    if (motor_status_.on && motor_status_.running) {
      wrench.wrench = wrench_limiter_(wrench_output_->getCommand());

      if (!result_written) {
#if (GAZEBO_MAJOR_VERSION >= 8)
        ignition::math::Vector3d force(wrench.wrench.force.x, wrench.wrench.force.y, wrench.wrench.force.z);
        ignition::math::Vector3d torque(wrench.wrench.torque.x, wrench.wrench.torque.y, wrench.wrench.torque.z);
#else
        gazebo::math::Vector3 force(wrench.wrench.force.x, wrench.wrench.force.y, wrench.wrench.force.z);
        gazebo::math::Vector3 torque(wrench.wrench.torque.x, wrench.wrench.torque.y, wrench.wrench.torque.z);
#endif
        link_->AddRelativeForce(force);
#if (GAZEBO_MAJOR_VERSION >= 8)
        link_->AddRelativeTorque(torque - link_->GetInertial()->CoG().Cross(force));
#else
        link_->AddRelativeTorque(torque - link_->GetInertial()->GetCoG().Cross(force));
#endif
      }

    } else {
      wrench.wrench = geometry_msgs::Wrench();
    }

    wrench_command_publisher_.publish(wrench);
  }
}

bool QuadrotorHardwareSim::enableMotorsCallback(hector_uav_msgs::EnableMotors::Request &req, hector_uav_msgs::EnableMotors::Response &res)
{
  res.success = enableMotors(req.enable);
  return true;
}

bool QuadrotorHardwareSim::enableMotors(bool enable)
{
  motor_status_.running = enable;
  return true;
}

} // namespace hector_quadrotor_controller_gazebo

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hector_quadrotor_controller_gazebo::QuadrotorHardwareSim, gazebo_ros_control::RobotHWSim)
