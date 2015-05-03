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

#include <hector_quadrotor_controller/quadrotor_hardware_gazebo.h>

#include <geometry_msgs/WrenchStamped.h>

namespace hector_quadrotor_controller_gazebo {

QuadrotorHardwareSim::QuadrotorHardwareSim()
{
  this->registerInterface(static_cast<QuadrotorInterface *>(this));

  wrench_output_ = addInput<WrenchCommandHandle>("wrench");
  motor_output_ = addInput<MotorCommandHandle>("motor");
}

QuadrotorHardwareSim::~QuadrotorHardwareSim()
{

}

bool QuadrotorHardwareSim::initSim(
    const std::string& robot_namespace,
    ros::NodeHandle model_nh,
    gazebo::physics::ModelPtr parent_model,
    const urdf::Model *const urdf_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  ros::NodeHandle param_nh(model_nh, "controller");

  // store parent model pointer
  model_ = parent_model;
  link_ = model_->GetLink();
  physics_ = model_->GetWorld()->GetPhysicsEngine();

  model_nh.param<std::string>("world_frame", world_frame_, "world");
  model_nh.param<std::string>("base_link_frame", base_link_frame_, "base_link");

  // subscribe state
  std::string state_topic;
  param_nh.getParam("state_topic", state_topic);
  if (!state_topic.empty()) {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<nav_msgs::Odometry>(state_topic, 1, boost::bind(&QuadrotorHardwareSim::stateCallback, this, _1), ros::VoidConstPtr(), &callback_queue_);
    subscriber_state_ = model_nh.subscribe(ops);

    gzlog << "[hector_quadrotor_controller_gazebo] Using topic '" << subscriber_state_.getTopic() << "' as state input for control" << std::endl;
  } else {
    gzlog << "[hector_quadrotor_controller_gazebo] Using ground truth from Gazebo as state input for control" << std::endl;
  }

  // subscribe imu
  std::string imu_topic;
  param_nh.getParam("imu_topic", imu_topic);
  if (!imu_topic.empty()) {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<sensor_msgs::Imu>(imu_topic, 1, boost::bind(&QuadrotorHardwareSim::imuCallback, this, _1), ros::VoidConstPtr(), &callback_queue_);
    subscriber_imu_ = model_nh.subscribe(ops);
    gzlog << "[hector_quadrotor_controller_gazebo] Using topic '" << subscriber_imu_.getTopic() << "' as imu input for control" << std::endl;
  } else {
    gzlog << "[hector_quadrotor_controller_gazebo] Using ground truth from Gazebo as imu input for control" << std::endl;
  }

  // subscribe motor_status
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<hector_uav_msgs::MotorStatus>("motor_status", 1, boost::bind(&QuadrotorHardwareSim::motorStatusCallback, this, _1), ros::VoidConstPtr(), &callback_queue_);
    subscriber_motor_status_ = model_nh.subscribe(ops);
  }

  // publish wrench
  {
    ros::AdvertiseOptions ops = ros::AdvertiseOptions::create<geometry_msgs::WrenchStamped>("command/wrench", 1, ros::SubscriberStatusCallback(), ros::SubscriberStatusCallback(), ros::VoidConstPtr(), &callback_queue_);
    publisher_wrench_command_ = model_nh.advertise(ops);
  }

  // publish motor command
  {
    ros::AdvertiseOptions ops = ros::AdvertiseOptions::create<hector_uav_msgs::MotorCommand>("command/motor", 1, ros::SubscriberStatusCallback(), ros::SubscriberStatusCallback(), ros::VoidConstPtr(), &callback_queue_);
    publisher_motor_command_ = model_nh.advertise(ops);
  }

  return true;
}

bool QuadrotorHardwareSim::getMassAndInertia(double &mass, double inertia[3]) {
  if (!link_) return false;
  mass = link_->GetInertial()->GetMass();
  gazebo::math::Vector3 Inertia = link_->GetInertial()->GetPrincipalMoments();
  inertia[0] = Inertia.x;
  inertia[1] = Inertia.y;
  inertia[2] = Inertia.z;
  return true;
}

void QuadrotorHardwareSim::stateCallback(const nav_msgs::OdometryConstPtr &state) {
  // calculate acceleration
  if (!header_.stamp.isZero() && !state->header.stamp.isZero()) {
    const double acceleration_time_constant = 0.1;
    double dt((state->header.stamp - header_.stamp).toSec());
    if (dt > 0.0) {
      acceleration_.x = ((state->twist.twist.linear.x - twist_.linear.x) + acceleration_time_constant * acceleration_.x) / (dt + acceleration_time_constant);
      acceleration_.y = ((state->twist.twist.linear.y - twist_.linear.y) + acceleration_time_constant * acceleration_.y) / (dt + acceleration_time_constant);
      acceleration_.z = ((state->twist.twist.linear.z - twist_.linear.z) + acceleration_time_constant * acceleration_.z) / (dt + acceleration_time_constant);
    }
  }

  header_ = state->header;
  pose_ = state->pose.pose;
  twist_ = state->twist.twist;
}

void QuadrotorHardwareSim::imuCallback(const sensor_msgs::ImuConstPtr &imu) {
  imu_ = *imu;
}

void QuadrotorHardwareSim::motorStatusCallback(const hector_uav_msgs::MotorStatusConstPtr &motor_status) {
  motor_status_ = *motor_status;
}

void QuadrotorHardwareSim::readSim(ros::Time time, ros::Duration period)
{
  // call all available subscriber callbacks now
  callback_queue_.callAvailable();

  // read state from Gazebo
  const double acceleration_time_constant = 0.1;
  gz_pose_             =  link_->GetWorldPose();
  gz_acceleration_     = ((link_->GetWorldLinearVel() - gz_velocity_) + acceleration_time_constant * gz_acceleration_) / (period.toSec() + acceleration_time_constant);
  gz_velocity_         =  link_->GetWorldLinearVel();
  gz_angular_velocity_ =  link_->GetWorldAngularVel();

  if (!subscriber_state_) {
    header_.frame_id = world_frame_;
    header_.stamp = time;
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
    acceleration_.x = gz_acceleration_.x;
    acceleration_.y = gz_acceleration_.y;
    acceleration_.z = gz_acceleration_.z;
  }

  if (!subscriber_imu_) {
    imu_.orientation.w = gz_pose_.rot.w;
    imu_.orientation.x = gz_pose_.rot.x;
    imu_.orientation.y = gz_pose_.rot.y;
    imu_.orientation.z = gz_pose_.rot.z;

    gazebo::math::Vector3 gz_angular_velocity_body = gz_pose_.rot.RotateVectorReverse(gz_angular_velocity_);
    imu_.angular_velocity.x = gz_angular_velocity_body.x;
    imu_.angular_velocity.y = gz_angular_velocity_body.y;
    imu_.angular_velocity.z = gz_angular_velocity_body.z;

    gazebo::math::Vector3 gz_linear_acceleration_body = gz_pose_.rot.RotateVectorReverse(gz_acceleration_ - physics_->GetGravity());
    imu_.linear_acceleration.x = gz_linear_acceleration_body.x;
    imu_.linear_acceleration.y = gz_linear_acceleration_body.y;
    imu_.linear_acceleration.z = gz_linear_acceleration_body.z;
  }
}

void QuadrotorHardwareSim::writeSim(ros::Time time, ros::Duration period)
{
  bool result_written = false;

  if (motor_output_->connected() && motor_output_->enabled()) {
    publisher_motor_command_.publish(motor_output_->getCommand());
    result_written = true;
  }

  if (wrench_output_->connected() && wrench_output_->enabled()) {
    geometry_msgs::WrenchStamped wrench;
    wrench.header.stamp = time;
    wrench.header.frame_id = base_link_frame_;
    wrench.wrench = wrench_output_->getCommand();
    publisher_wrench_command_.publish(wrench);

    if (!result_written) {
      gazebo::math::Vector3 force(wrench.wrench.force.x, wrench.wrench.force.y, wrench.wrench.force.z);
      gazebo::math::Vector3 torque(wrench.wrench.torque.x, wrench.wrench.torque.y, wrench.wrench.torque.z);
      link_->AddRelativeForce(force);
      link_->AddRelativeTorque(torque - link_->GetInertial()->GetCoG().Cross(force));
    }
  }
}

} // namespace hector_quadrotor_controller_gazebo

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hector_quadrotor_controller_gazebo::QuadrotorHardwareSim, gazebo_ros_control::RobotHWSim)
