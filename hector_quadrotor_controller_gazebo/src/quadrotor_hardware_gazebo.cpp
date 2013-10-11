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

namespace hector_quadrotor_controller_gazebo {

QuadrotorHardwareSim::QuadrotorHardwareSim()
{
  this->registerInterface(static_cast<QuadrotorInterface *>(this));
}

QuadrotorHardwareSim::~QuadrotorHardwareSim()
{

}

bool QuadrotorHardwareSim::initSim(
    ros::NodeHandle model_nh,
    gazebo::physics::ModelPtr parent_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  ros::SubscribeOptions sops;
  ros::AdvertiseOptions aops;
  ros::NodeHandle param_nh(model_nh, "controller");

  // subscribe state
  std::string state_topic = "state";
  if (param_nh.getParam("state_topic", state_topic)) {
      gzlog << "[hector_quadrotor_controller_gazebo] Using topic " << state_topic << " as state input for control" << std::endl;
  }
  sops = ros::SubscribeOptions::create<nav_msgs::Odometry>(state_topic, 1, boost::bind(&QuadrotorHardwareSim::stateCallback, this, _1), ros::VoidConstPtr(), &callback_queue_);
  subscriber_state_ = model_nh.subscribe(sops);

  // subscribe imu
  std::string imu_topic = "imu";
  if (param_nh.getParam("imu_topic", imu_topic)) {
      gzlog << "[hector_quadrotor_controller_gazebo] Using topic " << imu_topic << " as imu input for control" << std::endl;
  }
  sops = ros::SubscribeOptions::create<sensor_msgs::Imu>(imu_topic, 1, boost::bind(&QuadrotorHardwareSim::imuCallback, this, _1), ros::VoidConstPtr(), &callback_queue_);
  subscriber_imu_ = model_nh.subscribe(sops);

  // subscribe motor_status
  sops = ros::SubscribeOptions::create<hector_uav_msgs::MotorStatus>("motor_status", 1, boost::bind(&QuadrotorHardwareSim::motorStatusCallback, this, _1), ros::VoidConstPtr(), &callback_queue_);
  subscriber_motor_status_ = model_nh.subscribe(sops);

  // set controller mode
  std::string mode = "twist";
  param_nh.getParam("mode", mode);
  if (mode == "twist") {
    mode_ = MODE_TWIST;
    gzlog << "[hector_quadrotor_controller_gazebo] Running in TWIST mode" << std::endl;
  } else if (mode == "wrench") {
    mode_ = MODE_WRENCH;
    gzlog << "[hector_quadrotor_controller_gazebo] Running in WRENCH mode" << std::endl;
  } else if (mode == "motor") {
    mode_ = MODE_MOTOR;
    gzlog << "[hector_quadrotor_controller_gazebo] Running in MOTOR mode" << std::endl;
  } else {
    gzerr << "[hector_quadrotor_controller_gazebo] Unknown mode string '" << mode << "'" << std::endl;
    return false;
  }

  // publish twist
  if (mode_ == MODE_TWIST) {
    aops = ros::AdvertiseOptions::create<geometry_msgs::Twist>("cmd_vel", 1, ros::SubscriberStatusCallback(), ros::SubscriberStatusCallback(), ros::VoidConstPtr(), &callback_queue_);
    publisher_twist_command_ = model_nh.advertise(aops);
  }

  // publish wrench
  if (mode_ == MODE_WRENCH) {
    aops = ros::AdvertiseOptions::create<geometry_msgs::Wrench>("command/wrench", 1, ros::SubscriberStatusCallback(), ros::SubscriberStatusCallback(), ros::VoidConstPtr(), &callback_queue_);
    publisher_wrench_command_ = model_nh.advertise(aops);
  }

  // publish motor command
  if (mode_ == MODE_MOTOR) {
    aops = ros::AdvertiseOptions::create<hector_uav_msgs::MotorCommand>("command/motor", 1, ros::SubscriberStatusCallback(), ros::SubscriberStatusCallback(), ros::VoidConstPtr(), &callback_queue_);
    publisher_motor_command_ = model_nh.advertise(aops);
  }
}

void QuadrotorHardwareSim::stateCallback(const nav_msgs::OdometryConstPtr &state) {
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
  // just call all available subscriber callbacks
  callback_queue_.callAvailable();
}

void QuadrotorHardwareSim::writeSim(ros::Time time, ros::Duration period)
{
  TwistCommandHandle twist = getHandle<TwistCommandHandle>();
  if (mode_ == MODE_TWIST && publisher_twist_command_ && twist.enabled()) {
    publisher_twist_command_.publish(twist.getCommand());
    return;
  }

  WrenchCommandHandle wrench = getHandle<WrenchCommandHandle>();
  if (mode_ == MODE_WRENCH && wrench.enabled()) {
    publisher_wrench_command_.publish(wrench.getCommand());
    return;
  }

  MotorCommandHandle motor = getHandle<MotorCommandHandle>();
  if (mode_ == MODE_MOTOR && motor.enabled()) {
    publisher_motor_command_.publish(motor.getCommand());
    return;
  }
}

} // namespace hector_quadrotor_controller_gazebo

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hector_quadrotor_controller_gazebo::QuadrotorHardwareSim, gazebo_ros_control::RobotHWSim)
