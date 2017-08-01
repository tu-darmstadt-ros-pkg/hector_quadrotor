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

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

#include <controller_interface/controller.h>
#include <control_toolbox/pid.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>

#include <tf/transform_listener.h> // for tf::getPrefixParam()

#include <hector_quadrotor_interface/quadrotor_interface.h>
#include <hector_quadrotor_interface/limiters.h>
#include <hector_quadrotor_interface/helpers.h>

#include <boost/thread/mutex.hpp>
#include <limits>

namespace hector_quadrotor_controllers
{

using namespace hector_quadrotor_interface;

class VelocityController : public controller_interface::Controller<hector_quadrotor_interface::QuadrotorInterface>
{
public:
  VelocityController()
  {
  }

  virtual ~VelocityController()
  {
    twist_subscriber_.shutdown();
    cmd_vel_subscriber_.shutdown();
  }

  virtual bool init(hector_quadrotor_interface::QuadrotorInterface *interface, ros::NodeHandle &root_nh,
            ros::NodeHandle &controller_nh)
  {
    // get interface handles
    pose_ = interface->getPose();
    twist_ = interface->getTwist();
//    acceleration_  = interface->getAcceleration();
    motor_status_ = interface->getMotorStatus();

    // load parameters
    root_nh.param<std::string>("base_link_frame", base_link_frame_, "base_link");
    root_nh.param<std::string>("world_frame", world_frame_, "/world");
    root_nh.param<std::string>("base_stabilized_frame", base_stabilized_frame_, "base_stabilized");
    controller_nh.param("limits/load_factor", load_factor_limit_, 1.5);
    getMassAndInertia(root_nh, mass_, inertia_);

    // resolve frames
    tf_prefix_ = tf::getPrefixParam(root_nh);
    world_frame_ = tf::resolve(tf_prefix_, world_frame_);
    base_link_frame_ = tf::resolve(tf_prefix_, base_link_frame_);
    base_stabilized_frame_ = tf::resolve(tf_prefix_, base_stabilized_frame_);

    // initialize PID controllers
    pid_.x.init(ros::NodeHandle(controller_nh, "x"));
    pid_.y.init(ros::NodeHandle(controller_nh, "y"));
    pid_.z.init(ros::NodeHandle(controller_nh, "z"));

    twist_limiter_.linear.init(controller_nh);
    twist_limiter_.angular.init(controller_nh, "yawrate");

    // initialize inputs/outputs
    twist_input_ = interface->addInput<TwistCommandHandle>("twist");
    attitude_output_ = interface->addOutput<AttitudeCommandHandle>("attitude");
    yawrate_output_ = interface->addOutput<YawrateCommandHandle>("yawrate");
    thrust_output_ = interface->addOutput<ThrustCommandHandle>("thrust");

    // Subscribe to commanded twist (geometry_msgs/TwistStamped) and cmd_vel (geometry_msgs/Twist)
    twist_subscriber_ = root_nh.subscribe<geometry_msgs::TwistStamped>("command/twist", 1, boost::bind(
        &VelocityController::twistCommandCallback, this, _1));
    cmd_vel_subscriber_ = root_nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, boost::bind(
        &VelocityController::cmd_velCommandCallback, this, _1));

    return true;
  }

  void reset()
  {
    pid_.x.reset();
    pid_.y.reset();
    pid_.z.reset();

    // Reset command
    twist_command_ = TwistStamped();
  }

  virtual void starting(const ros::Time &time)
  {
    reset();
  }

  virtual void stopping(const ros::Time &time)
  {
    attitude_output_->stop();
    yawrate_output_->stop();
    thrust_output_->stop();
  }

  void twistCommandCallback(const geometry_msgs::TwistStampedConstPtr &command)
  {
    boost::mutex::scoped_lock lock(command_mutex_);

    // start controller if it not running
    if (!isRunning()) this->startRequest(command->header.stamp);

    std::string frame_id = tf::resolve(tf_prefix_, command->header.frame_id);
    if (frame_id != base_stabilized_frame_ && frame_id != world_frame_) {
      ROS_WARN_STREAM_THROTTLE_NAMED(1.0, "velocity_controller", "Velocity commands must be given in either the "
                                     << base_stabilized_frame_ << " or the "
                                     << world_frame_ << " frame, ignoring command");
      return;
    }

    twist_command_ = *command;
    twist_command_.header.frame_id = frame_id; // == resolved frame_id

    // disable position control if command is non-zero
    if ((twist_command_.twist.linear.x != 0.0) ||
        (twist_command_.twist.linear.y != 0.0) ||
        (twist_command_.twist.linear.z != 0.0) ||
        (twist_command_.twist.angular.x != 0.0) ||
        (twist_command_.twist.angular.y != 0.0) ||
        (twist_command_.twist.angular.z != 0.0)) {
      twist_input_->preempt();
    }
  }

  void cmd_velCommandCallback(const geometry_msgs::TwistConstPtr &command)
  {
    boost::mutex::scoped_lock lock(command_mutex_);
    ros::Time now = ros::Time::now();

    // start controller if it not running
    if (!isRunning()) this->startRequest(now);

    twist_command_.twist = *command;
    twist_command_.header.stamp = now;
    twist_command_.header.frame_id = base_stabilized_frame_;

    // disable position control if command is non-zero
    if ((twist_command_.twist.linear.x != 0.0) ||
        (twist_command_.twist.linear.y != 0.0) ||
        (twist_command_.twist.linear.z != 0.0) ||
        (twist_command_.twist.angular.x != 0.0) ||
        (twist_command_.twist.angular.y != 0.0) ||
        (twist_command_.twist.angular.z != 0.0)) {
      twist_input_->preempt();
    }
  }

  virtual void update(const ros::Time &time, const ros::Duration &period)
  {
    boost::mutex::scoped_lock lock(command_mutex_);

    // Get twist command input
    if (twist_input_->connected() && twist_input_->enabled())
    {
      twist_command_.twist = twist_input_->getCommand();
      twist_command_.header.stamp = time;
      twist_command_.header.frame_id = world_frame_;
    }

    // Reset if motors are not running
    if (motor_status_->motorStatus().running == false) {
      reset();
    }

    // Start outputs
    attitude_output_->start();
    yawrate_output_->start();
    thrust_output_->start();

    // Limit twist input
    Twist command = twist_limiter_(twist_command_.twist);

    // Get current yaw and twist in body frame
    double yaw = pose_->getYaw(), sin_yaw, cos_yaw;
    sincos(yaw, &sin_yaw, &cos_yaw);
    Twist twist = twist_->twist(), twist_body;
    twist_body.linear = pose_->toBody(twist.linear);
    twist_body.angular = pose_->toBody(twist.angular);

    // Velocity is controlled in the world frame!
    // Transform to world coordinates if necessary (yaw only)
    if (twist_command_.header.frame_id == base_stabilized_frame_) {
      Twist transformed = command;
      transformed.linear.x  = cos_yaw * command.linear.x  - sin_yaw * command.linear.y;
      transformed.linear.y  = sin_yaw * command.linear.x  + cos_yaw * command.linear.y;
      transformed.angular.x = cos_yaw * command.angular.x - sin_yaw * command.angular.y;
      transformed.angular.y = sin_yaw * command.angular.x + cos_yaw * command.angular.y;
      command = transformed;
    }

    // Get gravity and load factor
    const double gravity = 9.8065;
    double load_factor = 1. / (  pose_->pose().orientation.w * pose_->pose().orientation.w
                                 - pose_->pose().orientation.x * pose_->pose().orientation.x
                                 - pose_->pose().orientation.y * pose_->pose().orientation.y
                                 + pose_->pose().orientation.z * pose_->pose().orientation.z );
    // Note: load_factor could be NaN or Inf...?
    if (load_factor_limit_ > 0.0 && !(load_factor < load_factor_limit_)) load_factor = load_factor_limit_;

    // Run PID loops
    Vector3 acceleration_command;
    acceleration_command.x = pid_.x.computeCommand(command.linear.x - twist.linear.x, period);
    acceleration_command.y = pid_.y.computeCommand(command.linear.y - twist.linear.y, period);
    acceleration_command.z = pid_.z.computeCommand(command.linear.z - twist.linear.z, period) + gravity;

    // Transform acceleration command to base_stabilized frame
    Vector3 acceleration_command_base_stabilized;
    acceleration_command_base_stabilized.x  =  cos_yaw * acceleration_command.x  + sin_yaw * acceleration_command.y;
    acceleration_command_base_stabilized.y  = -sin_yaw * acceleration_command.x  + cos_yaw * acceleration_command.y;
    acceleration_command_base_stabilized.z  = acceleration_command.z;

    ROS_DEBUG_STREAM_NAMED("velocity_controller", "twist.linear:               [" << twist.linear.x << " " << twist.linear.y << " " << twist.linear.z << "]");
    ROS_DEBUG_STREAM_NAMED("velocity_controller", "twist_body.angular:         [" << twist_body.angular.x << " " << twist_body.angular.y << " " << twist_body.angular.z << "]");
    ROS_DEBUG_STREAM_NAMED("velocity_controller", "twist_command.linear:       [" << command.linear.x << " " << command.linear.y << " " << command.linear.z << "]");
    ROS_DEBUG_STREAM_NAMED("velocity_controller", "twist_command.angular:      [" << command.angular.x << " " << command.angular.y << " " << command.angular.z << "]");
//    ROS_DEBUG_STREAM_NAMED("velocity_controller", "acceleration:               [" << acceleration_->acceleration().x << " " << acceleration_->acceleration().y << " " << acceleration_->acceleration().z << "]");
    ROS_DEBUG_STREAM_NAMED("velocity_controller", "acceleration_command_world: [" << acceleration_command.x << " " << acceleration_command.y << " " << acceleration_command.z << "]");
    ROS_DEBUG_STREAM_NAMED("velocity_controller", "acceleration_command_body:  [" << acceleration_command_base_stabilized.x << " " << acceleration_command_base_stabilized.y << " " << acceleration_command_base_stabilized.z << "]");

    hector_uav_msgs::AttitudeCommand attitude_control;
    hector_uav_msgs::YawrateCommand yawrate_control;
    hector_uav_msgs::ThrustCommand thrust_control;
    attitude_control.roll    = -asin(std::min(std::max(acceleration_command_base_stabilized.y / gravity, -1.0), 1.0));
    attitude_control.pitch   =  asin(std::min(std::max(acceleration_command_base_stabilized.x / gravity, -1.0), 1.0));
    yawrate_control.turnrate = command.angular.z;
    thrust_control.thrust    = mass_ * ((acceleration_command.z - gravity) * load_factor + gravity);

    // pass down time stamp from twist command
    attitude_control.header.stamp = twist_command_.header.stamp;
    yawrate_control.header.stamp = twist_command_.header.stamp;
    thrust_control.header.stamp = twist_command_.header.stamp;

    // Update output from controller
    attitude_output_->setCommand(attitude_control);
    yawrate_output_->setCommand(yawrate_control);
    thrust_output_->setCommand(thrust_control);
  }

private:
  PoseHandlePtr pose_;
  TwistHandlePtr twist_;
  MotorStatusHandlePtr motor_status_;

  TwistCommandHandlePtr twist_input_;
  AttitudeCommandHandlePtr attitude_output_;
  YawrateCommandHandlePtr yawrate_output_;
  ThrustCommandHandlePtr thrust_output_;

  ros::Subscriber twist_subscriber_;
  ros::Subscriber cmd_vel_subscriber_;

  geometry_msgs::TwistStamped twist_command_;

  hector_quadrotor_interface::TwistLimiter twist_limiter_;
  std::string base_link_frame_, base_stabilized_frame_, world_frame_;
  std::string tf_prefix_;

  struct
  {
    control_toolbox::Pid x, y, z;
  } pid_;

  double load_factor_limit_;
  double mass_;
  double inertia_[3];

  boost::mutex command_mutex_;
};

} // namespace hector_quadrotor_controllers

PLUGINLIB_EXPORT_CLASS(hector_quadrotor_controllers::VelocityController, controller_interface::ControllerBase)
