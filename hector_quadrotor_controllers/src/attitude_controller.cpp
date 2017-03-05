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

#include <controller_interface/controller.h>
#include <control_toolbox/pid.h>

#include <geometry_msgs/WrenchStamped.h>

#include <hector_quadrotor_interface/limiters.h>
#include <hector_quadrotor_interface/quadrotor_interface.h>
#include <hector_quadrotor_interface/helpers.h>

#include <tf/transform_listener.h> // for tf::getPrefixParam()

#include <boost/thread/mutex.hpp>
#include <std_msgs/Bool.h>
#include <limits>

namespace hector_quadrotor_controllers
{

using namespace hector_quadrotor_interface;

class AttitudeController : public controller_interface::Controller<hector_quadrotor_interface::QuadrotorInterface>
{
public:
  AttitudeController()
  {
  }

  virtual ~AttitudeController()
  {
    attitude_subscriber_helper_.reset();
    estop_sub_.shutdown();
  }

  virtual bool init(hector_quadrotor_interface::QuadrotorInterface *interface, ros::NodeHandle &root_nh,
            ros::NodeHandle &controller_nh)
  {
    pose_ = interface->getPose();
    twist_ = interface->getTwist();
    accel_ = interface->getAccel();
    motor_status_ = interface->getMotorStatus();

    root_nh.param<std::string>("base_link_frame", base_link_frame_, "base_link");
    root_nh.param<std::string>("base_stabilized_frame", base_stabilized_frame_, "base_stabilized");
    root_nh.param<double>("estop_deceleration", estop_deceleration_, 1.0);
    double command_timeout_sec = 0.0;
    root_nh.param<double>("command_timeout", command_timeout_sec, 0.0);
    command_timeout_ = ros::Duration(command_timeout_sec);

    // resolve frames
    tf_prefix_ = tf::getPrefixParam(root_nh);
    base_link_frame_ = tf::resolve(tf_prefix_, base_link_frame_);
    base_stabilized_frame_ = tf::resolve(tf_prefix_, base_stabilized_frame_);

    getMassAndInertia(root_nh, mass_, inertia_);

    attitude_input_ = interface->addInput<AttitudeCommandHandle>("attitude");
    yawrate_input_ = interface->addInput<YawrateCommandHandle>("yawrate");
    thrust_input_ = interface->addInput<ThrustCommandHandle>("thrust");
    wrench_output_ = interface->addOutput<WrenchCommandHandle>("wrench");

    // subscribe to attitude, yawrate, and thrust
    attitude_subscriber_helper_ = boost::make_shared<AttitudeSubscriberHelper>(ros::NodeHandle(root_nh, "command"),
                                                                               boost::ref(command_mutex_),
                                                                               boost::ref(attitude_command_),
                                                                               boost::ref(yawrate_command_),
                                                                               boost::ref(thrust_command_));

    // initialize PID controllers
    pid_.roll.init(ros::NodeHandle(controller_nh, "roll"));
    pid_.pitch.init(ros::NodeHandle(controller_nh, "pitch"));
    pid_.yawrate.init(ros::NodeHandle(controller_nh, "yawrate"));

    attitude_limiter_.init(controller_nh);
    yawrate_limiter_.init(controller_nh, "yawrate");

    estop_ = false;
    estop_sub_ = root_nh.subscribe("estop", 1, &AttitudeController::estopCb, this);

    return true;
  }

  void reset()
  {
    pid_.roll.reset();
    pid_.pitch.reset();
    pid_.yawrate.reset();
    wrench_control_ = geometry_msgs::WrenchStamped();
  }

  virtual void starting(const ros::Time &time)
  {
    reset();
    wrench_output_->start();
  }

  virtual void stopping(const ros::Time &time)
  {
    wrench_output_->stop();
  }

  virtual void update(const ros::Time &time, const ros::Duration &period)
  {
    boost::mutex::scoped_lock lock(command_mutex_);

    if (attitude_input_->connected() && attitude_input_->enabled())
    {
      attitude_command_ = attitude_input_->getCommand();
    }
    if (yawrate_input_->connected() && yawrate_input_->enabled())
    {
      yawrate_command_ = yawrate_input_->getCommand();
    }
    if (thrust_input_->connected() && thrust_input_->enabled())
    {
      thrust_command_ = thrust_input_->getCommand();
    }

    attitude_command_ = attitude_limiter_(attitude_command_);
    yawrate_command_ = yawrate_limiter_(yawrate_command_);
    thrust_command_ = thrust_limiter_(thrust_command_);

    // TODO move estop to gazebo plugin
    if ((motor_status_->motorStatus().running == true) &&
        !command_timeout_.isZero() && (
            time > attitude_command_.header.stamp + command_timeout_ ||
            time > yawrate_command_.header.stamp + command_timeout_ ||
            time > thrust_command_.header.stamp + command_timeout_ ) )
    {
      if (!command_estop_) {
        estop_thrust_command_ = thrust_command_;
      }
      ROS_WARN_STREAM_THROTTLE_NAMED(1.0, "attitude_controller", "No command received for "
                                    << (time - std::min(std::min(attitude_command_.header.stamp, yawrate_command_.header.stamp), thrust_command_.header.stamp)).toSec() <<
          "s, triggering estop");
      command_estop_ = true;
    } else if (command_estop_) {
      command_estop_ = false;
    }

    double roll, pitch, yaw;
    pose_->getEulerRPY(roll, pitch, yaw);

    Twist twist = twist_->twist(), twist_body;
    twist_body.linear = pose_->toBody(twist.linear);
    twist_body.angular = pose_->toBody(twist.angular);

    Accel accel = accel_->acceleration(), accel_body;
    accel_body.linear = pose_->toBody(accel.linear);
    accel_body.angular = pose_->toBody(accel.angular);

    if (estop_ || command_estop_)
    {
      attitude_command_.roll = attitude_command_.pitch = yawrate_command_.turnrate = 0;
      estop_thrust_command_.thrust -= estop_deceleration_ * mass_ * period.toSec();
      if (estop_thrust_command_.thrust < 0) estop_thrust_command_.thrust = 0;
      thrust_command_ = estop_thrust_command_;
    }

    // Control approach:
    // 1. We consider the roll and pitch commands as desired accelerations in the base_stabilized frame,
    //    not considering wind (inverse of the calculation in velocity_controller.cpp).
    Vector3 acceleration_command_base_stabilized;
    acceleration_command_base_stabilized.x =  sin(attitude_command_.pitch);
    acceleration_command_base_stabilized.y = -sin(attitude_command_.roll);
    acceleration_command_base_stabilized.z = 1.0;

    // 2. Transform desired acceleration to the body frame (via the world frame).
    //    The result is independent of the yaw angle because the yaw rotation will be undone in the toBody() step.
    double sin_yaw, cos_yaw;
    sincos(yaw, &sin_yaw, &cos_yaw);
    Vector3 acceleration_command_world, acceleration_command_body;
    acceleration_command_world.x = cos_yaw * acceleration_command_base_stabilized.x - sin_yaw * acceleration_command_base_stabilized.y;
    acceleration_command_world.y = sin_yaw * acceleration_command_base_stabilized.x + cos_yaw * acceleration_command_base_stabilized.y;
    acceleration_command_world.z = acceleration_command_base_stabilized.z;
    acceleration_command_body = pose_->toBody(acceleration_command_world);

    // 3. Control error is proportional to the desired acceleration in the body frame!
    wrench_control_.wrench.torque.x = inertia_[0] * pid_.roll.computeCommand(-acceleration_command_body.y, period);
    wrench_control_.wrench.torque.y = inertia_[1] * pid_.pitch.computeCommand(acceleration_command_body.x, period);
    wrench_control_.wrench.torque.z = inertia_[2] * pid_.yawrate.computeCommand((yawrate_command_.turnrate - twist_body.angular.z), period);
    wrench_control_.wrench.force.x  = 0.0;
    wrench_control_.wrench.force.y  = 0.0;
    wrench_control_.wrench.force.z = thrust_command_.thrust;

    // set wrench output
    wrench_control_.header.stamp = time;
    wrench_control_.header.frame_id = base_link_frame_;
    wrench_output_->setCommand(wrench_control_.wrench);
  }

  void estopCb(const std_msgs::BoolConstPtr &estop_msg)
  {
    boost::mutex::scoped_lock lock(command_mutex_);

    bool estop = static_cast<bool>(estop_msg->data);
    if (estop_ == false && estop == true)
    {
      estop_thrust_command_ = thrust_command_;
    }
    estop_ = estop;
  }

private:

  PoseHandlePtr pose_;
  TwistHandlePtr twist_;
  AccelerationHandlePtr accel_;
  MotorStatusHandlePtr motor_status_;

  AttitudeCommandHandlePtr attitude_input_;
  YawrateCommandHandlePtr yawrate_input_;
  ThrustCommandHandlePtr thrust_input_;
  WrenchCommandHandlePtr wrench_output_;

  boost::shared_ptr<hector_quadrotor_interface::AttitudeSubscriberHelper> attitude_subscriber_helper_;

  hector_uav_msgs::AttitudeCommand attitude_command_;
  hector_uav_msgs::YawrateCommand yawrate_command_;
  hector_uav_msgs::ThrustCommand thrust_command_;
  geometry_msgs::WrenchStamped wrench_control_;

  hector_quadrotor_interface::AttitudeCommandLimiter attitude_limiter_;
  hector_quadrotor_interface::YawrateCommandLimiter yawrate_limiter_;
  hector_quadrotor_interface::ThrustCommandLimiter thrust_limiter_;
  std::string base_link_frame_, base_stabilized_frame_;
  std::string tf_prefix_;

  ros::Subscriber estop_sub_;
  bool estop_, command_estop_;
  hector_uav_msgs::ThrustCommand estop_thrust_command_;
  double estop_deceleration_;
  ros::Duration command_timeout_;

  struct
  {
    control_toolbox::Pid roll, pitch, yawrate;
  } pid_;

  double mass_;
  double inertia_[3];

  boost::mutex command_mutex_;
};

} // namespace hector_quadrotor_controllers

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(hector_quadrotor_controllers::AttitudeController, controller_interface::ControllerBase
)
