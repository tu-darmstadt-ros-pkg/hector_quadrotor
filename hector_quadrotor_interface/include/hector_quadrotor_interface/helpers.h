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

#ifndef HECTOR_QUADROTOR_INTERFACE_HELPERS_H
#define HECTOR_QUADROTOR_INTERFACE_HELPERS_H

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/TransformStamped.h>

#include <hector_uav_msgs/AttitudeCommand.h>
#include <hector_uav_msgs/YawrateCommand.h>
#include <hector_uav_msgs/ThrustCommand.h>
#include <std_msgs/Header.h>

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <boost/circular_buffer.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace hector_quadrotor_interface
{

class ImuSubscriberHelper
{
public:
  ImuSubscriberHelper(ros::NodeHandle nh, std::string topic, sensor_msgs::Imu &imu)
      : imu_(imu)
  {
    imu_sub_ = nh.subscribe<sensor_msgs::Imu>(topic, 1, boost::bind(&ImuSubscriberHelper::imuCallback, this, _1));
  }

  ~ImuSubscriberHelper()
  {
    imu_sub_.shutdown();
  }

private:
  sensor_msgs::Imu &imu_;
  ros::Subscriber imu_sub_;

  void imuCallback(const sensor_msgs::ImuConstPtr &imu)
  {
    imu_ = *imu;
  }

};

// TODO switch to shapeshifter

class OdomSubscriberHelper
{
public:
  OdomSubscriberHelper(ros::NodeHandle nh, std::string topic, geometry_msgs::Pose &pose, geometry_msgs::Twist &twist,
                       geometry_msgs::Accel &acceleration, std_msgs::Header &header)
      : pose_(pose), twist_(twist), acceleration_(acceleration), header_(header)
  {
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>(topic, 1,
                                                 boost::bind(&OdomSubscriberHelper::odomCallback, this, _1));
  }

  ~OdomSubscriberHelper()
  {
    odom_sub_.shutdown();
  }

private:
  ros::Subscriber odom_sub_;

  geometry_msgs::Pose &pose_;
  geometry_msgs::Twist &twist_;
  geometry_msgs::Accel &acceleration_;
  std_msgs::Header header_;

  void odomCallback(const nav_msgs::OdometryConstPtr &odom)
  {
    // calculate acceleration
    if (!header_.stamp.isZero() && !odom->header.stamp.isZero())
    {
      const double acceleration_time_constant = 0.1;
      double dt((odom->header.stamp - header_.stamp).toSec());
      if (dt > 0.0)
      {
        acceleration_.linear.x =
            ((odom->twist.twist.linear.x - twist_.linear.x) + acceleration_time_constant * acceleration_.linear.x) /
            (dt + acceleration_time_constant);
        acceleration_.linear.y =
            ((odom->twist.twist.linear.y - twist_.linear.y) + acceleration_time_constant * acceleration_.linear.y) /
            (dt + acceleration_time_constant);
        acceleration_.linear.z =
            ((odom->twist.twist.linear.z - twist_.linear.z) + acceleration_time_constant * acceleration_.linear.z) /
            (dt + acceleration_time_constant);
        acceleration_.angular.x = ((odom->twist.twist.angular.x - twist_.angular.x) +
                                   acceleration_time_constant * acceleration_.angular.x) /
                                  (dt + acceleration_time_constant);
        acceleration_.angular.y = ((odom->twist.twist.angular.y - twist_.angular.y) +
                                   acceleration_time_constant * acceleration_.angular.y) /
                                  (dt + acceleration_time_constant);
        acceleration_.angular.z = ((odom->twist.twist.angular.z - twist_.angular.z) +
                                   acceleration_time_constant * acceleration_.angular.z) /
                                  (dt + acceleration_time_constant);
      }
    }

    header_ = odom->header;
    pose_ = odom->pose.pose;
    twist_ = odom->twist.twist;
  }

};

class PoseSubscriberHelper
{
public:
  PoseSubscriberHelper(ros::NodeHandle nh, std::string topic)
  {
    pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>(topic, 1,
                                                         boost::bind(&PoseSubscriberHelper::poseCallback, this,
                                                                     _1));
  }

  ~PoseSubscriberHelper()
  {
    pose_sub_.shutdown();
  }

  geometry_msgs::PoseStampedConstPtr get() const { return pose_; }

private:
  ros::Subscriber pose_sub_;

  geometry_msgs::PoseStampedConstPtr pose_;

  void poseCallback(const geometry_msgs::PoseStampedConstPtr &pose)
  {
    pose_ = pose;
  }

};

class PoseDifferentiatorHelper
{
public:
  void updateAndEstimate(const ros::Time &time, const geometry_msgs::Pose &pose, geometry_msgs::Twist &twist,
                         geometry_msgs::Accel &accel)
  {

    if (last_pose_)
    {

      double dt = (time - last_time_).toSec();
      double roll, pitch, yaw, last_roll, last_pitch, last_yaw;
      tf2::Quaternion q;

      tf2::fromMsg(pose.orientation, q);
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

      tf2::fromMsg(last_pose_->orientation, q);
      tf2::Matrix3x3(q).getRPY(last_roll, last_pitch, last_yaw);

      twist.linear.x = (pose.position.x - last_pose_->position.x) / dt;
      twist.linear.y = (pose.position.y - last_pose_->position.y) / dt;
      twist.linear.z = (pose.position.z - last_pose_->position.z) / dt;
      twist.angular.x = differenceWithWraparound(roll, last_roll) / dt;
      twist.angular.y = differenceWithWraparound(pitch, last_pitch) / dt;
      twist.angular.z = differenceWithWraparound(yaw, last_yaw) / dt;

      if (last_twist_)
      {
        accel.linear.x = (twist.linear.x - last_twist_->linear.x) / dt;
        accel.linear.y = (twist.linear.y - last_twist_->linear.y) / dt;
        accel.linear.z = (twist.linear.z - last_twist_->linear.z) / dt;

        accel.angular.x = (twist.angular.x - last_twist_->angular.x) / dt;
        accel.angular.y = (twist.angular.y - last_twist_->angular.y) / dt;
        accel.angular.z = (twist.angular.z - last_twist_->angular.z) / dt;
        *last_twist_ = twist;
      }
      else
      {
        last_twist_ = boost::make_shared<geometry_msgs::Twist>(twist);
      }
      *last_pose_ = pose;

    }
    else
    {
      last_pose_ = boost::make_shared<geometry_msgs::Pose>(pose);
    }
    last_time_ = time;

  }

private:
  geometry_msgs::PosePtr last_pose_;
  geometry_msgs::TwistPtr last_twist_;
  ros::Time last_time_;

  double differenceWithWraparound(double angle, double last_angle)
  {

    double diff = angle - last_angle;
    if (diff > M_PI)
    {
      return diff - 2 * M_PI;
    }
    else if (diff < -M_PI)
    {
      return diff + 2 * M_PI;
    }
    else
    {
      return diff;
    }

  }

};

class StateSubscriberHelper
{
public:
  StateSubscriberHelper(ros::NodeHandle nh, std::string topic, geometry_msgs::Pose &pose,
                       geometry_msgs::Twist &twist, geometry_msgs::Accel &accel, std_msgs::Header &header)
      : pose_(pose), twist_(twist), accel_(accel), header_(header)
  {
    available_ = false;
    tf_sub_ = nh.subscribe<geometry_msgs::TransformStamped>(topic, 1,
                                                            boost::bind(&StateSubscriberHelper::tfCb,
                                                                        this, _1));
  }

  ~StateSubscriberHelper()
  {
    tf_sub_.shutdown();
  }

  bool isAvailable(){ return available_; }

private:
  ros::Subscriber tf_sub_;

  //TODO remove available?
  bool available_;

  geometry_msgs::Pose &pose_;
  geometry_msgs::Twist &twist_;
  geometry_msgs::Accel &accel_;
  std_msgs::Header &header_;

  PoseDifferentiatorHelper diff_;

  void tfCb(const geometry_msgs::TransformStampedConstPtr &transform)
  {
    header_ = transform->header;
    pose_.position.x = transform->transform.translation.x;
    pose_.position.y = transform->transform.translation.y;
    pose_.position.z = transform->transform.translation.z;
    pose_.orientation = transform->transform.rotation;

    diff_.updateAndEstimate(header_.stamp, pose_, twist_, accel_);
    available_ = true;
  }

  // TODO shapeshifter to replace PoseSubscriber and OdomSubscriberHelper
//    void stateCb(topic_tools::ShapeShifter const &input) {
//      if (input.getDataType() == "nav_msgs/Odometry") {
//        nav_msgs::Odometry::ConstPtr odom = input.instantiate<nav_msgs::Odometry>();
//        odomCallback(*odom);
//        return;
//      }

//      if (input.getDataType() == "geometry_msgs/PoseStamped") {
//        geometry_msgs::PoseStamped::ConstPtr pose = input.instantiate<geometry_msgs::PoseStamped>();
//        poseCallback(*pose);
//        return;
//      }

//      if (input.getDataType() == "sensor_msgs/Imu") {
//        sensor_msgs::Imu::ConstPtr imu = input.instantiate<sensor_msgs::Imu>();
//        imuCallback(*imu);
//        return;
//      }

//      if (input.getDataType() == "geometry_msgs/TransformStamped") {
//        geometry_msgs::TransformStamped::ConstPtr tf = input.instantiate<geometry_msgs::TransformStamped>();
//        tfCallback(*tf);
//        return;
//      }

//      ROS_ERROR_THROTTLE(1.0, "message_to_tf received a %s message. Supported message types: nav_msgs/Odometry geometry_msgs/PoseStamped sensor_msgs/Imu", input.getDataType().c_str());
//    }

};

class AttitudeSubscriberHelper
{
public:

  AttitudeSubscriberHelper(ros::NodeHandle nh, boost::mutex &command_mutex,
                           hector_uav_msgs::AttitudeCommand &attitude_command,
                           hector_uav_msgs::YawrateCommand &yawrate_command,
                           hector_uav_msgs::ThrustCommand &thrust_command)
      : command_mutex_(command_mutex), attitude_command_(attitude_command), yawrate_command_(yawrate_command),
        thrust_command_(thrust_command)
  {
    attitude_subscriber_ = nh.subscribe<hector_uav_msgs::AttitudeCommand>("attitude", 1, boost::bind(
        &AttitudeSubscriberHelper::attitudeCommandCb, this, _1));
    yawrate_subscriber_ = nh.subscribe<hector_uav_msgs::YawrateCommand>("yawrate", 1, boost::bind(
        &AttitudeSubscriberHelper::yawrateCommandCb, this, _1));
    thrust_subscriber_ = nh.subscribe<hector_uav_msgs::ThrustCommand>("thrust", 1, boost::bind(
        &AttitudeSubscriberHelper::thrustCommandCb, this, _1));
  }

  ~AttitudeSubscriberHelper()
  {
    attitude_subscriber_.shutdown();
    yawrate_subscriber_.shutdown();
    thrust_subscriber_.shutdown();
  }

private:
  ros::Subscriber attitude_subscriber_, yawrate_subscriber_, thrust_subscriber_;
  boost::mutex &command_mutex_;
  hector_uav_msgs::AttitudeCommand &attitude_command_;
  hector_uav_msgs::YawrateCommand &yawrate_command_;
  hector_uav_msgs::ThrustCommand &thrust_command_;

  void attitudeCommandCb(const hector_uav_msgs::AttitudeCommandConstPtr &command)
  {
    boost::mutex::scoped_lock lock(command_mutex_);
    attitude_command_ = *command;
    if (attitude_command_.header.stamp.isZero())
    { attitude_command_.header.stamp = ros::Time::now(); }
  }

  void yawrateCommandCb(const hector_uav_msgs::YawrateCommandConstPtr &command)
  {
    boost::mutex::scoped_lock lock(command_mutex_);
    yawrate_command_ = *command;
    if (yawrate_command_.header.stamp.isZero())
    { yawrate_command_.header.stamp = ros::Time::now(); }
  }

  void thrustCommandCb(const hector_uav_msgs::ThrustCommandConstPtr &command)
  {
    boost::mutex::scoped_lock lock(command_mutex_);
    thrust_command_ = *command;
    if (thrust_command_.header.stamp.isZero())
    { attitude_command_.header.stamp = ros::Time::now(); }
  }
};

bool getMassAndInertia(const ros::NodeHandle &nh, double &mass, double inertia[3]);


bool poseWithinTolerance(const geometry_msgs::Pose &pose_current, const geometry_msgs::Pose &pose_target,
                         const double dist_tolerance, const double yaw_tolerance);

} // namespace hector_quadrotor_interface

#endif //  HECTOR_QUADROTOR_INTERFACE_HELPERS_H
