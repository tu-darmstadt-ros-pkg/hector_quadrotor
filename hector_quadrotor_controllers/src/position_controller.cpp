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

#include <controller_interface/controller.h>
#include <control_toolbox/pid.h>
#include <geometry_msgs/TwistStamped.h>
#include <hector_quadrotor_interface/quadrotor_interface.h>
#include <limits>
#include <ros/subscriber.h>
#include <hector_quadrotor_interface/limiters.h>
#include <boost/thread/mutex.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h> // for tf::getPrefixParam()
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <cstdlib>
#include <cmath>
#include <pluginlib/class_list_macros.h>
#include <hector_quadrotor_interface/helpers.h>

namespace hector_quadrotor_controllers
{

using namespace hector_quadrotor_interface;

class PositionController : public controller_interface::Controller<QuadrotorInterface>
{
public:
  PositionController()
    : pose_command_valid_(false), twist_limit_valid_(false)
  {
  }

  virtual ~PositionController()
  {
  }

  virtual bool init(QuadrotorInterface *interface,
            ros::NodeHandle &root_nh,
            ros::NodeHandle &controller_nh)
  {
    // get interface handles
    pose_ = interface->getPose();
    twist_ = interface->getTwist();
    motor_status_ = interface->getMotorStatus();

    root_nh.param<std::string>("base_link_frame", base_link_frame_, "base_link");
    root_nh.param<std::string>("world_frame", world_frame_, "/world");
    root_nh.param<std::string>("base_stabilized_frame", base_stabilized_frame_, "base_stabilized");

    // resolve frames
    tf_prefix_ = tf::getPrefixParam(root_nh);
    world_frame_ = tf::resolve(tf_prefix_, world_frame_);
    base_link_frame_ = tf::resolve(tf_prefix_, base_link_frame_);
    base_stabilized_frame_ = tf::resolve(tf_prefix_, base_stabilized_frame_);

    // Initialize PID controllers
    pid_.x.init(ros::NodeHandle(controller_nh, "x"));
    pid_.y.init(ros::NodeHandle(controller_nh, "y"));
    pid_.z.init(ros::NodeHandle(controller_nh, "z"));
    pid_.yaw.init(ros::NodeHandle(controller_nh, "yaw"));

    position_limiter_.init(controller_nh);

    // Setup pose visualization marker output
    initMarker(root_nh.getNamespace());
    marker_publisher_ = root_nh.advertise<visualization_msgs::Marker>("command/pose_marker", 1);

    // Initialize inputs/outputs
    pose_input_ = interface->addInput<PoseCommandHandle>("pose");
    twist_input_  = interface->addInput<TwistCommandHandle>("pose/twist");
    twist_limit_input_  = interface->addInput<TwistCommandHandle>("pose/twist_limit");
    twist_output_ = interface->addOutput<TwistCommandHandle>("twist");

    // subscribe to commanded pose and velocity
    pose_subscriber_ = root_nh.subscribe<geometry_msgs::PoseStamped>("command/pose", 1, boost::bind(
        &PositionController::poseCommandCallback, this, _1));
    twist_limit_subscriber_ = root_nh.subscribe<geometry_msgs::Twist>("command/twist_limit", 1, boost::bind(
        &PositionController::twistLimitCallback, this, _1));

    return true;
  }

  void reset()
  {
    pid_.x.reset();
    pid_.y.reset();
    pid_.z.reset();
    pid_.yaw.reset();

    // Set commanded pose to robot's current pose
    updatePoseCommand(pose_->pose());
    pose_command_valid_ = false;
  }

  virtual void starting(const ros::Time &time)
  {
    reset();
  }

  virtual void stopping(const ros::Time &time)
  {
    twist_output_->stop();
    pose_command_valid_ = false;
//    twist_limit_valid_ = false;
  }

  void poseCommandCallback(const geometry_msgs::PoseStampedConstPtr &command)
  {
    boost::mutex::scoped_lock lock(command_mutex_);

    ros::Time start_time = command->header.stamp;
    if (start_time.isZero()) start_time = ros::Time::now();
    if (!isRunning()) this->startRequest(start_time);

    updatePoseCommand(*command);
  }

  void twistLimitCallback(const geometry_msgs::TwistConstPtr &limit)
  {
    boost::mutex::scoped_lock lock(command_mutex_);

    twist_limit_ = *limit;
    twist_limit_valid_ = true;
  }

  virtual void update(const ros::Time &time, const ros::Duration &period)
  {
    boost::mutex::scoped_lock lock(command_mutex_);
    Twist output;

    // Get pose command command input
    if (pose_input_->connected() && pose_input_->enabled())
    {
      updatePoseCommand(pose_input_->getCommand());
    }

    // Get twist limit input
    if (twist_limit_input_->connected() && twist_limit_input_->enabled())
    {
      twist_limit_ = twist_limit_input_->getCommand();
      twist_limit_valid_ = true;
    }

    // check command timeout
    // TODO

    // Check if pose control was preempted
    if (twist_output_->preempted()) {
      if (pose_command_valid_) {
        ROS_INFO_NAMED("position_controller", "Position control preempted!");
      }
      pose_command_valid_ = false;
    }

    // Check if motors are running
    if (motor_status_->motorStatus().running == false) {
      if (pose_command_valid_) {
        ROS_INFO_NAMED("position_controller", "Disabled position control while motors are not running.");
      }
      pose_command_valid_ = false;
    }

    // Abort if no pose command is available
    if (!pose_command_valid_) {
      reset();
      twist_output_->stop();
      return;
    } else {
      twist_output_->start();
    }

    Pose pose = pose_->pose();
//    Twist twist = twist_->twist();

    double yaw_command;
    {
      tf2::Quaternion q;
      double temp;
      tf2::fromMsg(pose_command_.orientation, q);
      tf2::Matrix3x3(q).getRPY(temp, temp, yaw_command);
    }

    double yaw = pose_->getYaw();

    pose_command_.position = position_limiter_(pose_command_.position);

    output.linear.x = pid_.x.computeCommand(pose_command_.position.x - pose.position.x, period);
    output.linear.y = pid_.y.computeCommand(pose_command_.position.y - pose.position.y, period);
    output.linear.z = pid_.z.computeCommand(pose_command_.position.z - pose.position.z, period);

    double yaw_error = yaw_command - yaw;
    // detect wrap around pi and compensate
    if (yaw_error > M_PI)
    {
      yaw_error -= 2 * M_PI;
    }
    else if (yaw_error < -M_PI)
    {
      yaw_error += 2 * M_PI;
    }
    output.angular.z = pid_.yaw.computeCommand(yaw_error, period);

    // add twist command if available
    if (twist_input_->connected() && twist_input_->enabled())
    {
      output.linear.x  += twist_input_->getCommand().linear.x;
      output.linear.y  += twist_input_->getCommand().linear.y;
      output.linear.z  += twist_input_->getCommand().linear.z;
      output.angular.x += twist_input_->getCommand().angular.x;
      output.angular.y += twist_input_->getCommand().angular.y;
      output.angular.z += twist_input_->getCommand().angular.z;
    }

    // limit twist
    if (twist_limit_valid_)
    {
      double linear_xy = sqrt(output.linear.x*output.linear.x + output.linear.y*output.linear.y);
      double limit_linear_xy  = std::max(twist_limit_.linear.x, twist_limit_.linear.y);
      if (limit_linear_xy > 0.0 && linear_xy > limit_linear_xy) {
        output.linear.x *= limit_linear_xy / linear_xy;
        output.linear.y *= limit_linear_xy / linear_xy;
      }
      if (twist_limit_.linear.z > 0.0 && fabs(output.linear.z) > twist_limit_.linear.z) {
        output.linear.z *= twist_limit_.linear.z / fabs(output.linear.z);
      }
      double angular_xy = sqrt(output.angular.x*output.angular.x + output.angular.y*output.angular.y);
      double limit_angular_xy  = std::max(twist_limit_.angular.x, twist_limit_.angular.y);
      if (limit_angular_xy > 0.0 && angular_xy > limit_angular_xy) {
        output.angular.x *= limit_angular_xy / angular_xy;
        output.angular.y *= limit_angular_xy / angular_xy;
      }
      if (twist_limit_.angular.z > 0.0 && fabs(output.angular.z) > twist_limit_.angular.z) {
        output.angular.z *= twist_limit_.angular.z / fabs(output.angular.z);
      }
    }

    // set twist output
    twist_output_->setCommand(output);
  }

private:
  void updatePoseCommand(const geometry_msgs::PoseStamped &new_pose)
  {
    // TODO TF to world frame
    if (new_pose.header.frame_id != world_frame_) {
      ROS_WARN_STREAM_THROTTLE_NAMED(1.0, "position_controller", "Pose commands must be given in the " << world_frame_ << " frame, ignoring command");
    }
    else
    {
      updatePoseCommand(new_pose.pose);
    }
  }

  void updatePoseCommand(const geometry_msgs::Pose &new_pose)
  {
    {
      pose_command_.position = new_pose.position;
      // Strip non-yaw components from orientation
      tf2::Quaternion q;
      double roll, pitch, yaw;
      tf2::fromMsg(new_pose.orientation, q);
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      q.setRPY(0, 0, yaw);
      pose_command_.orientation = tf2::toMsg(q);
      pose_command_valid_ = true;
    }
    pose_marker_.pose = pose_command_;
    marker_publisher_.publish(pose_marker_);
  }

  void initMarker(std::string name)
  {
    pose_marker_.header.frame_id = world_frame_;
    pose_marker_.ns = name;
    pose_marker_.id = 0;
    pose_marker_.type = visualization_msgs::Marker::ARROW;
    pose_marker_.scale.x = 0.15;
    pose_marker_.scale.y = pose_marker_.scale.z = 0.03;
    pose_marker_.color.r = 0.5;
    pose_marker_.color.g = 0.5;
    pose_marker_.color.r = 0.5;
    pose_marker_.color.a = 1.0;
  }

  PoseHandlePtr pose_;
  TwistHandlePtr twist_;
  MotorStatusHandlePtr motor_status_;

  PoseCommandHandlePtr pose_input_;
  TwistCommandHandlePtr twist_input_;
  TwistCommandHandlePtr twist_limit_input_;
  TwistCommandHandlePtr twist_output_;

  hector_quadrotor_interface::PointLimiter position_limiter_;

  ros::Subscriber pose_subscriber_, twist_limit_subscriber_;
  ros::Publisher marker_publisher_;

  visualization_msgs::Marker pose_marker_;

  geometry_msgs::Pose pose_command_;
  geometry_msgs::Twist twist_limit_;
  bool pose_command_valid_, twist_limit_valid_;

  std::string base_link_frame_, base_stabilized_frame_, world_frame_;
  std::string tf_prefix_;

  struct
  {
    control_toolbox::Pid x, y, z, yaw;
  } pid_;

  boost::mutex command_mutex_;
};

} // namespace hector_quadrotor_controllers

PLUGINLIB_EXPORT_CLASS(hector_quadrotor_controllers::PositionController, controller_interface::ControllerBase)
