//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
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

#include <hector_quadrotor_gazebo_plugins/gazebo_quadrotor_aerodynamics.h>
#include <hector_quadrotor_model/helpers.h>

#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#include <geometry_msgs/WrenchStamped.h>

namespace gazebo {

using namespace common;
using namespace math;
using namespace hector_quadrotor_model;

GazeboQuadrotorAerodynamics::GazeboQuadrotorAerodynamics()
  : node_handle_(0)
{
}

GazeboQuadrotorAerodynamics::~GazeboQuadrotorAerodynamics()
{
  event::Events::DisconnectWorldUpdateBegin(updateConnection);
  if (node_handle_) {
    node_handle_->shutdown();
    if (callback_queue_thread_.joinable())
      callback_queue_thread_.join();
    delete node_handle_;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboQuadrotorAerodynamics::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  world = _model->GetWorld();
  link = _model->GetLink();

  // default parameters
  namespace_.clear();
  param_namespace_ = "quadrotor_aerodynamics";
  wind_topic_ = "/wind";
  wrench_topic_ = "aerodynamics/wrench";
  frame_id_ = link->GetName();

  // load parameters
  if (_sdf->HasElement("robotNamespace")) namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  if (_sdf->HasElement("paramNamespace")) param_namespace_ = _sdf->GetElement("paramNamespace")->Get<std::string>();
  if (_sdf->HasElement("windTopicName"))  wind_topic_ = _sdf->GetElement("windTopicName")->Get<std::string>();
  if (_sdf->HasElement("wrenchTopic"))    wrench_topic_ = _sdf->GetElement("wrenchTopic")->Get<std::string>();
  if (_sdf->HasElement("frameId"))    frame_id_= _sdf->GetElement("frameId")->Get<std::string>();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  node_handle_ = new ros::NodeHandle(namespace_);

  // get model parameters
  if (!model_.configure(ros::NodeHandle(*node_handle_, param_namespace_))) {
    gzwarn << "[quadrotor_propulsion] Could not properly configure the aerodynamics plugin. Make sure you loaded the parameter file." << std::endl;
    return;
  }

  // subscribe command
  if (!wind_topic_.empty())
  {
    ros::SubscribeOptions ops;
    ops.callback_queue = &callback_queue_;
    ops.initByFullCallbackType<const geometry_msgs::Vector3 &>(
      wind_topic_, 1,
      boost::bind(&QuadrotorAerodynamics::setWind, &model_, _1)
    );
    wind_subscriber_ = node_handle_->subscribe(ops);
  }

  // advertise wrench
  if (!wrench_topic_.empty())
  {
    ros::AdvertiseOptions ops;
    ops.callback_queue = &callback_queue_;
    ops.init<geometry_msgs::WrenchStamped>(wrench_topic_, 10);
    wrench_publisher_ = node_handle_->advertise(ops);
  }

  // callback_queue_thread_ = boost::thread( boost::bind( &GazeboQuadrotorAerodynamics::QueueThread,this ) );

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboQuadrotorAerodynamics::Update, this));
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboQuadrotorAerodynamics::Update()
{
  // Get simulator time
  Time current_time = world->GetSimTime();
  Time dt = current_time - last_time_;
  last_time_ = current_time;
  if (dt <= 0.0) return;

  // Get new commands/state
  callback_queue_.callAvailable();

  // fill input vector u for drag model
  geometry_msgs::Quaternion orientation;
  fromQuaternion(link->GetWorldPose().rot, orientation);
  model_.setOrientation(orientation);

  geometry_msgs::Twist twist;
  fromVector(link->GetWorldLinearVel(), twist.linear);
  fromVector(link->GetWorldAngularVel(), twist.angular);
  model_.setTwist(twist);

  // update the model
  model_.update(dt.Double());

  // get wrench from model
  Vector3 force, torque;
  toVector(model_.getWrench().force, force);
  toVector(model_.getWrench().torque, torque);

  // publish wrench
  if (wrench_publisher_) {
    geometry_msgs::WrenchStamped wrench_msg;
    wrench_msg.header.stamp = ros::Time(current_time.sec, current_time.nsec);
    wrench_msg.header.frame_id = frame_id_;
    wrench_msg.wrench = model_.getWrench();
    wrench_publisher_.publish(wrench_msg);
  }

  // set force and torque in gazebo
  link->AddRelativeForce(force);
  link->AddRelativeTorque(torque - link->GetInertial()->GetCoG().Cross(force));
}

////////////////////////////////////////////////////////////////////////////////
// Reset the controller
void GazeboQuadrotorAerodynamics::Reset()
{
  model_.reset();
}

////////////////////////////////////////////////////////////////////////////////
// custom callback queue thread
void GazeboQuadrotorAerodynamics::QueueThread()
{
  static const double timeout = 0.01;

  while (node_handle_->ok())
  {
    callback_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboQuadrotorAerodynamics)

} // namespace gazebo
