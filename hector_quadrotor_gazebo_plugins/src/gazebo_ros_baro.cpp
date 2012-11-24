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

#include <hector_quadrotor_gazebo_plugins/gazebo_ros_baro.h>
#include "common/Events.hh"
#include "physics/physics.h"

static const double DEFAULT_ELEVATION = 0.0;
static const double DEFAULT_QNH       = 1013.25;

namespace gazebo {

GazeboRosBaro::GazeboRosBaro()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosBaro::~GazeboRosBaro()
{
  event::Events::DisconnectWorldUpdateStart(updateConnection);

  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosBaro::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  world = _model->GetWorld();

  // load parameters
  if (!_sdf->HasElement("robotNamespace"))
    namespace_.clear();
  else
    namespace_ = _sdf->GetElement("robotNamespace")->GetValueString() + "/";

  if (!_sdf->HasElement("bodyName"))
  {
    link = _model->GetLink();
    link_name_ = link->GetName();
  }
  else {
    link_name_ = _sdf->GetElement("bodyName")->GetValueString();
    link = _model->GetLink(link_name_);
  }

  if (!link)
  {
    ROS_FATAL("gazebo_ros_baro plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }

  double update_rate = 0.0;
  if (_sdf->HasElement("updateRate")) update_rate = _sdf->GetElement("updateRate")->GetValueDouble();
  update_period = update_rate > 0.0 ? 1.0/update_rate : 0.0;

  if (!_sdf->HasElement("frameId"))
    frame_id_ = link->GetName();
  else
    frame_id_ = _sdf->GetElement("frameId")->GetValueString();

  if (!_sdf->HasElement("topicName"))
    height_topic_ = "pressure_height";
  else
    height_topic_ = _sdf->GetElement("topicName")->GetValueString();

  if (!_sdf->HasElement("altimeterTopicName"))
    altimeter_topic_ = "altimeter";
  else
    altimeter_topic_ = _sdf->GetElement("altimeterTopicName")->GetValueString();

  if (!_sdf->HasElement("elevation"))
    elevation_ = DEFAULT_ELEVATION;
  else
    elevation_ = _sdf->GetElement("elevation")->GetValueDouble();

  if (!_sdf->HasElement("qnh"))
    qnh_ = DEFAULT_QNH;
  else
    qnh_ = _sdf->GetElement("qnh")->GetValueDouble();

  sensor_model_.Load(_sdf);
  height_.header.frame_id = frame_id_;

  // start ros node
  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  node_handle_ = new ros::NodeHandle(namespace_);
  if (!height_topic_.empty()) {
#ifdef USE_MAV_MSGS
    height_publisher_ = node_handle_->advertise<mav_msgs::Height>(height_topic_, 10);
#else
    height_publisher_ = node_handle_->advertise<geometry_msgs::PointStamped>(height_topic_, 10);
#endif
  }

  if (!altimeter_topic_.empty()) {
    altimeter_publisher_ = node_handle_->advertise<hector_uav_msgs::Altimeter>(altimeter_topic_, 10);
  }

  Reset();

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection = event::Events::ConnectWorldUpdateStart(
      boost::bind(&GazeboRosBaro::Update, this));
}

void GazeboRosBaro::Reset()
{
  sensor_model_.reset();
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosBaro::Update()
{
  common::Time sim_time = world->GetSimTime();
  double dt = (sim_time - last_time).Double();
  if (last_time + update_period > sim_time) return;

  math::Pose pose = link->GetWorldPose();
  double height = sensor_model_(pose.pos.z, dt);

  if (height_publisher_) {
#ifdef USE_MAV_MSGS
    double previous_height = height_.height;
    height_.header.stamp = ros::Time(sim_time.sec, sim_time.nsec);
    height_.height = height;
    height_.height_variance = sensor_model_.gaussian_noise*sensor_model_.gaussian_noise + sensor_model_.drift*sensor_model_.drift;
    height_.climb = dt > 0.0 ? (height_.height - previous_height) / dt : 0.0;
    height_.climb_variance = sensor_model_.gaussian_noise*sensor_model_.gaussian_noise;
#else
    height_.header.stamp = ros::Time(sim_time.sec, sim_time.nsec);
    height_.point.z = height;
#endif

    height_publisher_.publish(height_);
  }

  if (altimeter_publisher_) {
    altimeter_.header = height_.header;
    altimeter_.altitude = height + elevation_;
    altimeter_.pressure = pow((1.0 - altimeter_.altitude / 44330.0), 5.263157) * qnh_;
    altimeter_.qnh = qnh_;
    altimeter_publisher_.publish(altimeter_);
  }

  // save last time stamp
  last_time = sim_time;
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosBaro)

} // namespace gazebo
