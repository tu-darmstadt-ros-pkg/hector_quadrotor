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
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

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
  updateTimer.Disconnect(updateConnection);

  dynamic_reconfigure_server_.reset();

  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosBaro::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  world = _model->GetWorld();
  link = _model->GetLink();
  link_name_ = link->GetName();

  if (_sdf->HasElement("bodyName"))
  {
    link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();
    link = _model->GetLink(link_name_);
  }

  if (!link)
  {
    ROS_FATAL("gazebo_ros_baro plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }

  // default parameters
  namespace_.clear();
  frame_id_ = link->GetName();
  height_topic_ = "pressure_height";
  altimeter_topic_ = "altimeter";
  elevation_ = DEFAULT_ELEVATION;
  qnh_ = DEFAULT_QNH;

  // load parameters
  if (_sdf->HasElement("robotNamespace"))     namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>();
  if (_sdf->HasElement("frameId"))            frame_id_ = _sdf->GetElement("frameId")->Get<std::string>();
  if (_sdf->HasElement("topicName"))          height_topic_ = _sdf->GetElement("topicName")->Get<std::string>();
  if (_sdf->HasElement("altimeterTopicName")) altimeter_topic_ = _sdf->GetElement("altimeterTopicName")->Get<std::string>();
  if (_sdf->HasElement("elevation"))          elevation_ = _sdf->GetElement("elevation")->Get<double>();
  if (_sdf->HasElement("qnh"))                qnh_ = _sdf->GetElement("qnh")->Get<double>();

  // load sensor model
  sensor_model_.Load(_sdf);

  height_.header.frame_id = frame_id_;

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  node_handle_ = new ros::NodeHandle(namespace_);

  // advertise height
  if (!height_topic_.empty()) {
    height_publisher_ = node_handle_->advertise<geometry_msgs::PointStamped>(height_topic_, 10);
  }

  // advertise altimeter
  if (!altimeter_topic_.empty()) {
    altimeter_publisher_ = node_handle_->advertise<hector_uav_msgs::Altimeter>(altimeter_topic_, 10);
  }

  // setup dynamic_reconfigure server
  dynamic_reconfigure_server_.reset(new dynamic_reconfigure::Server<SensorModelConfig>(ros::NodeHandle(*node_handle_, altimeter_topic_)));
  dynamic_reconfigure_server_->setCallback(boost::bind(&SensorModel::dynamicReconfigureCallback, &sensor_model_, _1, _2));

  Reset();

  // connect Update function
  updateTimer.Load(world, _sdf);
  updateConnection = updateTimer.Connect(boost::bind(&GazeboRosBaro::Update, this));
}

void GazeboRosBaro::Reset()
{
  updateTimer.Reset();
  sensor_model_.reset();
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosBaro::Update()
{
  common::Time sim_time = world->GetSimTime();
  double dt = updateTimer.getTimeSinceLastUpdate().Double();

  math::Pose pose = link->GetWorldPose();
  double height = sensor_model_(pose.pos.z, dt);

  if (height_publisher_) {
    height_.header.stamp = ros::Time(sim_time.sec, sim_time.nsec);
    height_.point.z = height;
    height_publisher_.publish(height_);
  }

  if (altimeter_publisher_) {
    altimeter_.header = height_.header;
    altimeter_.altitude = height + elevation_;
    altimeter_.pressure = pow((1.0 - altimeter_.altitude / 44330.0), 5.263157) * qnh_;
    altimeter_.qnh = qnh_;
    altimeter_publisher_.publish(altimeter_);
  }
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosBaro)

} // namespace gazebo
