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

#include <gazebo/Sensor.hh>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/World.hh>
#include <gazebo/PhysicsEngine.hh>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("hector_gazebo_ros_baro", GazeboRosBaro)

GazeboRosBaro::GazeboRosBaro(Entity *parent)
   : Controller(parent)
   , sensor_model_(parameters)
{
  parent_ = dynamic_cast<Model*>(parent);
  if (!parent_) gzthrow("GazeboRosBaro controller requires a Model as its parent");

  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv, "gazebo", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  Param::Begin(&parameters);
  namespace_ = new ParamT<std::string>("robotNamespace", "", false);
  body_name_ = new ParamT<std::string>("bodyName", "", true);
  frame_id_ = new ParamT<std::string>("frameId", "", false);
  topic_ = new ParamT<std::string>("topicName", "", true);
  elevation_ = new ParamT<double>("elevation", 0.0, false);;
  qnh_ = new ParamT<double>("qnh", 1013.25, false);

  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosBaro::~GazeboRosBaro()
{
  delete namespace_;
  delete body_name_;
  delete frame_id_;
  delete topic_;
  delete elevation_;
  delete qnh_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosBaro::LoadChild(XMLConfigNode *node)
{
  namespace_->Load(node);
  body_name_->Load(node);
  frame_id_->Load(node);
  topic_->Load(node);

  // assert that the body by body_name_ exists
  body_ = dynamic_cast<Body*>(parent_->GetBody(**body_name_));
  if (!body_) gzthrow("gazebo_quadrotor_simple_controller plugin error: body_name_: " << **body_name_ << "does not exist\n");

  elevation_->Load(node);
  qnh_->Load(node);

  sensor_model_.Load(node);
  height_.header.frame_id = **frame_id_;
}

///////////////////////////////////////////////////////
// Initialize the controller
void GazeboRosBaro::InitChild()
{
  node_handle_ = new ros::NodeHandle(**namespace_);
  publisher_ = node_handle_->advertise<mav_msgs::Height>(**topic_, 10);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosBaro::UpdateChild()
{
  Time sim_time = Simulator::Instance()->GetSimTime();
  double dt = (sim_time - lastUpdate).Double();

  Pose3d pose = body_->GetWorldPose();

  double previous_height = height_.height;
  height_.header.stamp = ros::Time(sim_time.sec, sim_time.nsec);
  height_.height = sensor_model_(pose.pos.z, dt) + **elevation_;
  height_.height_variance = sensor_model_.gaussian_noise*sensor_model_.gaussian_noise + sensor_model_.drift*sensor_model_.drift;
  height_.climb = (height_.height - previous_height) / dt;
  height_.climb_variance = sensor_model_.gaussian_noise*sensor_model_.gaussian_noise;

  publisher_.publish(height_);
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void GazeboRosBaro::FiniChild()
{
  node_handle_->shutdown();
  delete node_handle_;
}

