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

#include <hector_quadrotor_controller/quadrotor_aerodynamics.h>
#include "common/Events.hh"
#include "physics/physics.h"

extern "C" {
  #include "quadrotorDrag/quadrotorDrag.h"
  #include "quadrotorDrag/quadrotorDrag_initialize.h"
}

#include <boost/array.hpp>

namespace gazebo {

using namespace common;
using namespace math;

template <typename T> static inline T checknan(const T& value, const std::string& text = "") {
  if (!(value == value)) {
    if (!text.empty()) std::cerr << text << " contains **!?* Nan values!" << std::endl;
    return T();
  }
  return value;
}

// extern void quadrotorDrag(const real_T uin[6], const DragParameters parameter, real_T dt, real_T y[6]);
struct GazeboQuadrotorAerodynamics::DragModel {
  DragParameters parameters_;
  boost::array<real_T,6> u;
  boost::array<real_T,6> y;
};

GazeboQuadrotorAerodynamics::GazeboQuadrotorAerodynamics()
{
  // initialize drag model
  quadrotorDrag_initialize();
  drag_model_ = new DragModel;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboQuadrotorAerodynamics::~GazeboQuadrotorAerodynamics()
{
  event::Events::DisconnectWorldUpdateStart(updateConnection);
  node_handle_->shutdown();
  callback_queue_thread_.join();
  delete node_handle_;

  delete drag_model_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboQuadrotorAerodynamics::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  world = _model->GetWorld();
  link = _model->GetLink();

  // load parameters
  if (!_sdf->HasElement("robotNamespace"))
    namespace_.clear();
  else
    namespace_ = _sdf->GetElement("robotNamespace")->GetValueString() + "/";

  if (!_sdf->HasElement("paramNamespace"))
    param_namespace_ = "~/quadrotor_aerodynamics/";
  else
    param_namespace_ = _sdf->GetElement("paramNamespace")->GetValueString() + "/";

  if (!_sdf->HasElement("windTopicName"))
    wind_topic_ = "wind";
  else
    wind_topic_ = _sdf->GetElement("windTopicName")->GetValueString();

  // start ros node
  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  node_handle_ = new ros::NodeHandle(namespace_);

  // subscribe command
  if (!wind_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<geometry_msgs::Vector3>(
      wind_topic_, 1,
      boost::bind(&GazeboQuadrotorAerodynamics::WindCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    wind_subscriber_ = node_handle_->subscribe(ops);
  }

  callback_queue_thread_ = boost::thread( boost::bind( &GazeboQuadrotorAerodynamics::QueueThread,this ) );

  // get model parameters
  ros::NodeHandle param(param_namespace_);
  param.getParam("C_wxy", drag_model_->parameters_.C_wxy);
  param.getParam("C_wz",  drag_model_->parameters_.C_wz);
  param.getParam("C_mxy", drag_model_->parameters_.C_mxy);
  param.getParam("C_mz",  drag_model_->parameters_.C_mz);

  std::cout << "Loaded the following quadrotor drag model parameters from namespace " << param.getNamespace() << ":" << std::endl;
  std::cout << "C_wxy = " << drag_model_->parameters_.C_wxy << std::endl;
  std::cout << "C_wz = "  << drag_model_->parameters_.C_wz << std::endl;
  std::cout << "C_mxy = " << drag_model_->parameters_.C_mxy << std::endl;
  std::cout << "C_mz = "  << drag_model_->parameters_.C_mz << std::endl;

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection = event::Events::ConnectWorldUpdateStart(
      boost::bind(&GazeboQuadrotorAerodynamics::Update, this));
}

////////////////////////////////////////////////////////////////////////////////
// Callbacks
void GazeboQuadrotorAerodynamics::WindCallback(const geometry_msgs::Vector3ConstPtr& wind)
{
  boost::mutex::scoped_lock lock(wind_mutex_);
  wind_ = *wind;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboQuadrotorAerodynamics::Update()
{
  Vector3 force(0.0, 0.0, 0.0), torque(0.0, 0.0, 0.0);
  boost::mutex::scoped_lock lock(wind_mutex_);

  // Get simulator time
  Time current_time = world->GetSimTime();
  Time dt = current_time - last_time_;
  last_time_ = current_time;
  if (dt <= 0.0) return;

  // Get new commands/state
  // callback_queue_.callAvailable();

  // fill input vector u for drag model
  velocity = link->GetRelativeLinearVel();
  rate = link->GetRelativeAngularVel();
  drag_model_->u[0] =  (velocity.x - wind_.x);
  drag_model_->u[1] = -(velocity.y - wind_.y);
  drag_model_->u[2] = -(velocity.z - wind_.z);
  drag_model_->u[3] =  rate.x;
  drag_model_->u[4] = -rate.y;
  drag_model_->u[5] = -rate.z;

//  std::cout << "u = [ ";
//  for(std::size_t i = 0; i < drag_model_->u.size(); ++i)
//    std::cout << drag_model_->u[i] << " ";
//  std::cout << "]" << std::endl;

  checknan(drag_model_->u, "drag model input");

  // update drag model
  quadrotorDrag(drag_model_->u.data(), drag_model_->parameters_, dt.Double(), drag_model_->y.data());
  force  = force  - checknan(Vector3(drag_model_->y[0], -drag_model_->y[1], -drag_model_->y[2]), "drag model force");
  torque = torque - checknan(Vector3(drag_model_->y[3], -drag_model_->y[4], -drag_model_->y[5]), "drag model torque");

  // set force and torque in gazebo
  link->AddRelativeForce(force);
  link->AddRelativeTorque(torque - link->GetInertial()->GetCoG().GetCrossProd(force));
}

////////////////////////////////////////////////////////////////////////////////
// Reset the controller
void GazeboQuadrotorAerodynamics::Reset()
{
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
