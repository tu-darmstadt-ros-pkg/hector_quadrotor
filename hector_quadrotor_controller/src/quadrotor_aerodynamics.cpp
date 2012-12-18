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

#include <gazebo/Sensor.hh>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/World.hh>
#include <gazebo/PhysicsEngine.hh>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>

extern "C" {
  #include "quadrotorDrag/quadrotorDrag.h"
  #include "quadrotorDrag/quadrotorDrag_initialize.h"
}

#include <boost/array.hpp>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("hector_gazebo_quadrotor_aerodynamics", GazeboQuadrotorAerodynamics)

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

GazeboQuadrotorAerodynamics::GazeboQuadrotorAerodynamics(Entity *parent)
   : Controller(parent)
{
  parent_ = dynamic_cast<Model*>(parent);
  if (!parent_) gzthrow("GazeboQuadrotorAerodynamics controller requires a Model as its parent");

  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv, "gazebo", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  Param::Begin(&parameters);
  namespace_ = new ParamT<std::string>("robotNamespace", "", false);
  param_namespace_ = new ParamT<std::string>("paramNamespace", "~/quadrotor_aerodynamics", false);
  body_name_ = new ParamT<std::string>("bodyName", "", true);
  wind_topic_ = new ParamT<std::string>("windTopicName", "wind", false);
  Param::End();

  // initialize drag model
  quadrotorDrag_initialize();
  drag_model_ = new DragModel;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboQuadrotorAerodynamics::~GazeboQuadrotorAerodynamics()
{
  delete namespace_;
  delete param_namespace_;
  delete body_name_;
  delete wind_topic_;

  delete drag_model_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboQuadrotorAerodynamics::LoadChild(XMLConfigNode *node)
{
  namespace_->Load(node);
  param_namespace_->Load(node);
  body_name_->Load(node);
  wind_topic_->Load(node);

  // assert that the body by body_name_ exists
  body_ = dynamic_cast<Body*>(parent_->GetBody(**body_name_));
  if (!body_) gzthrow("gazebo_aerodynamics plugin error: body_name_: " << **body_name_ << "does not exist\n");

  // check update rate against world physics update rate
  // should be equal or higher to guarantee the wrench applied is not "diluted"
  if (this->updatePeriod > 0 &&
      (gazebo::World::Instance()->GetPhysicsEngine()->GetUpdateRate() > 1.0/this->updatePeriod))
    ROS_ERROR_NAMED(this->GetName(), "gazebo_aerodynamics controller update rate is less than physics update rate, wrench applied will be diluted (applied intermittently)");

  // get model parameters
  ros::NodeHandle param(**param_namespace_);
  param.getParam("C_wxy", drag_model_->parameters_.C_wxy);
  param.getParam("C_wz",  drag_model_->parameters_.C_wz);
  param.getParam("C_mxy", drag_model_->parameters_.C_mxy);
  param.getParam("C_mz",  drag_model_->parameters_.C_mz);

  std::cout << "Loaded the following quadrotor drag model parameters from namespace " << param.getNamespace() << ":" << std::endl;
  std::cout << "C_wxy = " << drag_model_->parameters_.C_wxy << std::endl;
  std::cout << "C_wz = "  << drag_model_->parameters_.C_wz << std::endl;
  std::cout << "C_mxy = " << drag_model_->parameters_.C_mxy << std::endl;
  std::cout << "C_mz = "  << drag_model_->parameters_.C_mz << std::endl;
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void GazeboQuadrotorAerodynamics::InitChild()
{
  node_handle_ = new ros::NodeHandle(**namespace_);

  // subscribe command
  if (!(**wind_topic_).empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<geometry_msgs::Vector3>(
      **wind_topic_, 1,
      boost::bind(&GazeboQuadrotorAerodynamics::WindCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    wind_subscriber_ = node_handle_->subscribe(ops);
  }

  callback_queue_thread_ = boost::thread( boost::bind( &GazeboQuadrotorAerodynamics::QueueThread,this ) );

  ResetChild();
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
void GazeboQuadrotorAerodynamics::UpdateChild()
{
  Vector3 force(0.0, 0.0, 0.0), torque(0.0, 0.0, 0.0);
  boost::mutex::scoped_lock lock(wind_mutex_);

  // Get simulator time
  Time current_time = Simulator::Instance()->GetSimTime();
  double dt = current_time - lastUpdate;
  if (dt <= 0.0) return;

  // Get new commands/state
  // callback_queue_.callAvailable();

  // fill input vector u for drag model
  velocity = body_->GetRelativeLinearVel();
  rate = body_->GetRelativeAngularVel();
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
  quadrotorDrag(drag_model_->u.data(), drag_model_->parameters_, dt, drag_model_->y.data());
  force  = force  - checknan(Vector3(drag_model_->y[0], -drag_model_->y[1], -drag_model_->y[2]), "drag model force");
  torque = torque - checknan(Vector3(drag_model_->y[3], -drag_model_->y[4], -drag_model_->y[5]), "drag model torque");

  // set force and torque in gazebo
  body_->SetForce(force);
  body_->SetTorque(torque - body_->GetMass().GetCoG().GetCrossProd(force));
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void GazeboQuadrotorAerodynamics::FiniChild()
{
  node_handle_->shutdown();
  callback_queue_thread_.join();

  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Reset the controller
void GazeboQuadrotorAerodynamics::ResetChild()
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
