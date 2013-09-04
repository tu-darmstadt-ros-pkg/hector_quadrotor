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

#include <hector_quadrotor_model/quadrotor_aerodynamics.h>
#include <hector_quadrotor_model/helpers.h>

#include <ros/node_handle.h>
#include <boost/array.hpp>

extern "C" {
  #include "quadrotorDrag/quadrotorDrag.h"
  #include "quadrotorDrag/quadrotorDrag_initialize.h"
}

#include <boost/array.hpp>

namespace hector_quadrotor_model {

// extern void quadrotorDrag(const real_T uin[6], const DragParameters parameter, real_T dt, real_T y[6]);
struct QuadrotorAerodynamics::DragModel {
  DragParameters parameters_;
  boost::array<real_T,6> u;
  boost::array<real_T,6> y;
};

QuadrotorAerodynamics::QuadrotorAerodynamics()
{
  // initialize drag model
  quadrotorDrag_initialize();
  drag_model_ = new DragModel;
}

QuadrotorAerodynamics::~QuadrotorAerodynamics()
{
  delete drag_model_;
}

inline void QuadrotorAerodynamics::f(const real_T uin[10], real_T dt, real_T y[14]) const
{
  ::quadrotorDrag(uin, drag_model_->parameters_, dt, y);
}

void QuadrotorAerodynamics::configure(const std::string &ns)
{
  ros::NodeHandle param(ns);

  // get model parameters
  param.getParam("C_wxy", drag_model_->parameters_.C_wxy);
  param.getParam("C_wz",  drag_model_->parameters_.C_wz);
  param.getParam("C_mxy", drag_model_->parameters_.C_mxy);
  param.getParam("C_mz",  drag_model_->parameters_.C_mz);

#ifndef NDEBUG
  std::cout << "Loaded the following quadrotor drag model parameters from namespace " << param.getNamespace() << ":" << std::endl;
  std::cout << "C_wxy = " << drag_model_->parameters_.C_wxy << std::endl;
  std::cout << "C_wz = "  << drag_model_->parameters_.C_wz << std::endl;
  std::cout << "C_mxy = " << drag_model_->parameters_.C_mxy << std::endl;
  std::cout << "C_mz = "  << drag_model_->parameters_.C_mz << std::endl;
#endif

  reset();
}

void QuadrotorAerodynamics::reset()
{
  boost::mutex::scoped_lock lock(mutex_);
  drag_model_->u.assign(0.0);
  drag_model_->y.assign(0.0);

  twist_ = geometry_msgs::Twist();
  wind_ = geometry_msgs::Vector3();
  wrench_ = geometry_msgs::Wrench();
}

void QuadrotorAerodynamics::setTwist(const geometry_msgs::Twist& twist)
{
  boost::mutex::scoped_lock lock(mutex_);
  twist_ = twist;
}

void QuadrotorAerodynamics::setWind(const geometry_msgs::Vector3& wind)
{
  boost::mutex::scoped_lock lock(mutex_);
  wind_ = wind;
}

void QuadrotorAerodynamics::update(double dt)
{
  if (dt <= 0.0) return;
  boost::mutex::scoped_lock lock(mutex_);

  // fill input vector u for drag model
  drag_model_->u[0] =  (twist_.linear.x - wind_.x);
  drag_model_->u[1] = -(twist_.linear.y - wind_.y);
  drag_model_->u[2] = -(twist_.linear.z - wind_.z);
  drag_model_->u[3] =  twist_.angular.x;
  drag_model_->u[4] = -twist_.angular.y;
  drag_model_->u[5] = -twist_.angular.z;

//  std::cout << "u = [ ";
//  for(std::size_t i = 0; i < drag_model_->u.size(); ++i)
//    std::cout << drag_model_->u[i] << " ";
//  std::cout << "]" << std::endl;

  checknan(drag_model_->u, "drag model input");

  // update drag model
  f(drag_model_->u.data(), dt, drag_model_->y.data());

  checknan(drag_model_->y, "drag model output");

  //  std::cout << "y = [ ";
  //  for(std::size_t i = 0; i < propulsion_model_->y.size(); ++i)
  //    std::cout << propulsion_model_->y[i] << " ";
  //  std::cout << "]" << std::endl;

  wrench_.force.x  =  drag_model_->y[0];
  wrench_.force.y  = -drag_model_->y[1];
  wrench_.force.z  = -drag_model_->y[2];
  wrench_.torque.x =  drag_model_->y[3];
  wrench_.torque.x = -drag_model_->y[4];
  wrench_.torque.x = -drag_model_->y[5];
}

} // namespace hector_quadrotor_model
