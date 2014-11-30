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

//extern "C" {
//  #include "quadrotorDrag/quadrotorDrag.h"
//  #include "quadrotorDrag/quadrotorDrag_initialize.h"
//}

#include <boost/array.hpp>
#include <Eigen/Geometry>

#include "matlab_helpers.h"

namespace hector_quadrotor_model {

struct DragParameters
{
    real_T C_wxy;
    real_T C_wz;
    real_T C_mxy;
    real_T C_mz;

    DragParameters()
      : C_wxy(0.0)
      , C_wz(0.0)
      , C_mxy(0.0)
      , C_mz(0.0)
    {}
};

// extern void quadrotorDrag(const real_T uin[6], const DragParameters parameter, real_T dt, real_T y[6]);
struct QuadrotorAerodynamics::DragModel {
  DragParameters parameters_;
  boost::array<real_T,6> u;
  boost::array<real_T,6> y;
};

QuadrotorAerodynamics::QuadrotorAerodynamics()
{
  // initialize drag model
  // quadrotorDrag_initialize();
  drag_model_ = new DragModel;
}

QuadrotorAerodynamics::~QuadrotorAerodynamics()
{
  delete drag_model_;
}

/*
 * quadrotorDrag.c
 *
 * Code generation for function 'quadrotorDrag'
 *
 * C source code generated on: Sun Nov  3 13:34:38 2013
 *
 */

/* Include files */
//#include "rt_nonfinite.h"
//#include "quadrotorDrag.h"

/* Type Definitions */

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Definitions */
void quadrotorDrag(const real_T uin[6], const DragParameters parameter, real_T
                   dt, real_T y[6])
{
  int32_T i;
  real_T absoluteVelocity;
  real_T absoluteAngularVelocity;

  /*  initialize vectors */
  for (i = 0; i < 6; i++) {
    y[i] = 0.0;
  }

  /*  Input variables */
  /*  Constants */
  /*  temporarily used vector */
  absoluteVelocity = sqrt((rt_powd_snf(uin[0], 2.0) + rt_powd_snf(uin[1], 2.0))
    + rt_powd_snf(uin[2], 2.0));
  absoluteAngularVelocity = sqrt((rt_powd_snf(uin[3], 2.0) + rt_powd_snf(uin[4],
    2.0)) + rt_powd_snf(uin[5], 2.0));

  /*  system outputs */
  /*  calculate drag force */
  y[0] = parameter.C_wxy * absoluteVelocity * uin[0];
  y[1] = parameter.C_wxy * absoluteVelocity * uin[1];
  y[2] = parameter.C_wz * absoluteVelocity * uin[2];

  /*  calculate draq torque */
  y[3] = parameter.C_mxy * absoluteAngularVelocity * uin[3];
  y[4] = parameter.C_mxy * absoluteAngularVelocity * uin[4];
  y[5] = parameter.C_mz * absoluteAngularVelocity * uin[5];
}

/* End of code generation (quadrotorDrag.c) */

inline void QuadrotorAerodynamics::f(const double uin[10], double dt, double y[14]) const
{
  quadrotorDrag(uin, drag_model_->parameters_, dt, y);
}

bool QuadrotorAerodynamics::configure(const ros::NodeHandle &param)
{
  // get model parameters
  if (!param.getParam("C_wxy", drag_model_->parameters_.C_wxy)) return false;
  if (!param.getParam("C_wz",  drag_model_->parameters_.C_wz)) return false;
  if (!param.getParam("C_mxy", drag_model_->parameters_.C_mxy)) return false;
  if (!param.getParam("C_mz",  drag_model_->parameters_.C_mz)) return false;

#ifndef NDEBUG
  std::cout << "Loaded the following quadrotor drag model parameters from namespace " << param.getNamespace() << ":" << std::endl;
  std::cout << "C_wxy = " << drag_model_->parameters_.C_wxy << std::endl;
  std::cout << "C_wz = "  << drag_model_->parameters_.C_wz << std::endl;
  std::cout << "C_mxy = " << drag_model_->parameters_.C_mxy << std::endl;
  std::cout << "C_mz = "  << drag_model_->parameters_.C_mz << std::endl;
#endif

  reset();
  return true;
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

void QuadrotorAerodynamics::setOrientation(const geometry_msgs::Quaternion& orientation)
{
  boost::mutex::scoped_lock lock(mutex_);
  orientation_ = orientation;
}

void QuadrotorAerodynamics::setTwist(const geometry_msgs::Twist& twist)
{
  boost::mutex::scoped_lock lock(mutex_);
  twist_ = twist;
}

void QuadrotorAerodynamics::setBodyTwist(const geometry_msgs::Twist& body_twist)
{
  boost::mutex::scoped_lock lock(mutex_);
  Eigen::Quaterniond orientation(orientation_.w, orientation_.x, orientation_.y, orientation_.z);
  Eigen::Matrix<double,3,3> inverse_rotation_matrix(orientation.inverse().toRotationMatrix());

  Eigen::Vector3d body_linear(body_twist.linear.x, body_twist.linear.y, body_twist.linear.z);
  Eigen::Vector3d world_linear(inverse_rotation_matrix * body_linear);
  twist_.linear.x = world_linear.x();
  twist_.linear.y = world_linear.y();
  twist_.linear.z = world_linear.z();

  Eigen::Vector3d body_angular(body_twist.angular.x, body_twist.angular.y, body_twist.angular.z);
  Eigen::Vector3d world_angular(inverse_rotation_matrix * body_angular);
  twist_.angular.x = world_angular.x();
  twist_.angular.y = world_angular.y();
  twist_.angular.z = world_angular.z();
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

  // We limit the input velocities to +-100. Required for numeric stability in case of collisions,
  // where velocities returned by Gazebo can be very high.
  limit(drag_model_->u, -100.0, 100.0);

  // convert input to body coordinates
  Eigen::Quaterniond orientation(orientation_.w, orientation_.x, orientation_.y, orientation_.z);
  Eigen::Matrix<double,3,3> rotation_matrix(orientation.toRotationMatrix());
  Eigen::Map<Eigen::Vector3d> linear(&(drag_model_->u[0]));
  Eigen::Map<Eigen::Vector3d> angular(&(drag_model_->u[3]));
  linear  = rotation_matrix * linear;
  angular = rotation_matrix * angular;

  ROS_DEBUG_STREAM_NAMED("quadrotor_aerodynamics", "aerodynamics.twist:  " << PrintVector<double>(drag_model_->u.begin(), drag_model_->u.begin() + 6));
  checknan(drag_model_->u, "drag model input");

  // update drag model
  f(drag_model_->u.data(), dt, drag_model_->y.data());

  ROS_DEBUG_STREAM_NAMED("quadrotor_aerodynamics", "aerodynamics.force:  " << PrintVector<double>(drag_model_->y.begin() + 0, drag_model_->y.begin() + 3));
  ROS_DEBUG_STREAM_NAMED("quadrotor_aerodynamics", "aerodynamics.torque: " << PrintVector<double>(drag_model_->y.begin() + 3, drag_model_->y.begin() + 6));
  checknan(drag_model_->y, "drag model output");

  // drag_model_ gives us inverted vectors!
  wrench_.force.x  = -( drag_model_->y[0]);
  wrench_.force.y  = -(-drag_model_->y[1]);
  wrench_.force.z  = -(-drag_model_->y[2]);
  wrench_.torque.x = -( drag_model_->y[3]);
  wrench_.torque.y = -(-drag_model_->y[4]);
  wrench_.torque.z = -(-drag_model_->y[5]);
}

} // namespace hector_quadrotor_model
