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

#ifndef HECTOR_QUADROTOR_MODEL_QUADROTOR_AERODYNAMICS_H
#define HECTOR_QUADROTOR_MODEL_QUADROTOR_AERODYNAMICS_H

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Wrench.h>

#include <ros/node_handle.h>

#include <boost/thread/mutex.hpp>

namespace hector_quadrotor_model
{

class QuadrotorAerodynamics {
public:
  QuadrotorAerodynamics();
  ~QuadrotorAerodynamics();

  bool configure(const ros::NodeHandle &param = ros::NodeHandle("~"));
  void reset();
  void update(double dt);

  void setOrientation(const geometry_msgs::Quaternion& orientation);
  void setTwist(const geometry_msgs::Twist& twist);
  void setBodyTwist(const geometry_msgs::Twist& twist);
  void setWind(const geometry_msgs::Vector3& wind);

  const geometry_msgs::Wrench& getWrench() const { return wrench_; }

  void f(const double uin[6], double dt, double y[6]) const;

private:
  geometry_msgs::Quaternion orientation_;
  geometry_msgs::Twist twist_;
  geometry_msgs::Vector3 wind_;

  geometry_msgs::Wrench wrench_;

  boost::mutex mutex_;

  class DragModel;
  DragModel *drag_model_;
};

}

#endif // HECTOR_QUADROTOR_MODEL_QUADROTOR_AERODYNAMICS_H
