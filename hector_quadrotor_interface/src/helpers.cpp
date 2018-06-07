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

#include <hector_quadrotor_interface/helpers.h>
#include <urdf_parser/urdf_parser.h>

namespace hector_quadrotor_interface
{

bool getMassAndInertia(const ros::NodeHandle &nh, double &mass, double inertia[3])
{

  std::string robot_description;
  if (!nh.getParam("robot_description", robot_description))
  {
    ROS_ERROR_STREAM("getMassAndInertia() couldn't find URDF at " << nh.getNamespace() << "/robot_description");
    return false;
  }

  urdf::ModelInterfaceSharedPtr model;
  try
  {
    model = urdf::parseURDF(robot_description);
  }
  catch (std::exception ex)
  {
    ROS_ERROR_STREAM(
        "getMassAndInertia() couldn't parse URDF at " << nh.getNamespace() << "/robot_description: " << ex.what());
    return false;
  }

  urdf::InertialSharedPtr inertial = model->getRoot()->inertial;
  if (!inertial || !inertial->mass || !inertial->ixx || !inertial->iyy || !inertial->izz)
  {
    ROS_ERROR_STREAM(
        "getMassAndInertia() requires inertial information stored on the root link " << nh.getNamespace() <<
        "/robot_description");
    return false;
  }

  mass = inertial->mass;
  inertia[0] = inertial->ixx;
  inertia[1] = inertial->iyy;
  inertia[2] = inertial->izz;
  return true;
}

bool poseWithinTolerance(const geometry_msgs::Pose &pose_current, const geometry_msgs::Pose &pose_target,
                         const double dist_tolerance, const double yaw_tolerance)
{

  if(yaw_tolerance > 0.0)
  {
    double yaw_current, yaw_target;
    tf2::Quaternion q;
    double temp;
    tf2::fromMsg(pose_current.orientation, q);
    tf2::Matrix3x3(q).getRPY(temp, temp, yaw_current);
    tf2::fromMsg(pose_target.orientation, q);
    tf2::Matrix3x3(q).getRPY(temp, temp, yaw_target);

    double yaw_error = yaw_current - yaw_target;
    // detect wrap around pi and compensate
    if (yaw_error > M_PI)
    {
      yaw_error -= 2 * M_PI;
    }
    else if (yaw_error < -M_PI)
    {
      yaw_error += 2 * M_PI;
    }
    if (std::abs(yaw_error) > yaw_tolerance)
    {
      ROS_DEBUG_STREAM("Waiting for yaw " << std::abs(yaw_current - yaw_target));
      return false;
    }

  }

  if(dist_tolerance > 0.0)
  {
    tf2::Vector3 v_current(pose_current.position.x, pose_current.position.y, pose_current.position.z);
    tf2::Vector3 v_target(pose_target.position.x, pose_target.position.y, pose_target.position.z);
    if ((v_current - v_target).length() > dist_tolerance)
    {
      ROS_DEBUG_STREAM("Waiting for distance " << (v_current - v_target).length());
      return false;
    }
  }

  return true;

}

} // namespace hector_quadrotor_interface
