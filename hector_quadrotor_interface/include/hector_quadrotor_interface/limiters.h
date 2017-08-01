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

#ifndef HECTOR_QUADROTOR_INTERFACE_LIMITERS_H
#define HECTOR_QUADROTOR_INTERFACE_LIMITERS_H

#include <ros/node_handle.h>
#include <limits>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <hector_uav_msgs/AttitudeCommand.h>
#include <hector_uav_msgs/ThrustCommand.h>
#include <hector_uav_msgs/YawrateCommand.h>

namespace hector_quadrotor_interface
{

using namespace geometry_msgs;
using namespace hector_uav_msgs;

template <typename T>
class FieldLimiter
{
public:
  FieldLimiter()
    : min_(-std::numeric_limits<T>::infinity())
    , max_( std::numeric_limits<T>::infinity())
  {}

  void init(const ros::NodeHandle &nh, const std::string &field = std::string())
  {
    std::string prefix = !field.empty() ? field + "/" : "";
    if (nh.hasParam(prefix + "max") || nh.hasParam(prefix + "min")) {
      nh.param<T>(prefix + "max", max_, std::numeric_limits<T>::infinity());
      nh.param<T>(prefix + "min", min_, -max_);
      ROS_INFO_STREAM("Limits " << nh.getNamespace() + "/" + field << " initialized " << field << " with min " << min_ << " and max " << max_);
    }
  }

  T limit(const T &value)
  {
    return std::max(min_, std::min(max_, value));
  }

  T operator()(const T &value)
  {
    return limit(value);
  }

  T min_, max_;
};

class PointLimiter
{
public:
  void init(const ros::NodeHandle &nh, const std::string &field = std::string())
  {
    ros::NodeHandle field_nh(nh, field);
    x.init(field_nh, "x");
    y.init(field_nh, "y");
    z.init(field_nh, "z");
  }

  Point limit(const Point &input)
  {
    Point output;
    output.x = x.limit(input.x);
    output.y = y.limit(input.y);
    output.z = z.limit(input.z);
    return output;
  }

  Point operator()(const Point &input)
  {
    return limit(input);
  }

  FieldLimiter<double> x, y, z;
};

class Vector3Limiter
{
public:
  void init(const ros::NodeHandle &nh, const std::string &field = std::string())
  {
    ros::NodeHandle field_nh(nh, field);
    x.init(field_nh, "x");
    y.init(field_nh, "y");
    z.init(field_nh, "z");
    field_nh.param("max", absolute_maximum, std::numeric_limits<double>::infinity());
    field_nh.param("max_xy", absolute_maximum_xy, std::numeric_limits<double>::infinity());
  }

  Vector3 limit(const Vector3 &input)
  {
    Vector3 output;
    output.x = x.limit(input.x);
    output.y = y.limit(input.y);
    output.z = z.limit(input.z);

    double absolute_value_xy = sqrt(output.x * output.x + output.y * output.y);
    if (absolute_value_xy > absolute_maximum_xy) {
      output.x *= absolute_maximum_xy / absolute_value_xy;
      output.y *= absolute_maximum_xy / absolute_value_xy;
      output.z *= absolute_maximum_xy / absolute_value_xy;
    }

    double absolute_value = sqrt(output.x * output.x + output.y * output.y + output.z * output.z);
    if (absolute_value > absolute_maximum) {
      output.x *= absolute_maximum / absolute_value;
      output.y *= absolute_maximum / absolute_value;
      output.z *= absolute_maximum / absolute_value;
    }

    return output;
  }

  Vector3 operator()(const Vector3 &input)
  {
    return limit(input);
  }

  FieldLimiter<double> x, y, z;
  double absolute_maximum, absolute_maximum_xy;
};


class TwistLimiter
{
public:
  void init(const ros::NodeHandle &nh, const std::string &field = std::string())
  {
    ros::NodeHandle field_nh(nh, field);
    linear.init(field_nh, "linear");
    angular.init(field_nh, "angular");
  }

  Twist limit(const Twist &input)
  {
    Twist output;
    output.linear = linear.limit(input.linear);
    output.angular = angular.limit(input.angular);
    return output;
  }

  Twist operator()(const Twist &input)
  {
    return limit(input);
  }

  Vector3Limiter linear, angular;
};

class WrenchLimiter
{
public:
  void init(const ros::NodeHandle &nh, const std::string &field = std::string())
  {
    ros::NodeHandle field_nh(nh, field);
    force.init(field_nh, "force");
    torque.init(field_nh, "torque");
  }

  Wrench limit(const Wrench &input)
  {
    Wrench output;
    output.force = force.limit(input.force);
    output.torque = torque.limit(input.torque);
    return output;
  }

  Wrench operator()(const Wrench &input)
  {
    return limit(input);
  }

  Vector3Limiter force, torque;
};

class AttitudeCommandLimiter
{
public:
  void init(const ros::NodeHandle &nh, const std::string &field = std::string())
  {
    ros::NodeHandle field_nh(nh, field);
    roll.init(field_nh, "roll");
    pitch.init(field_nh, "pitch");
    field_nh.param("max_roll_pitch", absolute_max, std::numeric_limits<double>::infinity());
  }

  AttitudeCommand limit(const AttitudeCommand &input)
  {
    AttitudeCommand output;
    output.header = input.header;
    output.roll = roll.limit(input.roll);
    output.pitch = pitch.limit(input.pitch);

    double absolute_value = sqrt(output.roll * output.roll + output.pitch * output.pitch);
    if (absolute_value > absolute_max) {
      output.roll *= absolute_max / absolute_value;
      output.pitch *= absolute_max / absolute_value;
    }

    return output;
  }

  AttitudeCommand operator()(const AttitudeCommand &input)
  {
    return limit(input);
  }

  FieldLimiter<double> roll, pitch;
  double absolute_max;
};

class YawrateCommandLimiter
{

public:
  void init(const ros::NodeHandle &nh, const std::string &field = std::string())
  {
    turnrate.init(nh, field);
  }

  YawrateCommand limit(const YawrateCommand &input)
  {
    YawrateCommand output;
    output.header = input.header;
    output.turnrate = turnrate.limit(input.turnrate);
    return output;
  }

  YawrateCommand operator()(const YawrateCommand &input)
  {
    return limit(input);
  }

  FieldLimiter<double> turnrate;
};

class ThrustCommandLimiter
{
public:
  void init(const ros::NodeHandle &nh, const std::string &field = std::string())
  {
    thrust.init(nh, field);
  }

  ThrustCommand limit(const ThrustCommand &input)
  {
    ThrustCommand output;
    output.header = input.header;
    output.thrust = thrust.limit(input.thrust);
    return output;
  }

  ThrustCommand operator()(const ThrustCommand &input)
  {
    return limit(input);
  }

  FieldLimiter<double> thrust;
};

} // namespace hector_quadrotor_interface

#endif //  HECTOR_QUADROTOR_INTERFACE_LIMITERS_H
