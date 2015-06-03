#ifndef HECTOR_QUADROTOR_INTERFACE_LIMITERS_H
#define HECTOR_QUADROTOR_INTERFACE_LIMITERS_H

#include "ros/node_handle.h"
#include <limits>
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "hector_uav_msgs/AttitudeCommand.h"
#include "hector_uav_msgs/ThrustCommand.h"
#include "hector_uav_msgs/YawrateCommand.h"

namespace hector_quadrotor_interface
{

  using namespace geometry_msgs;
  using namespace hector_uav_msgs;

  template<typename T>
  class FieldLimiter
  {
  public:
    FieldLimiter(ros::NodeHandle nh, std::string field)
    {
      nh.param<T>(field + "/max", max_, std::numeric_limits<T>::max());
      nh.param<T>(field + "/min", min_, -max_);
      ROS_INFO_STREAM("limits " << nh.getNamespace() + "/" + field << " initialized " << field << " with min " << min_ << " and max " << max_);
    }

    T limit(const T &value)
    {
      return std::max(min_, std::min(max_, value));
    }

    FieldLimiter()
    {
    };
    T min_, max_;

  };

  class PointLimiter
  {
  public:
    PointLimiter(ros::NodeHandle nh, std::string field)
        : nh_(ros::NodeHandle(nh, field)), x_(nh_, "x"), y_(nh_, "y"), z_(nh_, "z")
    {
    }

    Point limit(const Point &input)
    {
      Point output;
      output.x = x_.limit(input.x);
      output.y = y_.limit(input.y);
      output.z = z_.limit(input.z);
      return output;
    }

    ros::NodeHandle nh_;
    FieldLimiter<double> x_, y_, z_;
  };

  class Vector3Limiter
  {
  public:
    Vector3Limiter(ros::NodeHandle nh, std::string field)
        : nh_(ros::NodeHandle(nh, field)), x_(nh_, "x"), y_(nh_, "y"), z_(nh_, "z")
    {
    }

    Vector3 limit(const Vector3 &input)
    {
      Vector3 output;
      output.x = x_.limit(input.x);
      output.y = y_.limit(input.y);
      output.z = z_.limit(input.z);
      return output;
    }

    ros::NodeHandle nh_;
    FieldLimiter<double> x_, y_, z_;
  };


  class TwistLimiter
  {
  public:
    TwistLimiter(ros::NodeHandle nh, std::string field)
        : linear_(ros::NodeHandle(nh, field), "linear"), angular_(ros::NodeHandle(nh, field), "angular")
    {
    }

    Twist limit(const Twist &input)
    {
      Twist output;
      output.linear = linear_.limit(input.linear);
      output.angular = angular_.limit(input.angular);
      return output;
    }

    Vector3Limiter linear_, angular_;
  };

  class WrenchLimiter
  {
  public:
    WrenchLimiter(ros::NodeHandle nh, std::string field)
        : force_(ros::NodeHandle(nh, field), "force"), torque_(ros::NodeHandle(nh, field), "torque")
    {
    }

    Wrench limit(const Wrench &input)
    {
      Wrench output;
      output.force = force_.limit(input.force);
      output.torque = torque_.limit(input.torque);
      return output;
    }

    Vector3Limiter force_, torque_;
  };

  class AttitudeCommandLimiter
  {
  public:
    AttitudeCommandLimiter(ros::NodeHandle nh, std::string field)
        : roll_(ros::NodeHandle(nh, field), "x"), pitch_(ros::NodeHandle(nh, field), "y")
    {
    }

    AttitudeCommand limit(const AttitudeCommand &input)
    {
      AttitudeCommand output;
      output.header = input.header;
      output.roll = roll_.limit(input.roll);
      output.pitch = pitch_.limit(input.pitch);
      return output;
    }

    FieldLimiter<double> roll_, pitch_;
  };

  class YawrateCommandLimiter
  {

  public:
    YawrateCommandLimiter(ros::NodeHandle nh, std::string field)
        : turnrate_(ros::NodeHandle(nh, field), "z")
    {
    }

    YawrateCommand limit(const YawrateCommand &input)
    {
      YawrateCommand output;
      output.header = input.header;
      output.turnrate = turnrate_.limit(input.turnrate);
      return output;
    }

    FieldLimiter<double> turnrate_;
  };

  class ThrustCommandLimiter
  {
  public:
    ThrustCommandLimiter(ros::NodeHandle nh, std::string field)
        : thrust_(ros::NodeHandle(nh, field), "z")
    {
    }

    ThrustCommand limit(const ThrustCommand &input)
    {
      ThrustCommand output;
      output.header = input.header;
      output.thrust = thrust_.limit(input.thrust);
      return output;
    }

    FieldLimiter<double> thrust_;
  };

}

#endif //  HECTOR_QUADROTOR_INTERFACE_LIMITERS_H
