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


#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <hector_uav_msgs/YawrateCommand.h>
#include <hector_uav_msgs/ThrustCommand.h>
#include <hector_uav_msgs/AttitudeCommand.h>

namespace hector_quadrotor
{

class Teleop
{
private:
  ros::NodeHandle node_handle_;
  ros::Subscriber joy_subscriber_;

  ros::Publisher velocity_publisher_, attitude_publisher_, yawrate_publisher_, thrust_publisher_;
  geometry_msgs::Twist velocity_;
  hector_uav_msgs::AttitudeCommand attitude_;
  hector_uav_msgs::ThrustCommand thrust_;
  hector_uav_msgs::YawrateCommand yawrate_;

  struct Axis
  {
    int axis;
    double max;
  };

  struct Button
  {
    int button;
  };

  struct
  {
    Axis x;
    Axis y;
    Axis z;
    Axis yaw;
  } axes_;

  struct
  {
    Button slow;
  } buttons_;

  double slow_factor_;

public:
  Teleop()
  {
    ros::NodeHandle params("~");

    params.param<int>("x_axis", axes_.x.axis, 4);
    params.param<int>("y_axis", axes_.y.axis, 3);
    params.param<int>("z_axis", axes_.z.axis, 2);
    params.param<int>("yaw_axis", axes_.yaw.axis, 1);

    params.param<double>("yaw_velocity_max", axes_.yaw.max, 90.0 * M_PI / 180.0);
    params.param<int>("slow_button", buttons_.slow.button, 1);
    params.param<double>("slow_factor", slow_factor_, 0.2);

    std::string control_mode_str;
    params.param<std::string>("control_mode", control_mode_str, "twist");

    if (control_mode_str == "twist")
    {
      params.param<double>("x_velocity_max", axes_.x.max, 2.0);
      params.param<double>("y_velocity_max", axes_.y.max, 2.0);
      params.param<double>("z_velocity_max", axes_.z.max, 2.0);

      joy_subscriber_ = node_handle_.subscribe<sensor_msgs::Joy>("joy", 1, boost::bind(&Teleop::joyTwistCallback, this, _1));
      velocity_publisher_ = node_handle_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    }
    else if (control_mode_str == "attitude")
    {
      params.param<double>("x_roll_max", axes_.x.max, 0.35);
      params.param<double>("y_pitch_max", axes_.y.max, 0.35);
      params.param<double>("z_thrust_max", axes_.z.max, 25.0);
      joy_subscriber_ = node_handle_.subscribe<sensor_msgs::Joy>("joy", 1, boost::bind(&Teleop::joyAttitudeCallback, this, _1));
      attitude_publisher_ = node_handle_.advertise<hector_uav_msgs::AttitudeCommand>("command/attitude", 10);
      yawrate_publisher_ = node_handle_.advertise<hector_uav_msgs::YawrateCommand>("command/yawrate", 10);
      thrust_publisher_ = node_handle_.advertise<hector_uav_msgs::ThrustCommand>("command/thrust", 10);
    }

  }

  ~Teleop()
  {
    stop();
  }

  void joyTwistCallback(const sensor_msgs::JoyConstPtr &joy)
  {
    velocity_.linear.x = getAxis(joy, axes_.x);
    velocity_.linear.y = getAxis(joy, axes_.y);
    velocity_.linear.z = getAxis(joy, axes_.z);
    velocity_.angular.z = getAxis(joy, axes_.yaw);
    if (getButton(joy, buttons_.slow.button))
    {
      velocity_.linear.x *= slow_factor_;
      velocity_.linear.y *= slow_factor_;
      velocity_.linear.z *= slow_factor_;
      velocity_.angular.z *= slow_factor_;
    }
    velocity_publisher_.publish(velocity_);
  }

  void joyAttitudeCallback(const sensor_msgs::JoyConstPtr &joy)
  {
    attitude_.roll = getAxis(joy, axes_.x);
    attitude_.pitch = getAxis(joy, axes_.y);
    attitude_publisher_.publish(attitude_);

    thrust_.thrust = getAxis(joy, axes_.z);
    thrust_publisher_.publish(thrust_);

    yawrate_.turnrate = getAxis(joy, axes_.yaw);
    if (getButton(joy, buttons_.slow.button))
    {
      yawrate_.turnrate *= slow_factor_;
    }
    yawrate_publisher_.publish(yawrate_);
  }

  sensor_msgs::Joy::_axes_type::value_type getAxis(const sensor_msgs::JoyConstPtr &joy, Axis axis)
  {
    if (axis.axis == 0)
    {return 0;}
    sensor_msgs::Joy::_axes_type::value_type sign = 1.0;
    if (axis.axis < 0)
    {
      sign = -1.0;
      axis.axis = -axis.axis;
    }
    if ((size_t) axis.axis > joy->axes.size())
    {return 0;}
    return sign * joy->axes[axis.axis - 1] * axis.max;
  }

  sensor_msgs::Joy::_buttons_type::value_type getButton(const sensor_msgs::JoyConstPtr &joy, int button)
  {
    if (button <= 0)
    {return 0;}
    if ((size_t) button > joy->buttons.size())
    {return 0;}
    return joy->buttons[button - 1];
  }

  void stop()
  {
    if(velocity_publisher_.getNumSubscribers() > 0)
    {
      velocity_ = geometry_msgs::Twist();
      velocity_publisher_.publish(velocity_);
    }
    if(attitude_publisher_.getNumSubscribers() > 0)
    {
      attitude_ = hector_uav_msgs::AttitudeCommand();
      attitude_publisher_.publish(attitude_);
    }
    if(thrust_publisher_.getNumSubscribers() > 0)
    {
      thrust_ = hector_uav_msgs::ThrustCommand();
      thrust_publisher_.publish(thrust_);
    }
    if(yawrate_publisher_.getNumSubscribers() > 0)
    {
      yawrate_ = hector_uav_msgs::YawrateCommand();
      yawrate_publisher_.publish(yawrate_);
    }

  }
};

} // namespace hector_quadrotor

int main(int argc, char **argv)
{
  ros::init(argc, argv, "quadrotor_teleop");

  hector_quadrotor::Teleop teleop;
  ros::spin();

  return 0;
}
