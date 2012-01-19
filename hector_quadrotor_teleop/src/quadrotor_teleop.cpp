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

namespace hector_quadrotor {

class Teleop
{
private:
  ros::NodeHandle node_handle_;
  ros::Subscriber joy_subscriber_;
  ros::Publisher velocity_publisher_;
  geometry_msgs::Twist velocity_;

  struct Axis
  {
    int axis;
    double max;
  };

  struct Button
  {
    unsigned int button;
  };

  struct {
    Axis x;
    Axis y;
    Axis z;
    Axis yaw;
  } axes_;

  struct
  {
  } buttons_;

public:
  Teleop()
  {
    ros::NodeHandle params("~");
    axes_.x.axis = 999;
    axes_.x.max = 2.0;
    axes_.y.axis = 999;
    axes_.y.max = 2.0;
    axes_.z.axis = 999;
    axes_.z.max = 2.0;
    axes_.yaw.axis = 999;
    axes_.yaw.max = 90.0*M_PI/180.0;
    params.getParam("x_axis", axes_.x.axis);
    params.getParam("y_axis", axes_.y.axis);
    params.getParam("z_axis", axes_.z.axis);
    params.getParam("yaw_axis", axes_.yaw.axis);
    params.getParam("x_velocity_max", axes_.x.max);
    params.getParam("y_velocity_max", axes_.y.max);
    params.getParam("z_velocity_max", axes_.z.max);
    params.getParam("yaw_velocity_max", axes_.yaw.max);

    joy_subscriber_ = node_handle_.subscribe<sensor_msgs::Joy>("joy", 1, boost::bind(&Teleop::joyCallback, this, _1));
    velocity_publisher_ = node_handle_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  }

  ~Teleop()
  {
    stop();
  }

  void joyCallback(const sensor_msgs::JoyConstPtr& joy)
  {
    velocity_.linear.x  = (joy->axes.size() > (size_t)axes_.x.axis)   ? joy->axes[axes_.x.axis]   * axes_.x.max   : 0.0;
    velocity_.linear.y  = (joy->axes.size() > (size_t)axes_.y.axis)   ? joy->axes[axes_.y.axis]   * axes_.y.max   : 0.0;
    velocity_.linear.z  = (joy->axes.size() > (size_t)axes_.z.axis)   ? joy->axes[axes_.z.axis]   * axes_.z.max   : 0.0;
    velocity_.angular.z = (joy->axes.size() > (size_t)axes_.yaw.axis) ? joy->axes[axes_.yaw.axis] * axes_.yaw.max : 0.0;
    velocity_publisher_.publish(velocity_);
  }

  void stop()
  {
    velocity_ = geometry_msgs::Twist();
    velocity_publisher_.publish(velocity_);
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
