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
    int button;
  };

  struct {
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

    axes_.x.axis = 0;
    axes_.x.max = 2.0;
    axes_.y.axis = 0;
    axes_.y.max = 2.0;
    axes_.z.axis = 0;
    axes_.z.max = 2.0;
    axes_.yaw.axis = 0;
    axes_.yaw.max = 90.0*M_PI/180.0;
    buttons_.slow.button = 0;
    slow_factor_ = 0.2;

    params.getParam("x_axis", axes_.x.axis);
    params.getParam("y_axis", axes_.y.axis);
    params.getParam("z_axis", axes_.z.axis);
    params.getParam("yaw_axis", axes_.yaw.axis);
    params.getParam("x_velocity_max", axes_.x.max);
    params.getParam("y_velocity_max", axes_.y.max);
    params.getParam("z_velocity_max", axes_.z.max);
    params.getParam("yaw_velocity_max", axes_.yaw.max);
    params.getParam("slow_button", buttons_.slow.button);
    params.getParam("slow_factor", slow_factor_);

    joy_subscriber_ = node_handle_.subscribe<sensor_msgs::Joy>("joy", 1, boost::bind(&Teleop::joyCallback, this, _1));
    velocity_publisher_ = node_handle_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  }

  ~Teleop()
  {
    stop();
  }

  void joyCallback(const sensor_msgs::JoyConstPtr& joy)
  {
    velocity_.linear.x  = getAxis(joy, axes_.x.axis)   * axes_.x.max;
    velocity_.linear.y  = getAxis(joy, axes_.y.axis)   * axes_.y.max;
    velocity_.linear.z  = getAxis(joy, axes_.z.axis)   * axes_.z.max;
    velocity_.angular.z = getAxis(joy, axes_.yaw.axis) * axes_.yaw.max;
    if (getButton(joy, buttons_.slow.button)) {
      velocity_.linear.x  *= slow_factor_;
      velocity_.linear.y  *= slow_factor_;
      velocity_.linear.z  *= slow_factor_;
      velocity_.angular.z *= slow_factor_;
    }
    velocity_publisher_.publish(velocity_);
  }

  sensor_msgs::Joy::_axes_type::value_type getAxis(const sensor_msgs::JoyConstPtr& joy, int axis)
  {
    if (axis == 0) return 0;
    sensor_msgs::Joy::_axes_type::value_type sign = 1.0;
    if (axis < 0) { sign = -1.0; axis = -axis; }
    if ((size_t)axis > joy->axes.size()) return 0;
    return sign * joy->axes[axis - 1];
  }

  sensor_msgs::Joy::_buttons_type::value_type getButton(const sensor_msgs::JoyConstPtr& joy, int button)
  {
    if (button <= 0) return 0;
    if ((size_t)button > joy->axes.size()) return 0;
    return joy->buttons[button - 1];
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
