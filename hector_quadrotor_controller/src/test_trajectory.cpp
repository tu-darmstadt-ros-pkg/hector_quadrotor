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
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_trajectory");
  ros::NodeHandle nh;
  ros::Publisher publisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  geometry_msgs::Twist twist;

  publisher.publish(twist);
  ros::Duration(5.0).sleep();

  double speed = 3.0;
  double interval = 3.0;

  twist.linear.z = 2.0;
  publisher.publish(twist);
  ros::Duration(interval).sleep();

  twist.linear.z = 0.0;
  publisher.publish(twist);
  ros::Duration(interval).sleep();

  twist.linear.x = speed;
  publisher.publish(twist);
  ros::Duration(interval).sleep();

  twist.linear.x = 0.0;
  publisher.publish(twist);
  ros::Duration(interval).sleep();

  twist.linear.y = speed;
  publisher.publish(twist);
  ros::Duration(interval).sleep();

  twist.linear.y = 0.0;
  publisher.publish(twist);
  ros::Duration(interval).sleep();

  twist.linear.x = -speed;
  publisher.publish(twist);
  ros::Duration(interval).sleep();

  twist.linear.x = 0.0;
  publisher.publish(twist);
  ros::Duration(interval).sleep();

  twist.linear.y = -speed;
  publisher.publish(twist);
  ros::Duration(interval).sleep();

  twist.linear.y = 0.0;
  publisher.publish(twist);
  ros::Duration(interval).sleep();

  twist.linear.z = -1.0;
  publisher.publish(twist);
  ros::Duration(interval).sleep();

  twist.linear.z = 0.0;
  publisher.publish(twist);
  ros::Duration(interval).sleep();

  return 0;
}
