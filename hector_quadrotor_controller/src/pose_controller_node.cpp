//=================================================================================================
// Copyright (c) 2013, Johannes Meyer, TU Darmstadt
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

#include <hector_quadrotor_controller/quadrotor_hardware.h>
#include <hector_quadrotor_controller/pose_controller.h>

#include <ros/ros.h>

using namespace hector_quadrotor_controller;

QuadrotorHardware hw;
ControllerBase *pose_controller;

int main(int argc, char **argv)
{
  // initialize ROS node, publishers and subscribers
  ros::init(argc, argv, "pose_controller");
  ros::NodeHandle nh;
  ros::NodeHandle controller_nh("~/pose");
  ros::Subscriber stateSubscriber = nh.subscribe("state", 1, &QuadrotorHardware::setState, &hw);
  ros::Publisher commandPublisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, false);

  // initialize controller
  pose_controller = new PoseController();
  std::set<std::string> claimed_resources;
  if (!pose_controller->initRequest(&hw, nh, controller_nh, claimed_resources))
  {
    ROS_ERROR("Failed to initialize controller!");
    return 1;
  }

  boost::shared_ptr<TwistCommandHandle> command = hw.get<QuadrotorInterface>()->getOutput<TwistCommandHandle>("twist");
  if (!command)
  {
    ROS_ERROR("Could not connect to twist output!");
    return 1;
  }

  // run control loop
  double p_control_rate = 10.0;
  controller_nh.getParam("frequency", p_control_rate);
  ros::Rate rate(p_control_rate);
  while(ros::ok()) {
    ros::spinOnce();
    pose_controller->updateRequest(ros::Time::now(), rate.cycleTime());
    if (command->connected())
      commandPublisher.publish(command->getCommand());
    rate.sleep();
  }
  return 0;
}
