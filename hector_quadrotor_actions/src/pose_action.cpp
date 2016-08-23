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

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <hector_uav_msgs/PoseAction.h>
#include <geometry_msgs/PoseStamped.h>

#include <hector_quadrotor_interface/helpers.h>
#include <hector_quadrotor_actions/base_action.h>

namespace hector_quadrotor_actions
{

class PoseActionServer
{
public:
  PoseActionServer(ros::NodeHandle nh)
      : pose_server_(nh, "action/pose", boost::bind(&PoseActionServer::poseActionCb, this, _1))
  {
    pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("command/pose", 1);

    nh.param<double>("dist_tolerance", dist_tolerance_, 0.05);
    nh.param<double>("yaw_tolerance", yaw_tolerance_, 0.35);
    nh.param<double>("time_in_tolerance", time_in_tolerance_, 1.0);
    nh.param<double>("action_frequency", frequency_, 10.0);
    nh.param<double>("action_timeout", action_timeout_, 30.0);
  }

  void poseActionCb(const hector_uav_msgs::PoseGoalConstPtr &goal)
  {
    pose_server_.enableMotors(true);

    geometry_msgs::PoseStamped pose = goal->target_pose;

    ros::Rate r(frequency_);
    ros::Time start = ros::Time::now();
    ros::Time last_time_out_of_tolerance_ = ros::Time::now();

    while (ros::ok() && pose_server_.get()->isActive())
    {
      if (pose_server_.get()->isPreemptRequested())
      {

        if (!pose_server_.get()->isNewGoalAvailable())
        {
          //Stop moving
          pose_pub_.publish(pose_server_.getPose());
        }
        pose_server_.get()->setPreempted();
        return;
      }

      pose.header.stamp = ros::Time::now();
      pose_pub_.publish(pose);

      hector_uav_msgs::PoseFeedback feedback;
      feedback.current_pose = *pose_server_.getPose();
      pose_server_.get()->publishFeedback(feedback);

      if(!hector_quadrotor_interface::poseWithinTolerance(feedback.current_pose.pose, goal->target_pose.pose, dist_tolerance_, yaw_tolerance_))
      {
        last_time_out_of_tolerance_ = ros::Time::now();
      }
      else if (last_time_out_of_tolerance_ + ros::Duration(time_in_tolerance_ ) < ros::Time::now())
      {
        pose_server_.get()->setSucceeded();
        return;
      }

      if (ros::Time::now() > start + ros::Duration(action_timeout_))
      {
        pose_server_.get()->setAborted();
        return;
      }

      ros::spinOnce();
      r.sleep();
    }
  }

private:
  hector_quadrotor_actions::BaseActionServer<hector_uav_msgs::PoseAction> pose_server_;
  ros::Publisher pose_pub_;

  double frequency_, dist_tolerance_, yaw_tolerance_, action_timeout_, time_in_tolerance_;
};

} // namespace hector_quadrotor_actions


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_action");

  ros::NodeHandle nh;
  hector_quadrotor_actions::PoseActionServer server(nh);

  ros::spin();

  return 0;
}
