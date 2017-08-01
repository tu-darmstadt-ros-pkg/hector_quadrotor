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
#include <hector_uav_msgs/LandingAction.h>
#include <hector_quadrotor_interface/helpers.h>
#include <hector_quadrotor_actions/base_action.h>

namespace hector_quadrotor_actions
{

class LandingActionServer
{
public:
  LandingActionServer(ros::NodeHandle nh)
      : landing_server_(nh, "action/landing", boost::bind(&LandingActionServer::landingActionCb, this, _1)),
        pose_client_(nh, "action/pose")
  {
    nh.param<double>("action_frequency", frequency_, 10.0);
    nh.param<double>("landing_height", landing_height_, 0.3);
    nh.param<double>("connection_timeout", connection_timeout_, 10.0);
    nh.param<double>("action_timout", action_timeout_, 30.0);

    if(!pose_client_.waitForServer(ros::Duration(connection_timeout_))){
      ROS_ERROR_STREAM("Could not connect to " << nh.resolveName("action/pose"));
    }
  }

  void landingActionCb(const hector_uav_msgs::LandingGoalConstPtr &goal)
  {

    hector_uav_msgs::PoseGoal pose_goal;
    if(!goal->landing_zone.header.frame_id.empty()){
      pose_goal.target_pose = goal->landing_zone;
    }else{
      pose_goal.target_pose = *landing_server_.getPose();
    }
    pose_goal.target_pose.pose.position.z = std::min(landing_height_, pose_goal.target_pose.pose.position.z);
    pose_client_.sendGoal(pose_goal);
    pose_client_.waitForResult(ros::Duration(action_timeout_));

    if(pose_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      if(landing_server_.enableMotors(false))
      {
        landing_server_.get()->setSucceeded();
        ROS_WARN("Landing succeeded");
        return;
      }
    }
    ROS_WARN("Landing failed");
    landing_server_.get()->setAborted();

  }

private:
  actionlib::SimpleActionClient<hector_uav_msgs::PoseAction> pose_client_;
  hector_quadrotor_actions::BaseActionServer<hector_uav_msgs::LandingAction> landing_server_;
  ros::Publisher pose_pub_;

  double frequency_, landing_height_, connection_timeout_, action_timeout_;
};

} // namespace hector_quadrotor_actions


int main(int argc, char **argv)
{
  ros::init(argc, argv, "landing_action");

  ros::NodeHandle nh;
  hector_quadrotor_actions::LandingActionServer server(nh);

  ros::spin();

  return 0;
}
