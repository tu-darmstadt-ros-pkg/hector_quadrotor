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
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

class quadrotor_tf {
private:
  ros::NodeHandle node_handle_;
  tf::TransformBroadcaster tf_broadcaster_;
  ros::Subscriber subscriber_;

  std::string topic_;
  std::string frame_id_;
  std::string child_frame_id_;

  std::vector<geometry_msgs::TransformStamped> transforms_;

public:
  quadrotor_tf()
    : topic_("state")
    , transforms_(3)
  {
    ros::NodeHandle params("~");
    params.getParam("topic", topic_);
    params.getParam("frame_id", frame_id_);
    params.getParam("child_frame_id", child_frame_id_);

    transforms_[0].header.frame_id = frame_id_;
    transforms_[0].child_frame_id  = "base_footprint";
    transforms_[1].header.frame_id = frame_id_;
    transforms_[1].child_frame_id  = "base_stabilized";
    transforms_[2].header.frame_id = frame_id_;
    transforms_[2].child_frame_id  = child_frame_id_;

    subscriber_ = node_handle_.subscribe<nav_msgs::Odometry>(topic_, 10, boost::bind(&quadrotor_tf::stateCallback, this, _1));
  }
   ~quadrotor_tf() {}

  void stateCallback(const nav_msgs::OdometryConstPtr& state)
  {
    double yaw = tf::getYaw(state->pose.pose.orientation);

    transforms_[0].transform.translation.x = state->pose.pose.position.x;
    transforms_[0].transform.translation.y = state->pose.pose.position.y;
    transforms_[0].transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, yaw);
    transforms_[1].transform.translation.x = state->pose.pose.position.x;
    transforms_[1].transform.translation.y = state->pose.pose.position.y;
    transforms_[1].transform.translation.z = state->pose.pose.position.z;
    transforms_[1].transform.rotation = transforms_[0].transform.rotation;
    transforms_[2].transform.translation.x = state->pose.pose.position.x;
    transforms_[2].transform.translation.y = state->pose.pose.position.y;
    transforms_[2].transform.translation.z = state->pose.pose.position.z;
    transforms_[2].transform.rotation = state->pose.pose.orientation;

    transforms_[0].header.stamp = state->header.stamp;
    transforms_[1].header.stamp = state->header.stamp;
    transforms_[2].header.stamp = state->header.stamp;

    if (frame_id_.empty()) {
      transforms_[0].header.frame_id = transforms_[1].header.frame_id = transforms_[2].header.frame_id = state->header.frame_id;
    }
    if (child_frame_id_.empty()) {
      transforms_[2].child_frame_id = (!state->child_frame_id.empty() ? state->child_frame_id : "base_link");
    }

    tf_broadcaster_.sendTransform(transforms_);
  }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "quadrotor_tf");
  quadrotor_tf quadrotor_tf;
  ros::spin();
  return 0;
}

