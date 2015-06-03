#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <hector_uav_msgs/PoseAction.h>
#include <hector_uav_msgs/TakeoffAction.h>
#include <hector_quadrotor_interface/helpers.h>
#include <hector_quadrotor_actions/base_action.h>

namespace hector_quadrotor_actions
{

  class TakeoffActionServer{

  public:

    TakeoffActionServer(ros::NodeHandle nh)
        : takeoff_server_(nh, "action/takeoff", boost::bind(&TakeoffActionServer::takeoffActionCb, this, _1)),
          pose_client_(nh, "action/pose")
    {
      nh.param<double>("action_frequency", frequency_, 10.0);
      nh.param<double>("takeoff_height", takeoff_height_, 0.3);
      nh.param<double>("connection_timeout", connection_timeout_, 10.0);
      nh.param<double>("action_timout", action_timeout_, 30.0);

      if(!pose_client_.waitForServer(ros::Duration(connection_timeout_))){
        ROS_ERROR_STREAM("Could not connect to " << nh.resolveName("action/pose"));
      }
    }

    void takeoffActionCb(const hector_uav_msgs::TakeoffGoalConstPtr &goal)
    {

      if(takeoff_server_.enableMotors(true))
      {

        hector_uav_msgs::PoseGoal pose_goal;
        pose_goal.target_pose = *takeoff_server_.getPose();
        pose_goal.target_pose.pose.position.z = takeoff_height_;
        pose_client_.sendGoal(pose_goal);
        pose_client_.waitForResult(ros::Duration(action_timeout_));

        if (pose_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          ROS_WARN("Takeoff succeeded");
          takeoff_server_.get()->setSucceeded();
          return;
        }
      }
      ROS_WARN("Takeoff failed");
      takeoff_server_.get()->setAborted();
    }

  private:

    actionlib::SimpleActionClient<hector_uav_msgs::PoseAction> pose_client_;
    hector_quadrotor_actions::BaseActionServer<hector_uav_msgs::TakeoffAction> takeoff_server_;
    ros::Publisher pose_pub_;

    double frequency_, takeoff_height_, connection_timeout_, action_timeout_;

  };

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "takeoff_action");

  ros::NodeHandle nh;
  hector_quadrotor_actions::TakeoffActionServer server(nh);

  ros::spin();

  return 0;
}
