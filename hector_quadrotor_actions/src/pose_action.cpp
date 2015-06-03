#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <hector_uav_msgs/PoseAction.h>

#include <hector_quadrotor_interface/helpers.h>
#include <hector_quadrotor_actions/base_action.h>

namespace hector_quadrotor_actions
{

  class PoseActionServer{

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

          if(!pose_server_.get()->isNewGoalAvailable())
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
        }else if (last_time_out_of_tolerance_ + ros::Duration(time_in_tolerance_ ) < ros::Time::now()){
          pose_server_.get()->setSucceeded();
          return;
        }

        if (ros::Time::now() > start + ros::Duration(action_timeout_)){
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

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "takeoff_action");

  ros::NodeHandle nh;
  hector_quadrotor_actions::PoseActionServer server(nh);

  ros::spin();

  return 0;
}
