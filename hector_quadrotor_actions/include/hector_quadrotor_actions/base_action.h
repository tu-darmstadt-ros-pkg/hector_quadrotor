#ifndef HECTOR_QUADROTOR_ACTIONS_BASE_ACTION_H
#define HECTOR_QUADROTOR_ACTIONS_BASE_ACTION_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <hector_quadrotor_interface/helpers.h>
#include "geometry_msgs/PoseStamped.h"
#include <hector_uav_msgs/EnableMotors.h>

namespace hector_quadrotor_actions
{

  template <class ActionSpec>
  class BaseActionServer
  {
  public:

    typedef typename actionlib::SimpleActionServer<ActionSpec> ActionServer;

    BaseActionServer(ros::NodeHandle nh, std::string server_name, typename ActionServer::ExecuteCallback callback)
        : pose_sub_(nh, "pose"),
          as_(boost::make_shared<ActionServer>(nh, server_name, callback, false))
    {
      nh.param<double>("connection_timeout", connection_timeout_, 10.0);

      server_name_ = nh.resolveName(server_name);

      motor_enable_service_ = nh.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors");
      if(!motor_enable_service_.waitForExistence()){
        ROS_ERROR_STREAM("Could not connect to " << nh.resolveName("enable_motors"));
      }

      start_timer_ = nh.createTimer(ros::Duration(0.1), boost::bind(&BaseActionServer::startCb, this));
    }

    /*
     * Guaranteed to be available after server has started
     */
    geometry_msgs::PoseStampedConstPtr getPose()
    {
      return pose_sub_.get();
    }

    boost::shared_ptr<ActionServer> get() { return as_; };

    bool enableMotors(bool enable)
    {
      hector_uav_msgs::EnableMotors srv;
      srv.request.enable = enable;
      return motor_enable_service_.call(srv);
    }

  private:

    void startCb()
    {
      if (!pose_sub_.get())
      {
        ROS_INFO_STREAM_THROTTLE(1.0, "Waiting for position state to be available before starting " << server_name_ <<
                                                                                                       " server");
      }
      else
      {
        as_->start();
        ROS_INFO_STREAM("Server " << server_name_ << " started");
        start_timer_.stop();
      }
    }

    boost::shared_ptr<ActionServer> as_;

    std::string server_name_;
    ros::Timer start_timer_;
    hector_quadrotor_interface::PoseSubscriberHelper pose_sub_;
    ros::ServiceClient motor_enable_service_;

    double connection_timeout_;

  };

}

#endif //HECTOR_QUADROTOR_ACTIONS_BASE_ACTION_H
