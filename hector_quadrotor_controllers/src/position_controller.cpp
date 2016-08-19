#include <controller_interface/controller.h>
#include <control_toolbox/pid.h>
#include <geometry_msgs/TwistStamped.h>
#include <hector_quadrotor_interface/quadrotor_interface.h>
#include <limits>
#include <ros/subscriber.h>
#include <hector_quadrotor_interface/limiters.h>
#include <boost/thread/mutex.hpp>
#include "geometry_msgs/PoseStamped.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <cstdlib>
#include <cmath>
#include <pluginlib/class_list_macros.h>
#include <hector_quadrotor_interface/helpers.h>

namespace hector_quadrotor_controllers
{

using namespace hector_quadrotor_interface;

class PositionController : public controller_interface::Controller<QuadrotorInterface>
{
public:
  PositionController()
  {
  }

  virtual ~PositionController()
  {
  }

  virtual bool init(QuadrotorInterface *interface,
            ros::NodeHandle &root_nh,
            ros::NodeHandle &controller_nh)
  {
    // TODO factor out
    pose_ = interface->getPose();
    twist_ = interface->getTwist();
    motor_status_ = interface->getMotorStatus();
    root_nh.param<std::string>("base_link_frame", base_link_frame_, "base_link");
    root_nh.param<std::string>("world_frame", world_frame_, "world");
    root_nh.param<std::string>("base_stabilized_frame", base_stabilized_frame_, "base_stabilized");

    // Initialize PID controllers
    pid_.x.init(ros::NodeHandle(controller_nh, "x"));
    pid_.y.init(ros::NodeHandle(controller_nh, "y"));
    pid_.z.init(ros::NodeHandle(controller_nh, "z"));
    pid_.yaw.init(ros::NodeHandle(controller_nh, "yaw"));

    // Setup pose visualization marker output
    initMarker(root_nh.getNamespace());
    marker_publisher_ = root_nh.advertise<visualization_msgs::Marker>("command/pose_marker", 1);

    // Initialize inputs/outputs
    pose_input_ = interface->addInput<PoseCommandHandle>("pose");
    twist_output_ = interface->addOutput<TwistCommandHandle>("twist");

    position_limiter_ = boost::make_shared<hector_quadrotor_interface::PointLimiter>(root_nh, "limits/pose/position");

    pose_subscriber_ = root_nh.subscribe<geometry_msgs::PoseStamped>("command/pose", 1, boost::bind(
        &PositionController::poseCommandCb, this, _1));

    return true;
  }

  void reset()
  {
    pid_.x.reset();
    pid_.y.reset();
    pid_.z.reset();
    pid_.yaw.reset();

    twist_control_ = geometry_msgs::Twist();
    // Set commanded pose to robot's current pose
    updatePoseCommand(pose_->pose());
  }

  virtual void starting(const ros::Time &time)
  {
    reset();
    twist_output_->start();
  }

  virtual void stopping(const ros::Time &time)
  {
    twist_output_->stop();
  }

  void poseCommandCb(const geometry_msgs::PoseStampedConstPtr &command)
  {
    updatePoseCommand(*command);
  }

  virtual void update(const ros::Time &time, const ros::Duration &period)
  {
    boost::mutex::scoped_lock lock(command_mutex_);

    // Get pose command command input
    if (pose_input_->connected() && pose_input_->enabled())
    {
      updatePoseCommand(pose_input_->getCommand());
    }

    Pose pose = pose_->pose();
    Twist twist = twist_->twist();

    double yaw_command;
    {
      tf2::Quaternion q;
      double temp;
      tf2::fromMsg(pose_command_.orientation, q);
      tf2::Matrix3x3(q).getRPY(temp, temp, yaw_command);
    }

    double yaw = pose_->getYaw();

    pose_command_.position = position_limiter_->limit(pose_command_.position);

    twist_control_.linear.x = pid_.x.computeCommand(pose_command_.position.x - pose.position.x, period);
    twist_control_.linear.y = pid_.y.computeCommand(pose_command_.position.y - pose.position.y, period);
    twist_control_.linear.z = pid_.z.computeCommand(pose_command_.position.z - pose.position.z, period);

    double yaw_error = yaw_command - yaw;
    // detect wrap around pi and compensate
    if (yaw_error > M_PI)
    {
      yaw_error -= 2 * M_PI;
    }
    else if (yaw_error < -M_PI)
    {
      yaw_error += 2 * M_PI;
    }
    twist_control_.angular.z = pid_.yaw.computeCommand(yaw_error, period);

    // rotate by yaw
    {
      Twist temp = twist_control_;
      twist_control_.linear.x = sin(yaw) * temp.linear.y + cos(yaw) * temp.linear.x;
      twist_control_.linear.y = cos(yaw) * temp.linear.y - sin(yaw) * temp.linear.x;
      twist_control_.angular.x = sin(yaw) * temp.angular.y + cos(yaw) * temp.angular.x;
      twist_control_.angular.y = cos(yaw) * temp.angular.y - sin(yaw) * temp.angular.x;
    }

    // TODO pass down stamp from pose command
    twist_output_->setCommand(twist_control_);
  }

private:

  void updatePoseCommand(const geometry_msgs::PoseStamped &new_pose)
  {
    // TODO TF to world frame
    if(new_pose.header.frame_id != world_frame_){
      ROS_WARN_STREAM_THROTTLE(1.0, "Pose commands must be given in the " << world_frame_ << " frame, ignoring command");
    }else
    {
      updatePoseCommand(new_pose.pose);
    }
  }

  void updatePoseCommand(const geometry_msgs::Pose &new_pose)
  {
    {
      boost::mutex::scoped_lock lock(command_mutex_);
      pose_command_.position = new_pose.position;
      // Strip non-yaw components from orientation
      tf2::Quaternion q;
      double roll, pitch, yaw;
      tf2::fromMsg(new_pose.orientation, q);
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      q.setRPY(0, 0, yaw);
      pose_command_.orientation = tf2::toMsg(q);
    }
    pose_marker_.pose = pose_command_;
    marker_publisher_.publish(pose_marker_);
  }

  void initMarker(std::string name)
  {
    pose_marker_.header.frame_id = world_frame_;
    pose_marker_.ns = name;
    pose_marker_.id = 0;
    pose_marker_.type = visualization_msgs::Marker::ARROW;
    pose_marker_.scale.x = 0.15;
    pose_marker_.scale.y = pose_marker_.scale.z = 0.03;
    pose_marker_.color.r = 0.5;
    pose_marker_.color.g = 0.5;
    pose_marker_.color.r = 0.5;
    pose_marker_.color.a = 1.0;
  }

  PoseHandlePtr pose_;
  TwistHandlePtr twist_;
  MotorStatusHandlePtr motor_status_;

  TwistCommandHandlePtr twist_output_;
  PoseCommandHandlePtr pose_input_;

  boost::shared_ptr<hector_quadrotor_interface::PointLimiter> position_limiter_;

  ros::Subscriber pose_subscriber_;
  ros::Publisher marker_publisher_;

  visualization_msgs::Marker pose_marker_;

  geometry_msgs::Pose pose_command_;
  geometry_msgs::Twist twist_control_;

  std::string base_link_frame_, base_stabilized_frame_, world_frame_;

  struct
  {
    control_toolbox::Pid x, y, z, yaw;
  } pid_;

  boost::mutex command_mutex_;

};

} // namespace hector_quadrotor_controllers

PLUGINLIB_EXPORT_CLASS(hector_quadrotor_controllers::PositionController, controller_interface::ControllerBase)
