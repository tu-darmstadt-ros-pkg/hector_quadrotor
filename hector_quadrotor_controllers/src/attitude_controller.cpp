#include <ros/ros.h>

#include <controller_interface/controller.h>
#include <control_toolbox/pid.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/AccelStamped.h>

#include <hector_quadrotor_interface/limiters.h>
#include <hector_quadrotor_interface/quadrotor_interface.h>
#include <hector_quadrotor_interface/helpers.h>

#include <boost/thread/mutex.hpp>
#include <std_msgs/Bool.h>
#include <limits>

namespace hector_quadrotor_controllers
{

using namespace hector_quadrotor_interface;

class AttitudeController : public controller_interface::Controller<hector_quadrotor_interface::QuadrotorInterface>
{
public:
  AttitudeController()
  {
  }

  virtual ~AttitudeController()
  {
  }

  virtual bool init(hector_quadrotor_interface::QuadrotorInterface *interface, ros::NodeHandle &root_nh,
            ros::NodeHandle &controller_nh)
  {

    pose_ = interface->getPose();
    twist_ = interface->getTwist();
    accel_ = interface->getAccel();

    root_nh.param<std::string>("base_link_frame", base_link_frame_, "base_link");
    root_nh.param<std::string>("base_stabilized_frame", base_stabilized_frame_, "base_stabilized");
    root_nh.param<double>("estop_deceleration", estop_deceleration_, 1.0);
    root_nh.param<double>("command_timeout", command_timeout_, 0.05);

    getMassAndInertia(root_nh, mass_, inertia_);

    attitude_input_ = interface->addInput<AttitudeCommandHandle>("attitude");
    yawrate_input_ = interface->addInput<YawrateCommandHandle>("yawrate");
    thrust_input_ = interface->addInput<ThrustCommandHandle>("thrust");
    accel_output_ = interface->addOutput<AccelCommandHandle>("accel");

    // subscribe to attitude, yawrate, and thrust
    attitude_subscriber_helper_ = boost::make_shared<AttitudeSubscriberHelper>(ros::NodeHandle(root_nh, "command"),
                                                                               boost::ref(command_mutex_),
                                                                               boost::ref(attitude_command_),
                                                                               boost::ref(yawrate_command_),
                                                                               boost::ref(thrust_command_));

    // initialize PID controllers
    pid_.roll.init(ros::NodeHandle(controller_nh, "roll"));
    pid_.pitch.init(ros::NodeHandle(controller_nh, "pitch"));
    pid_.yawrate.init(ros::NodeHandle(controller_nh, "yawrate"));

    ros::NodeHandle limit_nh(root_nh, "limits");
    attitude_limiter_ = boost::make_shared<hector_quadrotor_interface::AttitudeCommandLimiter>(limit_nh,
                                                                                                "pose/orientation");
    yawrate_limiter_ = boost::make_shared<hector_quadrotor_interface::YawrateCommandLimiter>(limit_nh,
                                                                                              "twist/angular");
    thrust_limiter_ = boost::make_shared<hector_quadrotor_interface::ThrustCommandLimiter>(limit_nh, "wrench/force");

    estop_ = false;
    estop_sub_ = root_nh.subscribe("estop", 1, &AttitudeController::estopCb, this);

    return true;
  }

  void reset()
  {
    pid_.roll.reset();
    pid_.pitch.reset();
    pid_.yawrate.reset();
    accel_control_ = geometry_msgs::AccelStamped();
  }

  virtual void starting(const ros::Time &time)
  {
    reset();
    accel_output_->start();
  }

  virtual void stopping(const ros::Time &time)
  {
    accel_output_->stop();
  }

  virtual void update(const ros::Time &time, const ros::Duration &period)
  {
    boost::mutex::scoped_lock lock(command_mutex_);

    if (attitude_input_->connected() && attitude_input_->enabled())
    {
      attitude_command_ = attitude_input_->getCommand();
    }
    if (yawrate_input_->connected() && yawrate_input_->enabled())
    {
      yawrate_command_ = yawrate_input_->getCommand();
    }
    if (thrust_input_->connected() && thrust_input_->enabled())
    {
      thrust_command_ = thrust_input_->getCommand();
    }

    attitude_command_ = attitude_limiter_->limit(attitude_command_);
    yawrate_command_ = yawrate_limiter_->limit(yawrate_command_);
    thrust_command_ = thrust_limiter_->limit(thrust_command_);


    // TODO move estop to gazebo plugin
    if(time > attitude_command_.header.stamp + ros::Duration(command_timeout_) ||
       time > yawrate_command_.header.stamp + ros::Duration(command_timeout_) ||
       time > thrust_command_.header.stamp + ros::Duration(command_timeout_) )
    {
      if(!command_estop_){
        estop_thrust_command_ = thrust_command_;
      }
      ROS_WARN_STREAM_THROTTLE(1.0, "No command received for "
                                    << (time - std::min(std::min(attitude_command_.header.stamp, yawrate_command_.header.stamp), thrust_command_.header.stamp)).toSec() <<
          "s, triggering estop");
      command_estop_ = true;
    }else if(command_estop_){
      command_estop_ = false;
    }

    double roll, pitch, yaw;
    pose_->getEulerRPY(roll, pitch, yaw);

    Twist twist = twist_->twist(), twist_body;
    twist_body.linear = pose_->toBody(twist.linear);
    twist_body.angular = pose_->toBody(twist.angular);

    Accel accel = accel_->acceleration(), accel_body;
    accel_body.linear = pose_->toBody(accel.linear);
    accel_body.angular = pose_->toBody(accel.angular);

    if (estop_ || command_estop_)
    {
      attitude_command_.roll = attitude_command_.pitch = yawrate_command_.turnrate = 0;
      estop_thrust_command_.thrust -= estop_deceleration_ * mass_ * period.toSec();
      if(estop_thrust_command_.thrust < 0) estop_thrust_command_.thrust = 0;
      thrust_command_ = estop_thrust_command_;
    }
    accel_control_.accel.angular.x = pid_.roll.computeCommand(attitude_command_.roll - roll, period);
    accel_control_.accel.angular.y = pid_.pitch.computeCommand(attitude_command_.pitch - pitch, period);
    accel_control_.accel.angular.z = pid_.yawrate.computeCommand(yawrate_command_.turnrate - twist_body.angular.z,
                                                                 period);
    accel_control_.accel.linear.z = thrust_command_.thrust / mass_;

    // set wrench output
    accel_control_.header.stamp = time;
    accel_control_.header.frame_id = base_link_frame_;
    accel_output_->setCommand(accel_control_.accel);

  }

  void estopCb(const std_msgs::BoolConstPtr &estop_msg)
  {
    bool estop = static_cast<bool>(estop_msg->data);
    if (estop_ == false && estop == true)
    {
      estop_thrust_command_ = thrust_command_;
    }
    estop_ = estop;
  }

private:

  PoseHandlePtr pose_;
  TwistHandlePtr twist_;
  AccelerationHandlePtr accel_;

  AttitudeCommandHandlePtr attitude_input_;
  YawrateCommandHandlePtr yawrate_input_;
  ThrustCommandHandlePtr thrust_input_;
  AccelCommandHandlePtr accel_output_;

  boost::shared_ptr<hector_quadrotor_interface::AttitudeSubscriberHelper> attitude_subscriber_helper_;

  hector_uav_msgs::AttitudeCommand attitude_command_;
  hector_uav_msgs::YawrateCommand yawrate_command_;
  hector_uav_msgs::ThrustCommand thrust_command_;
  geometry_msgs::AccelStamped accel_control_;

  boost::shared_ptr<hector_quadrotor_interface::AttitudeCommandLimiter> attitude_limiter_;
  boost::shared_ptr<hector_quadrotor_interface::YawrateCommandLimiter> yawrate_limiter_;
  boost::shared_ptr<hector_quadrotor_interface::ThrustCommandLimiter> thrust_limiter_;
  std::string base_link_frame_, base_stabilized_frame_;

  ros::Subscriber estop_sub_;
  bool estop_, command_estop_;
  hector_uav_msgs::ThrustCommand estop_thrust_command_;
  double estop_deceleration_;
  double command_timeout_;

  struct
  {
    control_toolbox::Pid roll, pitch, yawrate;
  } pid_;

  double mass_;
  double inertia_[3];

  boost::mutex command_mutex_;

};

} // namespace hector_quadrotor_controllers

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(hector_quadrotor_controllers::AttitudeController, controller_interface::ControllerBase
)
