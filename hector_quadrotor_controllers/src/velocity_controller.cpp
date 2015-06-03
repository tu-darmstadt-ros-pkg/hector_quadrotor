#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

#include <controller_interface/controller.h>
#include <control_toolbox/pid.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>

#include <hector_quadrotor_interface/quadrotor_interface.h>
#include <hector_quadrotor_interface/limiters.h>
#include <hector_quadrotor_interface/helpers.h>

#include <boost/thread/mutex.hpp>
#include <limits>

namespace hector_quadrotor_controllers
{

  using namespace hector_quadrotor_interface;

  class VelocityController : public controller_interface::Controller<hector_quadrotor_interface::QuadrotorInterface>
  {
  public:
    VelocityController()
    {
    }

    virtual ~VelocityController()
    {
    }

    virtual bool init(hector_quadrotor_interface::QuadrotorInterface *interface, ros::NodeHandle &root_nh,
              ros::NodeHandle &controller_nh)
    {
      // TODO factor out
      pose_ = interface->getPose();
      twist_ = interface->getTwist();
      root_nh.param<std::string>("base_link_frame", base_link_frame_, "base_link");
      root_nh.param<std::string>("world_frame", world_frame_, "world");
      root_nh.param<std::string>("base_stabilized_frame", base_stabilized_frame_, "base_stabilized");
      getMassAndInertia(root_nh, mass_, inertia_);

      // Initialize PID controllers
      pid_.x.init(ros::NodeHandle(controller_nh, "x"));
      pid_.y.init(ros::NodeHandle(controller_nh, "y"));
      pid_.z.init(ros::NodeHandle(controller_nh, "z"));

      // Initialize inputs/outputs
      twist_input_ = interface->addInput<TwistCommandHandle>("twist");
      attitude_output_ = interface->addOutput<AttitudeCommandHandle>("attitude");
      yawrate_output_ = interface->addOutput<YawrateCommandHandle>("yawrate");
      thrust_output_ = interface->addOutput<ThrustCommandHandle>("thrust");

      twist_limiter_ = boost::make_shared<hector_quadrotor_interface::TwistLimiter>(root_nh, "limits/twist");

      twist_subscriber_ = root_nh.subscribe<geometry_msgs::TwistStamped>("command/twist", 1, boost::bind(
          &VelocityController::twistCommandCallback, this, _1));

      return true;
    }

    void reset()
    {
      pid_.x.reset();
      pid_.y.reset();
      pid_.z.reset();

      attitude_control_ = hector_uav_msgs::AttitudeCommand();
      yawrate_control_ = hector_uav_msgs::YawrateCommand();
      thrust_control_ = hector_uav_msgs::ThrustCommand();
    }

    virtual void starting(const ros::Time &time)
    {
      reset();
      attitude_output_->start();
      yawrate_output_->start();
      thrust_output_->start();
    }

    virtual void stopping(const ros::Time &time)
    {
      attitude_output_->stop();
      yawrate_output_->stop();
      thrust_output_->stop();
    }


    void twistCommandCallback(const geometry_msgs::TwistStampedConstPtr &command)
    {
      boost::mutex::scoped_lock lock(command_mutex_);
      // TODO tf to base_stabilized frame
      if(command->header.frame_id != base_stabilized_frame_){
        ROS_WARN_STREAM_THROTTLE(1.0, "Velocity commands must be given in the " << base_stabilized_frame_ << " frame, ignoring command");
      }else
      {
        twist_command_ = command->twist;
      }
    }

    virtual void update(const ros::Time &time, const ros::Duration &period)
    {
      boost::mutex::scoped_lock lock(command_mutex_);

      // Get twist command input
      if (twist_input_->connected() && twist_input_->enabled())
      {
        twist_command_ = twist_input_->getCommand();
      }

      // Limit twist input
      twist_command_ = twist_limiter_->limit(twist_command_);

      // Get current twist in body frame
      Twist twist = twist_->twist(), twist_body;
      twist_body.linear = pose_->toBody(twist.linear);
      twist_body.angular = pose_->toBody(twist.angular);

      static const double gravity = 9.80665;

      // Run PID loops
      attitude_control_.pitch = pid_.x.computeCommand(twist_command_.linear.x - twist_body.linear.x, period);
      attitude_control_.roll = -pid_.y.computeCommand(twist_command_.linear.y - twist_body.linear.y, period);
      yawrate_control_.turnrate = twist_command_.angular.z;
      thrust_control_.thrust =
          mass_ * (pid_.z.computeCommand(twist_command_.linear.z - twist.linear.z, period) + gravity);

      // TODO pass down stamp from twist command
      attitude_control_.header.stamp = time;
      yawrate_control_.header.stamp = time;
      thrust_control_.header.stamp = time;

      // Update output from controller
      attitude_output_->setCommand(attitude_control_);
      yawrate_output_->setCommand(yawrate_control_);
      thrust_output_->setCommand(thrust_control_);

    }

  private:

    PoseHandlePtr pose_;
    TwistHandlePtr twist_;

    TwistCommandHandlePtr twist_input_;
    AttitudeCommandHandlePtr attitude_output_;
    YawrateCommandHandlePtr yawrate_output_;
    ThrustCommandHandlePtr thrust_output_;

    ros::Subscriber twist_subscriber_;

    geometry_msgs::Twist twist_command_;
    hector_uav_msgs::AttitudeCommand attitude_control_;
    hector_uav_msgs::YawrateCommand yawrate_control_;
    hector_uav_msgs::ThrustCommand thrust_control_;

    boost::shared_ptr<hector_quadrotor_interface::TwistLimiter> twist_limiter_;
    std::string base_link_frame_, base_stabilized_frame_, world_frame_;

    struct
    {
      control_toolbox::Pid x, y, z;
    } pid_;

    double mass_;
    double inertia_[3];

    boost::mutex command_mutex_;

  };

} // namespace hector_quadrotor_controllers

PLUGINLIB_EXPORT_CLASS(hector_quadrotor_controllers::VelocityController, controller_interface::ControllerBase)
