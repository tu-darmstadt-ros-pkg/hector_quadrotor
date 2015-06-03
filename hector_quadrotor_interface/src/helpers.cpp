#include <hector_quadrotor_interface/helpers.h>
#include <urdf_parser/urdf_parser.h>

namespace hector_quadrotor_interface
{

  bool getMassAndInertia(const ros::NodeHandle &nh, double &mass, double inertia[3])
  {

    std::string robot_description;
    if (!nh.getParam("robot_description", robot_description))
    {
      ROS_ERROR_STREAM("getMassAndIntertia couldn't find URDF at " << nh.getNamespace() << "/robot_description");
      return false;
    }

    boost::shared_ptr<urdf::ModelInterface> model;
    try
    {
      model = urdf::parseURDF(robot_description);
    }
    catch (std::exception ex)
    {
      ROS_ERROR_STREAM(
          "getMassAndIntertia couldn't parse URDF at " << nh.getNamespace() << "/robot_description: " << ex.what());
      return false;
    }

    boost::shared_ptr<urdf::Inertial> inertial = model->getRoot()->inertial;
    if (!inertial || !inertial->mass || !inertial->ixx || !inertial->iyy || !inertial->izz)
    {
      ROS_ERROR_STREAM(
          "getMassAndIntertia requires intertial information stored on the root link " << nh.getNamespace() <<
          "/robot_description");
      return false;
    }

    mass = inertial->mass;
    inertia[0] = inertial->ixx;
    inertia[1] = inertial->iyy;
    inertia[2] = inertial->izz;
    return true;
  }

  bool poseWithinTolerance(const geometry_msgs::Pose &pose_current, const geometry_msgs::Pose &pose_target,
                           const double dist_tolerance, const double yaw_tolerance)
  {

    if(yaw_tolerance > 0.0)
    {
      double yaw_current, yaw_target;
      tf2::Quaternion q;
      double temp;
      tf2::fromMsg(pose_current.orientation, q);
      tf2::Matrix3x3(q).getRPY(temp, temp, yaw_current);
      tf2::fromMsg(pose_target.orientation, q);
      tf2::Matrix3x3(q).getRPY(temp, temp, yaw_target);

      double yaw_error = yaw_current - yaw_target;
      // detect wrap around pi and compensate
      if (yaw_error > M_PI)
      {
        yaw_error -= 2 * M_PI;
      }
      else if (yaw_error < -M_PI)
      {
        yaw_error += 2 * M_PI;
      }
      if (std::abs(yaw_error) > yaw_tolerance)
      {
        ROS_DEBUG_STREAM("Waiting for yaw " << std::abs(yaw_current - yaw_target));
        return false;
      }

    }

    if(dist_tolerance > 0.0)
    {
      tf2::Vector3 v_current(pose_current.position.x, pose_current.position.y, pose_current.position.z);
      tf2::Vector3 v_target(pose_target.position.x, pose_target.position.y, pose_target.position.z);
      if ((v_current - v_target).length() > dist_tolerance)
      {
        ROS_DEBUG_STREAM("Waiting for yaw " << (v_current - v_target).length());
        return false;
      }
    }

    return true;

  }

}