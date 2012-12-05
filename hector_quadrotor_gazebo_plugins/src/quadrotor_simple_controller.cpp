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

#include <hector_quadrotor_gazebo_plugins/quadrotor_simple_controller.h>
#include "common/Events.hh"
#include "physics/physics.h"

#include <cmath>

namespace gazebo {

GazeboQuadrotorSimpleController::GazeboQuadrotorSimpleController()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboQuadrotorSimpleController::~GazeboQuadrotorSimpleController()
{
  event::Events::DisconnectWorldUpdateStart(updateConnection);

  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboQuadrotorSimpleController::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  world = _model->GetWorld();

  // load parameters
  if (!_sdf->HasElement("robotNamespace"))
    namespace_.clear();
  else
    namespace_ = _sdf->GetElement("robotNamespace")->GetValueString() + "/";

  if (!_sdf->HasElement("topicName"))
    velocity_topic_ = "cmd_vel";
  else
    velocity_topic_ = _sdf->GetElement("topicName")->GetValueString();

  if (!_sdf->HasElement("imuTopic"))
    imu_topic_.clear();
  else
    imu_topic_ = _sdf->GetElement("imuTopic")->GetValueString();

  if (!_sdf->HasElement("stateTopic"))
    state_topic_.clear();
  else
    state_topic_ = _sdf->GetElement("stateTopic")->GetValueString();

  if (!_sdf->HasElement("bodyName"))
  {
    link = _model->GetLink();
    link_name_ = link->GetName();
  }
  else {
    link_name_ = _sdf->GetElement("bodyName")->GetValueString();
    link = _model->GetLink(link_name_);
  }

  if (!link)
  {
    ROS_FATAL("gazebo_ros_baro plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }

  if (!_sdf->HasElement("maxForce"))
    max_force_ = -1;
  else
    max_force_ = _sdf->GetElement("maxForce")->GetValueDouble();

  controllers_.roll.Load(_sdf, "rollpitch");
  controllers_.pitch.Load(_sdf, "rollpitch");
  controllers_.yaw.Load(_sdf, "yaw");
  controllers_.velocity_x.Load(_sdf, "velocityXY");
  controllers_.velocity_y.Load(_sdf, "velocityXY");
  controllers_.velocity_z.Load(_sdf, "velocityZ");

  // Get inertia and mass of quadrotor body
  inertia = link->GetInertial()->GetPrincipalMoments();
  mass = link->GetInertial()->GetMass();

  node_handle_ = new ros::NodeHandle(namespace_);

  // subscribe command
  if (!velocity_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<geometry_msgs::Twist>(
      velocity_topic_, 1,
      boost::bind(&GazeboQuadrotorSimpleController::VelocityCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    velocity_subscriber_ = node_handle_->subscribe(ops);
  }

  // subscribe imu
  if (!imu_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<sensor_msgs::Imu>(
      imu_topic_, 1,
      boost::bind(&GazeboQuadrotorSimpleController::ImuCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    imu_subscriber_ = node_handle_->subscribe(ops);

    ROS_INFO_NAMED("quadrotor_simple_controller", "Using imu information on topic %s as source of orientation and angular velocity.", imu_topic_.c_str());
  }

  // subscribe state
  if (!state_topic_.empty())
  {
    ros::SubscribeOptions ops = ros::SubscribeOptions::create<nav_msgs::Odometry>(
      state_topic_, 1,
      boost::bind(&GazeboQuadrotorSimpleController::StateCallback, this, _1),
      ros::VoidPtr(), &callback_queue_);
    state_subscriber_ = node_handle_->subscribe(ops);

    ROS_INFO_NAMED("quadrotor_simple_controller", "Using state information on topic %s as source of state information.", state_topic_.c_str());
  }

  // callback_queue_thread_ = boost::thread( boost::bind( &GazeboQuadrotorSimpleController::CallbackQueueThread,this ) );


  Reset();

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection = event::Events::ConnectWorldUpdateStart(
      boost::bind(&GazeboQuadrotorSimpleController::Update, this));
}

////////////////////////////////////////////////////////////////////////////////
// Callbacks
void GazeboQuadrotorSimpleController::VelocityCallback(const geometry_msgs::TwistConstPtr& velocity)
{
  velocity_command_ = *velocity;
}

void GazeboQuadrotorSimpleController::ImuCallback(const sensor_msgs::ImuConstPtr& imu)
{
  pose.rot.Set(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
  euler = pose.rot.GetAsEuler();
  angular_velocity = pose.rot.RotateVector(math::Vector3(imu->angular_velocity.x, imu->angular_velocity.y, imu->angular_velocity.z));
}

void GazeboQuadrotorSimpleController::StateCallback(const nav_msgs::OdometryConstPtr& state)
{
  math::Vector3 velocity1(velocity);

  if (imu_topic_.empty()) {
    pose.pos.Set(state->pose.pose.position.x, state->pose.pose.position.y, state->pose.pose.position.z);
    pose.rot.Set(state->pose.pose.orientation.w, state->pose.pose.orientation.x, state->pose.pose.orientation.y, state->pose.pose.orientation.z);
    euler = pose.rot.GetAsEuler();
    angular_velocity.Set(state->twist.twist.angular.x, state->twist.twist.angular.y, state->twist.twist.angular.z);
  }

  velocity.Set(state->twist.twist.linear.x, state->twist.twist.linear.y, state->twist.twist.linear.z);

  // calculate acceleration
  double dt = !state_stamp.isZero() ? (state->header.stamp - state_stamp).toSec() : 0.0;
  state_stamp = state->header.stamp;
  if (dt > 0.0) {
    acceleration = (velocity - velocity1) / dt;
  } else {
    acceleration.Set();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboQuadrotorSimpleController::Update()
{
  math::Vector3 force, torque;

  // Get new commands/state
  callback_queue_.callAvailable();

  // Get simulator time
  common::Time sim_time = world->GetSimTime();
  double dt = (sim_time - last_time).Double();
  if (dt == 0.0) return;

  // Get Pose/Orientation from Gazebo (if no state subscriber is active)
  if (imu_topic_.empty()) {
    pose = link->GetWorldPose();
    angular_velocity = link->GetWorldAngularVel();
    euler = pose.rot.GetAsEuler();
  }
  if (state_topic_.empty()) {
    acceleration = (link->GetWorldLinearVel() - velocity) / dt;
    velocity = link->GetWorldLinearVel();
  }

//  static Time lastDebug;
//  if ((world->GetSimTime() - lastDebug).Double() > 0.5) {
//    ROS_DEBUG_STREAM_NAMED("quadrotor_simple_controller", "Velocity:         gazebo = [" << link->GetWorldLinearVel()   << "], state = [" << velocity << "]");
//    ROS_DEBUG_STREAM_NAMED("quadrotor_simple_controller", "Acceleration:     gazebo = [" << link->GetWorldLinearAccel() << "], state = [" << acceleration << "]");
//    ROS_DEBUG_STREAM_NAMED("quadrotor_simple_controller", "Angular Velocity: gazebo = [" << link->GetWorldAngularVel() << "], state = [" << angular_velocity << "]");
//    lastDebug = world->GetSimTime();
//  }

  // Get gravity
  math::Vector3 gravity_body = pose.rot.RotateVector(world->GetPhysicsEngine()->GetGravity());
  double gravity = gravity_body.GetLength();
  double load_factor = gravity * gravity / world->GetPhysicsEngine()->GetGravity().GetDotProd(gravity_body);  // Get gravity

  // Rotate vectors to coordinate frames relevant for control
  math::Quaternion heading_quaternion(cos(euler.z/2),0,0,sin(euler.z/2));
  math::Vector3 velocity_xy = heading_quaternion.RotateVectorReverse(velocity);
  math::Vector3 acceleration_xy = heading_quaternion.RotateVectorReverse(acceleration);
  math::Vector3 angular_velocity_body = pose.rot.RotateVectorReverse(angular_velocity);

  // update controllers
  force.Set(0.0, 0.0, 0.0);
  torque.Set(0.0, 0.0, 0.0);
  double pitch_command =  controllers_.velocity_x.update(velocity_command_.linear.x, velocity_xy.x, acceleration_xy.x, dt) / gravity;
  double roll_command  = -controllers_.velocity_y.update(velocity_command_.linear.y, velocity_xy.y, acceleration_xy.y, dt) / gravity;
  torque.x = inertia.x *  controllers_.roll.update(roll_command, euler.x, angular_velocity_body.x, dt);
  torque.y = inertia.y *  controllers_.pitch.update(pitch_command, euler.y, angular_velocity_body.y, dt);
  // torque.x = inertia.x *  controllers_.roll.update(-velocity_command_.linear.y/gravity, euler.x, angular_velocity_body.x, dt);
  // torque.y = inertia.y *  controllers_.pitch.update(velocity_command_.linear.x/gravity, euler.y, angular_velocity_body.y, dt);
  torque.z = inertia.z *  controllers_.yaw.update(velocity_command_.angular.z, angular_velocity.z, 0, dt);
  force.z  = mass      * (controllers_.velocity_z.update(velocity_command_.linear.z,  velocity.z, acceleration.z, dt) + load_factor * gravity);
  if (max_force_ > 0.0 && force.z > max_force_) force.z = max_force_;
  if (force.z < 0.0) force.z = 0.0;

//  static double lastDebugOutput = 0.0;
//  if (last_time.Double() - lastDebugOutput > 0.1) {
//    ROS_DEBUG_NAMED("quadrotor_simple_controller", "Velocity = [%g %g %g], Acceleration = [%g %g %g]", velocity.x, velocity.y, velocity.z, acceleration.x, acceleration.y, acceleration.z);
//    ROS_DEBUG_NAMED("quadrotor_simple_controller", "Command: linear = [%g %g %g], angular = [%g %g %g], roll/pitch = [%g %g]", velocity_command_.linear.x, velocity_command_.linear.y, velocity_command_.linear.z, velocity_command_.angular.x*180/M_PI, velocity_command_.angular.y*180/M_PI, velocity_command_.angular.z*180/M_PI, roll_command*180/M_PI, pitch_command*180/M_PI);
//    ROS_DEBUG_NAMED("quadrotor_simple_controller", "Mass: %g kg, Inertia: [%g %g %g], Load: %g g", mass, inertia.x, inertia.y, inertia.z, load_factor);
//    ROS_DEBUG_NAMED("quadrotor_simple_controller", "Force: [%g %g %g], Torque: [%g %g %g]", force.x, force.y, force.z, torque.x, torque.y, torque.z);
//    lastDebugOutput = last_time.Double();
//  }

  // set force and torque in gazebo
  link->AddRelativeForce(force);
  link->AddRelativeTorque(torque - link->GetInertial()->GetCoG().GetCrossProd(force));

  // save last time stamp
  last_time = sim_time;
}

////////////////////////////////////////////////////////////////////////////////
// Reset the controller
void GazeboQuadrotorSimpleController::Reset()
{
  controllers_.roll.reset();
  controllers_.pitch.reset();
  controllers_.yaw.reset();
  controllers_.velocity_x.reset();
  controllers_.velocity_y.reset();
  controllers_.velocity_z.reset();

  link->SetForce(math::Vector3(0,0,0));
  link->SetTorque(math::Vector3(0,0,0));

  // reset state
  pose.Reset();
  velocity.Set();
  angular_velocity.Set();
  acceleration.Set();
  euler.Set();
  state_stamp = ros::Time();
}

////////////////////////////////////////////////////////////////////////////////
// PID controller implementation
GazeboQuadrotorSimpleController::PIDController::PIDController()
{
}

GazeboQuadrotorSimpleController::PIDController::~PIDController()
{
}

void GazeboQuadrotorSimpleController::PIDController::Load(sdf::ElementPtr _sdf, const std::string& prefix)
{
  gain_p = 0.0;
  gain_d = 0.0;
  gain_i = 0.0;
  time_constant = 0.0;
  limit = -1.0;

  if (!_sdf) return;
  // _sdf->PrintDescription(_sdf->GetName());
  if (_sdf->HasElement(prefix + "ProportionalGain")) gain_p = _sdf->GetElement(prefix + "ProportionalGain")->GetValueDouble();
  if (_sdf->HasElement(prefix + "DifferentialGain")) gain_d = _sdf->GetElement(prefix + "DifferentialGain")->GetValueDouble();
  if (_sdf->HasElement(prefix + "IntegralGain"))     gain_i = _sdf->GetElement(prefix + "IntegralGain")->GetValueDouble();
  if (_sdf->HasElement(prefix + "TimeConstant"))     time_constant = _sdf->GetElement(prefix + "TimeConstant")->GetValueDouble();
  if (_sdf->HasElement(prefix + "Limit"))            limit = _sdf->GetElement(prefix + "Limit")->GetValueDouble();
}

double GazeboQuadrotorSimpleController::PIDController::update(double new_input, double x, double dx, double dt)
{
  // limit command
  if (limit > 0.0 && fabs(new_input) > limit) new_input = (new_input < 0 ? -1.0 : 1.0) * limit;

  // filter command
  if (dt + time_constant > 0.0) {
    dinput = (new_input - input) / (dt + time_constant);
    input  = (dt * new_input + time_constant * input) / (dt + time_constant);
  }

  // update proportional, differential and integral errors
  p = input - x;
  d = dinput - dx;
  i = i + dt * p;

  // update control output
  output = gain_p * p + gain_d * d + gain_i * i;
  return output;
}

void GazeboQuadrotorSimpleController::PIDController::reset()
{
  input = dinput = 0;
  p = i = d = output = 0;
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboQuadrotorSimpleController)

} // namespace gazebo
