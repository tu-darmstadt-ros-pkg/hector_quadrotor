//=================================================================================================
// Copyright (c) 2013, Johannes Meyer, TU Darmstadt
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

#include <hector_quadrotor_controller/quadrotor_interface.h>
#include <controller_interface/controller.h>

#include <geometry_msgs/WrenchStamped.h>
#include <std_srvs/Empty.h>

#include <ros/subscriber.h>
#include <ros/callback_queue.h>

namespace hector_quadrotor_controller {

using namespace controller_interface;

class MotorController : public controller_interface::Controller<QuadrotorInterface>
{
public:
  MotorController()
    : node_handle_(0)
  {}

  ~MotorController()
  {
    if (node_handle_) {
      node_handle_->shutdown();
      delete node_handle_;
      node_handle_ = 0;
    }
  }

  bool init(QuadrotorInterface *interface, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
  {
    // get interface handles
    wrench_input_  = interface->addInput<WrenchCommandHandle>("wrench");
    motor_output_  = interface->addOutput<MotorCommandHandle>("motor");

    // initialize NodeHandle
    delete node_handle_;
    node_handle_ = new ros::NodeHandle(root_nh);

    // load parameters
    controller_nh.getParam("force_per_voltage", parameters_.force_per_voltage = 0.559966216);
    controller_nh.getParam("torque_per_voltage", parameters_.torque_per_voltage = 7.98598e-3);
    controller_nh.getParam("lever", parameters_.lever = 0.275);
    root_nh.param<std::string>("base_link_frame", base_link_frame_, "base_link");

    // TODO: calculate these parameters from the quadrotor_propulsion parameters
//    quadrotor_propulsion:
//            k_t: 0.015336864714397
//            k_m: -7.011631909766668e-5

//            CT2s: -1.3077e-2
//            CT1s: -2.5224e-4
//            CT0s:  1.538190483976698e-5

    return true;
  }

  void reset()
  {
    wrench_.wrench.force.x  = 0.0;
    wrench_.wrench.force.y  = 0.0;
    wrench_.wrench.force.z  = 0.0;
    wrench_.wrench.torque.x = 0.0;
    wrench_.wrench.torque.y = 0.0;
    wrench_.wrench.torque.z = 0.0;

    motor_.force.assign(4, 0.0);
    motor_.torque.assign(4, 0.0);
    motor_.frequency.clear();
    motor_.voltage.assign(4, 0.0);
  }

  void wrenchCommandCallback(const geometry_msgs::WrenchStampedConstPtr& command)
  {
    wrench_ = *command;
    if (wrench_.header.stamp.isZero()) wrench_.header.stamp = ros::Time::now();

    // start controller if it not running
    if (!isRunning()) this->startRequest(wrench_.header.stamp);
  }

  void starting(const ros::Time &time)
  {
    reset();
    motor_output_->start();
  }

  void stopping(const ros::Time &time)
  {
    motor_output_->stop();
  }

  void update(const ros::Time& time, const ros::Duration& period)
  {
    // Get wrench command input
    if (wrench_input_->connected() && wrench_input_->enabled()) {
      wrench_.wrench = wrench_input_->getCommand();
    }

    // Update output
    if (wrench_.wrench.force.z > 0.0) {

      double nominal_thrust_per_motor = wrench_.wrench.force.z / 4.0;
      motor_.force[0] =  nominal_thrust_per_motor - wrench_.wrench.torque.y / 2.0 / parameters_.lever;
      motor_.force[1] =  nominal_thrust_per_motor - wrench_.wrench.torque.x / 2.0 / parameters_.lever;
      motor_.force[2] =  nominal_thrust_per_motor + wrench_.wrench.torque.y / 2.0 / parameters_.lever;
      motor_.force[3] =  nominal_thrust_per_motor + wrench_.wrench.torque.x / 2.0 / parameters_.lever;

      double nominal_torque_per_motor = wrench_.wrench.torque.z / 4.0;
      motor_.voltage[0] = motor_.force[0] / parameters_.force_per_voltage + nominal_torque_per_motor / parameters_.torque_per_voltage;
      motor_.voltage[1] = motor_.force[1] / parameters_.force_per_voltage - nominal_torque_per_motor / parameters_.torque_per_voltage;
      motor_.voltage[2] = motor_.force[2] / parameters_.force_per_voltage + nominal_torque_per_motor / parameters_.torque_per_voltage;
      motor_.voltage[3] = motor_.force[3] / parameters_.force_per_voltage - nominal_torque_per_motor / parameters_.torque_per_voltage;

      motor_.torque[0] = motor_.voltage[0] * parameters_.torque_per_voltage;
      motor_.torque[1] = motor_.voltage[1] * parameters_.torque_per_voltage;
      motor_.torque[2] = motor_.voltage[2] * parameters_.torque_per_voltage;
      motor_.torque[3] = motor_.voltage[3] * parameters_.torque_per_voltage;

      if (motor_.voltage[0] < 0.0) motor_.voltage[0] = 0.0;
      if (motor_.voltage[1] < 0.0) motor_.voltage[1] = 0.0;
      if (motor_.voltage[2] < 0.0) motor_.voltage[2] = 0.0;
      if (motor_.voltage[3] < 0.0) motor_.voltage[3] = 0.0;

    } else {
      reset();
    }

    // set wrench output
    motor_.header.stamp = time;
    motor_.header.frame_id = "base_link";
    motor_output_->setCommand(motor_);
  }

private:
  WrenchCommandHandlePtr wrench_input_;
  MotorCommandHandlePtr motor_output_;

  ros::NodeHandle *node_handle_;
  ros::Subscriber wrench_subscriber_;
  ros::ServiceServer engage_service_server_;
  ros::ServiceServer shutdown_service_server_;

  geometry_msgs::WrenchStamped wrench_;
  hector_uav_msgs::MotorCommand motor_;
  std::string base_link_frame_;

  struct {
    double force_per_voltage;     // coefficient for linearized volts to force conversion for a single motor [N / V]
    double torque_per_voltage;    // coefficient for linearized volts to force conversion for a single motor [Nm / V]
    double lever;                 // the lever arm from origin to the motor axes (symmetry assumption) [m]
  } parameters_;
};

} // namespace hector_quadrotor_controller

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hector_quadrotor_controller::MotorController, controller_interface::ControllerBase)
