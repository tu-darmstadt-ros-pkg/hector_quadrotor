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

#include <cmath>

namespace hector_quadrotor_controller {

QuadrotorInterface::QuadrotorInterface()
{}

QuadrotorInterface::~QuadrotorInterface()
{}

bool QuadrotorInterface::enabled(const CommandHandle *handle) const
{
  if (!handle || !handle->connected()) return false;
  std::string resource = handle->getName();
  return enabled_.count(resource) > 0;
}

bool QuadrotorInterface::start(const CommandHandle *handle)
{
  if (!handle || !handle->connected()) return false;
  if (enabled(handle)) return true;
  std::string resource = handle->getName();
  enabled_[resource] = handle;
  ROS_DEBUG_NAMED("quadrotor_interface", "Enabled %s control", resource.c_str());
  return true;
}

void QuadrotorInterface::stop(const CommandHandle *handle)
{
  if (!handle) return;
  if (!enabled(handle)) return;
  std::string resource = handle->getName();
  std::map<std::string, const CommandHandle *>::iterator it = enabled_.lower_bound(resource);
  if (it != enabled_.end() && it->second == handle) enabled_.erase(it);
  ROS_DEBUG_NAMED("quadrotor_interface", "Disabled %s control", resource.c_str());
}

void QuadrotorInterface::disconnect(const CommandHandle *handle)
{
  if (!handle) return;
  std::string resource = handle->getName();
  if (inputs_.count(resource)) {
    const CommandHandlePtr& input = inputs_.at(resource);
    if (input.get() != handle) input->reset();
  }
  if (outputs_.count(resource)) {
    const CommandHandlePtr& output = outputs_.at(resource);
    if (output.get() != handle) output->reset();
  }
}

const Pose *QuadrotorInterface::getPoseCommand()          const { return getCommand<PoseCommandHandle>("pose"); }
const Twist *QuadrotorInterface::getTwistCommand()        const { return getCommand<TwistCommandHandle>("twist"); }
const Wrench *QuadrotorInterface::getWrenchCommand()      const { return getCommand<WrenchCommandHandle>("wrench"); }
const MotorCommand *QuadrotorInterface::getMotorCommand() const { return getCommand<MotorCommandHandle>("motor"); }

void PoseHandle::getEulerRPY(double &roll, double &pitch, double &yaw) const
{
  const Quaternion::_w_type& w = pose().orientation.w;
  const Quaternion::_x_type& x = pose().orientation.x;
  const Quaternion::_y_type& y = pose().orientation.y;
  const Quaternion::_z_type& z = pose().orientation.z;
  roll  =  atan2(2.*y*z + 2.*w*x, z*z - y*y - x*x + w*w);
  pitch = -asin(2.*x*z - 2.*w*y);
  yaw   =  atan2(2.*x*y + 2.*w*z, x*x + w*w - z*z - y*y);
}

double PoseHandle::getYaw() const
{
  const Quaternion::_w_type& w = pose().orientation.w;
  const Quaternion::_x_type& x = pose().orientation.x;
  const Quaternion::_y_type& y = pose().orientation.y;
  const Quaternion::_z_type& z = pose().orientation.z;
  return atan2(2.*x*y + 2.*w*z, x*x + w*w - z*z - y*y);
}

Vector3 PoseHandle::toBody(const Vector3& nav) const
{
  const Quaternion::_w_type& w = pose().orientation.w;
  const Quaternion::_x_type& x = pose().orientation.x;
  const Quaternion::_y_type& y = pose().orientation.y;
  const Quaternion::_z_type& z = pose().orientation.z;
  Vector3 body;
  body.x = (w*w+x*x-y*y-z*z) * nav.x + (2.*x*y + 2.*w*z) * nav.y + (2.*x*z - 2.*w*y) * nav.z;
  body.y = (2.*x*y - 2.*w*z) * nav.x + (w*w-x*x+y*y-z*z) * nav.y + (2.*y*z + 2.*w*x) * nav.z;
  body.z = (2.*x*z + 2.*w*y) * nav.x + (2.*y*z - 2.*w*x) * nav.y + (w*w-x*x-y*y+z*z) * nav.z;
  return body;
}

Vector3 PoseHandle::fromBody(const Vector3& body) const
{
  const Quaternion::_w_type& w = pose().orientation.w;
  const Quaternion::_x_type& x = pose().orientation.x;
  const Quaternion::_y_type& y = pose().orientation.y;
  const Quaternion::_z_type& z = pose().orientation.z;
  Vector3 nav;
  nav.x = (w*w+x*x-y*y-z*z) * body.x + (2.*x*y - 2.*w*z) * body.y + (2.*x*z + 2.*w*y) * body.z;
  nav.y = (2.*x*y + 2.*w*z) * body.x + (w*w-x*x+y*y-z*z) * body.y + (2.*y*z - 2.*w*x) * body.z;
  nav.z = (2.*x*z - 2.*w*y) * body.x + (2.*y*z + 2.*w*x) * body.y + (w*w-x*x-y*y+z*z) * body.z;
  return nav;
}

void HorizontalPositionCommandHandle::getError(const PoseHandle &pose, double &x, double &y) const
{
  getCommand(x, y);
  x -= pose.get()->position.x;
  y -= pose.get()->position.y;
}

double HeightCommandHandle::getError(const PoseHandle &pose) const
{
  return getCommand() - pose.get()->position.z;
}

void HeadingCommandHandle::setCommand(double command)
{
  if (get()) {
    get()->x = 0.0;
    get()->y = 0.0;
    get()->z = sin(command / 2.);
    get()->w = cos(command / 2.);
  }

  if (scalar_) {
    *scalar_ = command;
  }
}

double HeadingCommandHandle::getCommand() const {
  if (scalar_) return *scalar_;
  const Quaternion::_w_type& w = get()->w;
  const Quaternion::_x_type& x = get()->x;
  const Quaternion::_y_type& y = get()->y;
  const Quaternion::_z_type& z = get()->z;
  return atan2(2.*x*y + 2.*w*z, x*x + w*w - z*z - y*y);
}

bool HeadingCommandHandle::update(Pose& command) const {
  if (get()) {
    command.orientation = *get();
    return true;
  }
  if (scalar_) {
    command.orientation.x = 0.0;
    command.orientation.y = 0.0;
    command.orientation.z = sin(*scalar_ / 2.);
    command.orientation.x = cos(*scalar_ / 2.);
    return true;
  }
  return false;
}

double HeadingCommandHandle::getError(const PoseHandle &pose) const {
  static const double M_2PI = 2.0 * M_PI;
  double error = getCommand() - pose.getYaw() + M_PI;
  error -= floor(error / M_2PI) * M_2PI;
  return error - M_PI;
}

bool CommandHandle::enabled()    { return interface_->enabled(this); }
bool CommandHandle::start()      { return interface_->start(this); }
void CommandHandle::stop()       { interface_->stop(this); }
void CommandHandle::disconnect() { interface_->disconnect(this); }

} // namespace hector_quadrotor_controller
