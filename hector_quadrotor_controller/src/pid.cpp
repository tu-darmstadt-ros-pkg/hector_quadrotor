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

#include <hector_quadrotor_controller/pid.h>
#include <limits>

namespace hector_quadrotor_controller {

PID::state::state()
  : p(std::numeric_limits<double>::quiet_NaN())
  , i(0.0)
  , d(std::numeric_limits<double>::quiet_NaN())
  , input(std::numeric_limits<double>::quiet_NaN())
  , dinput(0.0)
  , dx(std::numeric_limits<double>::quiet_NaN())
{
}

PID::parameters::parameters()
  : enabled(true)
  , time_constant(0.0)
  , k_p(0.0)
  , k_i(0.0)
  , k_d(0.0)
  , limit_i(std::numeric_limits<double>::quiet_NaN())
  , limit_output(std::numeric_limits<double>::quiet_NaN())
{
}

PID::PID()
{}

PID::PID(const parameters& params)
  : parameters_(params)
{}

PID::~PID()
{}

void PID::init(const ros::NodeHandle &param_nh)
{
  param_nh.getParam("enabled", parameters_.enabled);
  param_nh.getParam("k_p", parameters_.k_p);
  param_nh.getParam("k_i", parameters_.k_i);
  param_nh.getParam("k_d", parameters_.k_d);
  param_nh.getParam("limit_i", parameters_.limit_i);
  param_nh.getParam("limit_output", parameters_.limit_output);
  param_nh.getParam("time_constant", parameters_.time_constant);
}

void PID::reset()
{
  state_ = state();
}

template <typename T> static inline T& checknan(T& value)
{
  if (std::isnan(value)) value = T();
  return value;
}

double PID::update(double input, double x, double dx, const ros::Duration& dt)
{
  if (!parameters_.enabled) return 0.0;
  double dt_sec = dt.toSec();

  // low-pass filter input
  if (std::isnan(state_.input)) state_.input = input;
  if (dt_sec + parameters_.time_constant > 0.0) {
    state_.dinput = (input - state_.input) / (dt_sec + parameters_.time_constant);
    state_.input  = (dt_sec * input + parameters_.time_constant * state_.input) / (dt_sec + parameters_.time_constant);
  }

  return update(state_.input - x, dx, dt);
}

double PID::update(double error, double dx, const ros::Duration& dt)
{
  if (!parameters_.enabled) return 0.0;
  if (std::isnan(error)) return 0.0;
  double dt_sec = dt.toSec();

  // integral error
  state_.i += error * dt_sec;
  if (parameters_.limit_i > 0.0)
  {
    if (state_.i >  parameters_.limit_i) state_.i =  parameters_.limit_i;
    if (state_.i < -parameters_.limit_i) state_.i = -parameters_.limit_i;
  }

  // differential error
  if (dt_sec > 0.0 && !std::isnan(state_.p) && !std::isnan(state_.dx)) {
    state_.d = (error - state_.p) / dt_sec + state_.dx - dx;
  } else {
    state_.d = -dx;
  }
  state_.dx = dx;

  // proportional error
  state_.p = error;

  // calculate output...
  double output = parameters_.k_p * state_.p + parameters_.k_i * state_.i + parameters_.k_d * state_.d;
  int antiwindup = 0;
  if (parameters_.limit_output > 0.0)
  {
    if (output >  parameters_.limit_output) { output =  parameters_.limit_output; antiwindup =  1; }
    if (output < -parameters_.limit_output) { output = -parameters_.limit_output; antiwindup = -1; }
  }
  if (antiwindup && (error * dt_sec * antiwindup > 0.0)) state_.i -= error * dt_sec;

  checknan(output);
  return output;
}

double PID::getFilteredControlError(double& filtered_error, double time_constant, const ros::Duration& dt)
{
  double dt_sec = dt.toSec();
  filtered_error = checknan(filtered_error);
  if (dt_sec + time_constant > 0.0) {
    filtered_error = (time_constant * filtered_error + dt_sec * state_.p) / (dt_sec + time_constant);
  }
  return filtered_error;
}

} // namespace hector_quadrotor_controller

