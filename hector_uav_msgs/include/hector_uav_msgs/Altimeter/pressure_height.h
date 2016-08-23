//=================================================================================================
// Copyright (c) 2012-2016, Institute of Flight Systems and Automatic Control,
// Technische Universität Darmstadt.
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of hector_quadrotor nor the names of its contributors
//       may be used to endorse or promote products derived from this software
//       without specific prior written permission.

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

#ifndef HECTOR_UAV_MSGS_ALTIMETER_PRESSURE_HEIGHT_H
#define HECTOR_UAV_MSGS_ALTIMETER_PRESSURE_HEIGHT_H

#include <hector_uav_msgs/Altimeter.h>
#include <cmath>

namespace hector_uav_msgs {

static const Altimeter::_qnh_type STANDARD_PRESSURE = 1013.25;

static inline Altimeter::_altitude_type altitudeFromPressure(Altimeter::_pressure_type pressure, Altimeter::_qnh_type qnh = STANDARD_PRESSURE) {
  return 288.15 / 0.0065 * (1.0 - pow(pressure / qnh, 1.0/5.255));
}

static inline Altimeter::_pressure_type pressureFromAltitude(Altimeter::_altitude_type altitude, Altimeter::_qnh_type qnh = STANDARD_PRESSURE) {
  return qnh * pow(1.0 - (0.0065 * altitude) / 288.15, 5.255);
}

static inline Altimeter& altitudeFromPressure(Altimeter& altimeter) {
  if (altimeter.qnh == 0.0) altimeter.qnh = STANDARD_PRESSURE;
  altimeter.altitude = altitudeFromPressure(altimeter.pressure, altimeter.qnh);
  return altimeter;
}

static inline Altimeter& pressureFromAltitude(Altimeter& altimeter) {
  if (altimeter.qnh == 0.0) altimeter.qnh = STANDARD_PRESSURE;
  altimeter.pressure = pressureFromAltitude(altimeter.altitude, altimeter.qnh);
  return altimeter;
}

} // namespace hector_uav_msgs

#endif // HECTOR_UAV_MSGS_ALTIMETER_PRESSURE_HEIGHT_H
