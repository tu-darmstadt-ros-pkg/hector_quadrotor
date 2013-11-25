//=================================================================================================
// Copyright (c) 2013, Johannes Meyer and contributors, Technische Universitat Darmstadt
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

#ifndef HECTOR_QUADROTOR_MODEL_HELPERS_H
#define HECTOR_QUADROTOR_MODEL_HELPERS_H

#include <geometry_msgs/Wrench.h>

namespace hector_quadrotor_model
{

template <typename T> static inline void checknan(T& value, const std::string& text = "") {
  if (!(value == value)) {
#ifndef NDEBUG
    if (!text.empty()) std::cerr << text << " contains **!?* Nan values!" << std::endl;
#endif
    value = T();
    return;
  }
}

template <typename Message, typename Vector> static inline void toVector(const Message& msg, Vector& vector)
{
  vector.x = msg.x;
  vector.y = msg.y;
  vector.z = msg.z;
}

template <typename Message, typename Vector> static inline void fromVector(const Vector& vector, Message& msg)
{
  msg.x = vector.x;
  msg.y = vector.y;
  msg.z = vector.z;
}

template <typename Message, typename Quaternion> static inline void toQuaternion(const Message& msg, Quaternion& vector)
{
  vector.w = msg.w;
  vector.x = msg.x;
  vector.y = msg.y;
  vector.z = msg.z;
}

template <typename Message, typename Quaternion> static inline void fromQuaternion(const Quaternion& vector, Message& msg)
{
  msg.w = vector.w;
  msg.x = vector.x;
  msg.y = vector.y;
  msg.z = vector.z;
}

static inline geometry_msgs::Vector3 operator+(const geometry_msgs::Vector3& a, const geometry_msgs::Vector3& b)
{
  geometry_msgs::Vector3 result;
  result.x = a.x + b.x;
  result.y = a.y + b.y;
  result.z = a.z + b.z;
  return result;
}

static inline geometry_msgs::Wrench operator+(const geometry_msgs::Wrench& a, const geometry_msgs::Wrench& b)
{
  geometry_msgs::Wrench result;
  result.force = a.force + b.force;
  result.torque = a.torque + b.torque;
  return result;
}

}

#endif // HECTOR_QUADROTOR_MODEL_HELPERS_H
