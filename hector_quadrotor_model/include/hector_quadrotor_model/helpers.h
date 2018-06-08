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

#ifndef HECTOR_QUADROTOR_MODEL_HELPERS_H
#define HECTOR_QUADROTOR_MODEL_HELPERS_H

#include <geometry_msgs/Wrench.h>
#include <boost/range/iterator_range.hpp>

namespace hector_quadrotor_model
{

template <typename T> int isnan(const T& value) {
  return std::isnan(value);
}

template <typename T, std::size_t N> int isnan(const boost::array<T,N>& array) {
  for(typename boost::array<T,N>::const_iterator it = array.begin(); it != array.end(); ++it)
    if (std::isnan(*it)) return std::isnan(*it);
  return false;
}

template <typename IteratorT> int isnan(const boost::iterator_range<IteratorT>& range, const typename boost::iterator_range<IteratorT>::value_type& min, const typename boost::iterator_range<IteratorT>::value_type& max) {
  for(IteratorT it = range.begin(); it != range.end(); ++it)
    if (std::isnan(*it)) return std::isnan(*it);
  return false;
}

template <typename T> int isinf(const T& value) {
  return std::isinf(value);
}

template <typename T, std::size_t N> int isinf(const boost::array<T,N>& array) {
  for(typename boost::array<T,N>::const_iterator it = array.begin(); it != array.end(); ++it)
    if (std::isinf(*it)) return std::isinf(*it);
  return false;
}

template <typename IteratorT> int isinf(const boost::iterator_range<IteratorT>& range, const typename boost::iterator_range<IteratorT>::value_type& min, const typename boost::iterator_range<IteratorT>::value_type& max) {
  for(IteratorT it = range.begin(); it != range.end(); ++it)
    if (std::isinf(*it)) return std::isinf(*it);
  return false;
}

template <typename T> void limit(T& value, const T& min, const T& max) {
  if (!isnan(min) && value < min) value = min;
  if (!isnan(max) && value > max) value = max;
}

template <typename T, std::size_t N> void limit(boost::array<T,N>& array, const T& min, const T& max) {
  for(typename boost::array<T,N>::iterator it = array.begin(); it != array.end(); ++it)
    limit(*it, min, max);
}

template <typename IteratorT> void limit(const boost::iterator_range<IteratorT>& range, const typename boost::iterator_range<IteratorT>::value_type& min, const typename boost::iterator_range<IteratorT>::value_type& max) {
  for(IteratorT it = range.begin(); it != range.end(); ++it)
    limit(*it, min, max);
}

template <typename T> static inline void checknan(T& value, const std::string& text = "") {
  if (isnan(value)) {
#ifndef NDEBUG
    if (!text.empty()) std::cerr << text << " contains **!?* Nan values!" << std::endl;
#endif
    value = T();
    return;
  }

  if (isinf(value)) {
#ifndef NDEBUG
    if (!text.empty()) std::cerr << text << " is +-Inf!" << std::endl;
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

template <typename Message, typename Quaternion> static inline void toQuaternion(const Message& msg, Quaternion& quaternion)
{
  quaternion.w = msg.w;
  quaternion.x = msg.x;
  quaternion.y = msg.y;
  quaternion.z = msg.z;
}

template <typename Message, typename Quaternion> static inline void fromQuaternion(const Quaternion& quaternion, Message& msg)
{
  msg.w = quaternion.w;
  msg.x = quaternion.x;
  msg.y = quaternion.y;
  msg.z = quaternion.z;
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

template<typename T>
class PrintVector {
public:
  typedef const T* const_iterator;
  PrintVector(const_iterator begin, const_iterator end, const std::string &delimiter = "[ ]") : begin_(begin), end_(end), delimiter_(delimiter) {}
  const_iterator begin() const { return begin_; }
  const_iterator end() const { return end_; }
  std::size_t size() const { return end_ - begin_; }

  std::ostream& operator>>(std::ostream& os) const {
    if (!delimiter_.empty()) os << delimiter_.substr(0, delimiter_.size() - 1);
    for(const_iterator it = begin(); it != end(); ++it) {
      if (it != begin()) os << " ";
      os << *it;
    }
    if (!delimiter_.empty()) os << delimiter_.substr(1, delimiter_.size() - 1);
    return os;
  }

private:
  const_iterator begin_, end_;
  std::string delimiter_;
};
template <typename T> std::ostream &operator<<(std::ostream& os, const PrintVector<T>& vector) { return vector >> os; }

}

#endif // HECTOR_QUADROTOR_MODEL_HELPERS_H
