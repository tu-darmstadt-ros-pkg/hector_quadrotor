//=================================================================================================
// Copyright (c) 2011, Johannes Meyer, TU Darmstadt
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

#ifndef HECTOR_QUADROTOR_POSE_ESTIMATION_NODE_H
#define HECTOR_QUADROTOR_POSE_ESTIMATION_NODE_H

#include <hector_pose_estimation/pose_estimation_node.h>
#include <hector_uav_msgs/Altimeter.h>

namespace hector_quadrotor_pose_estimation {

using namespace hector_pose_estimation;

class QuadrotorPoseEstimationNode : public PoseEstimationNode {
public:
  QuadrotorPoseEstimationNode(const SystemPtr& system = SystemPtr(), const StatePtr& state = StatePtr());
  virtual ~QuadrotorPoseEstimationNode();

  virtual bool init();

protected:
  void baroCallback(const hector_uav_msgs::AltimeterConstPtr& altimeter);

private:
  ros::Subscriber baro_subscriber_;
};

} // namespace hector_quadrotor_pose_estimation

#endif // HECTOR_QUADROTOR_POSE_ESTIMATION_NODE_H
