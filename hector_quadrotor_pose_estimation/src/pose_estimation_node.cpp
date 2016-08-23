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

#include <hector_quadrotor_pose_estimation/pose_estimation_node.h>
#include <hector_pose_estimation/measurements/baro.h>

namespace hector_quadrotor_pose_estimation {

QuadrotorPoseEstimationNode::QuadrotorPoseEstimationNode(const SystemPtr& system, const StatePtr& state)
  : PoseEstimationNode(system, state)
{
  pose_estimation_->addMeasurement(new Baro("baro"));
}

QuadrotorPoseEstimationNode::~QuadrotorPoseEstimationNode()
{
}

bool QuadrotorPoseEstimationNode::init() {
  if (!PoseEstimationNode::init()) return false;
  baro_subscriber_ = getNodeHandle().subscribe("altimeter", 10, &QuadrotorPoseEstimationNode::baroCallback, this);
  height_subscriber_.shutdown();
  return true;
}

void QuadrotorPoseEstimationNode::baroCallback(const hector_uav_msgs::AltimeterConstPtr& altimeter) {
  pose_estimation_->getMeasurement("baro")->add(Baro::Update(altimeter->pressure, altimeter->qnh));

  if (sensor_pose_publisher_) {
    boost::shared_ptr<Baro> baro = boost::static_pointer_cast<Baro>(pose_estimation_->getMeasurement("baro"));
    sensor_pose_.pose.position.z = baro->getModel()->getAltitude(Baro::Update(altimeter->pressure, altimeter->qnh)) - baro->getElevation();
  }
}

} // namespace hector_quadrotor_pose_estimation
