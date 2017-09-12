/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * 
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: Jan 10, 2015
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file implementation/Estimator.hpp
 * @brief Header implementation file for the Estimator class.
 * @author Stefan Leutenegger
 */

/// \brief okvis Main namespace of this package.
namespace okvis {

// Add an observation to a landmark.
template<class GEOMETRY_TYPE>
::ceres::ResidualBlockId Estimator::addObservation(uint64_t landmarkId,
                                                   uint64_t poseId,
                                                   size_t camIdx,
                                                   size_t keypointIdx) {
  LOG(INFO) << "Adding observation of landmark " << landmarkId << " at " << poseId << ", cam " << camIdx;
  OKVIS_ASSERT_TRUE_DBG(Exception, isLandmarkAdded(landmarkId),
                        "landmark not added");

  // avoid double observations
  okvis::KeypointIdentifier kid(poseId, camIdx, keypointIdx);
  if (landmarksMap_.at(landmarkId).observations.find(kid)
      != landmarksMap_.at(landmarkId).observations.end()) {
    return NULL;
  }

  // get the keypoint measurement
  okvis::MultiFramePtr multiFramePtr = multiFramePtrMap_.at(poseId);
  Eigen::Vector2d measurement;
  multiFramePtr->getKeypoint(camIdx, keypointIdx, measurement);
  Eigen::Matrix2d information = Eigen::Matrix2d::Identity();
  double size = 1.0;
  multiFramePtr->getKeypointSize(camIdx, keypointIdx, size);
  information *= 64.0 / (size * size);

  // create error term
  auto reprojectionError = std::shared_ptr<ceres::ReprojectionError<GEOMETRY_TYPE>>{
      new ceres::ReprojectionError<GEOMETRY_TYPE>(
      multiFramePtr->template geometryAs<GEOMETRY_TYPE>(camIdx),
      camIdx,
      measurement,
      information,
      multiFramePtr->T_SC(camIdx))};

  uint64_t T_SC_param_id;
  if (statesMap_.at(poseId).sensors.at(SensorStates::Camera).at(camIdx).at(
      CameraSensorStates::GimbalAngles).exists) {
    T_SC_param_id = statesMap_.at(poseId).sensors.at(SensorStates::Camera).at(camIdx).at(
        CameraSensorStates::GimbalAngles).id;
  } else {
    T_SC_param_id = statesMap_.at(poseId).sensors.at(SensorStates::Camera).at(camIdx).at(
        CameraSensorStates::T_SCi).id;
  }

  ::ceres::ResidualBlockId retVal = mapPtr_->addResidualBlock(
      reprojectionError,
      cauchyLossFunctionPtr_ ? cauchyLossFunctionPtr_.get() : NULL,
      mapPtr_->parameterBlockPtr(poseId),
      mapPtr_->parameterBlockPtr(landmarkId),
      mapPtr_->parameterBlockPtr(T_SC_param_id));

  // remember
  landmarksMap_.at(landmarkId).observations.insert(
      std::pair<okvis::KeypointIdentifier, uint64_t>(
          kid, reinterpret_cast<uint64_t>(retVal)));

  LOG(INFO) << "Done adding observation of landmark " << landmarkId << " at " << poseId;

  return retVal;
}

}  // namespace okvis
