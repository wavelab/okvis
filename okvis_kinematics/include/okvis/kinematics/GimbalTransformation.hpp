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
 *  Created on: Sep 5, 2017
 *      Author: Leo Koppel (lkoppel@uwaterloo.ca)
 *********************************************************************************/

/**
 * @file kinematics/DhParameters.hpp
 * @brief Header file for the GimbalTransformation class.
 * @author Leo Koppel
 */

#ifndef INCLUDE_OKVIS_GIMBALTRANSFORMATION_HPP_
#define INCLUDE_OKVIS_GIMBALTRANSFORMATION_HPP_

#include "okvis/kinematics/DhParameters.hpp"

/// \brief okvis Main namespace of this package.
namespace okvis {

/// \brief kinematics Namespace for kinematics functionality, i.e. transformations and stuff.
namespace kinematics {

/**
 * \brief A transformation composed by a kinematic chain, as on a gimbal (or robot arm).
 * \tparam N number of joints
 *
 * The joint angles (theta) are variable parameters, while the other parameters are constant.
 *
 * The overall transformation is here called T_SC, representing a transform from IMU sensor (S) to camera frame (C)
 *
 * This transform is composed of
 *   - T_SA, static transform from S to the base of the kinematic chain (A)
 *   - `N` sets of four DH parameters, together comprising T_AE
 *   - T_EC, static transform from end effector (E) to C
 */
template <int N>
class GimbalTransformation {

  template <typename... DhArgs>
  GimbalTransformation(Transformation T_EC, Transformation T_SA, DhArgs... dh) :
      T_EC_{std::move(T_EC)},
      T_SA_{std::move(T_SA)},
      dhChain_{dh...}
  {
    static_assert(N == sizeof...(dh), "Wrong number of DH parameter structs passes to contructor.");

    for (auto i = 0u; i < N; ++i) {
      parameters_[i] = dhChain_[i].theta;
    }
    updateCache();
  }

  /// \brief Parameter setting, all 7.
  /// \tparam Derived_coeffs Deducible matrix type.
  /// @param[in] parameters The parameters as theta angles in radians.
  template<typename Derived>
  bool setParameters(const Eigen::MatrixBase<Derived> & parameters) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 7);
    parameters_ = parameters;
    updateCache();
    return true;
  }

  /// \brief The overall transformation, T_SC
  Transformation overallT() const {
    return cachedT_SC_;
  }

  /// \brief The overall homogeneous transformation matrix, T_SC
  Eigen::Matrix4d T() const {
    return cachedT_SC_.T();
  }

  /// \brief Returns the rotation matrix (cached).
  const Eigen::Matrix3d & C() const {
    return overallT().C();
  }

  /// \brief Returns the translation vector r_AB (represented in frame A).
  const Eigen::Map<Eigen::Vector3d> & r() const  {
    return overallT().r();
  }

  /// \brief Returns the Quaternion q_AB (as an Eigen Quaternion).
  const Eigen::Map<Eigen::Quaterniond> & q() const  {
    return overallT().q();
  }

  /// \brief Get the upper 3x4 part of the homogeneous transformation matrix T_AB.
  Eigen::Matrix<double, 3, 4> T3x4() const  {
    return overallT().T3x4();
  }

  /// \brief The parameters as theta angles in radians.
  const Eigen::Matrix<double, N, 1> & parameters() const
  {
    return parameters_;
  }

  /// \brief Get the parameters --- support for ceres.
  /// \warning USE WITH CARE!
  const double* parameterPtr() const
  {
    return &parameters_[0];
  }

  /// \brief Returns a copy of the transformation inverted.
  Transformation inverse() const {
    return overallT().inverse();
  }

 protected:
  /// \brief Update the caching of the overall transformation
  void updateCache() {
    auto T_AE = Transformation{};
    for (const auto& dh : dhChain_) {
      T_AE = T_AE * transformationFromDh(dh);
    }
    cachedT_SC_ = T_SA_ * T_EC_;
  }

  Transformation T_EC_; ///< Static transform from end effector (E) to camera frame (C)
  Transformation T_SA_; ///< Static transform from IMU (S) to base of the kinematic chain (A)
  std::array<DhParameters, N> dhChain_; ///< chain of DH parameters comprising a transform from A to E. theta is initial
  Eigen::Matrix<double, N, 1> parameters_; ///< changing theta value from each dh joint
  Transformation cachedT_SC_; ///< The cached overall transformation T_SC.
};

}  // namespace kinematics
}  // namespace okvis

#endif /* INCLUDE_OKVIS_GIMBALTRANSFORMATION_HPP_ */
