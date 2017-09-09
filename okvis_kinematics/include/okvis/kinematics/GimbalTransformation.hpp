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
  GimbalTransformation(Transformation T_SA, Transformation T_EC, DhArgs... dh) :
      T_SA_{std::move(T_SA)},
      T_EC_{std::move(T_EC)},
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

  /// \brief the jacobian of the overall transformation T_SC, w.r.t to joint angle parameters
  /// Note output parameter is cast to non-const: see https://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html
  template <typename Derived_jacobian>
  bool overallTJacobian(const Eigen::MatrixBase<Derived_jacobian> & jacobianOut) {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived_jacobian, 6, N);
    auto& jacobian = const_cast<Eigen::MatrixBase<Derived_jacobian>&>(jacobianOut);

    // T_{SC} = T_{SA} * T_{A{L_1}} * ... * T_{{L_N}E} * T_{EC};
    // Use the product rule for N functions


    return false;
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

  /// \brief Assignment -- copy. Takes care of proper caching.
  /// @param[in] rhs The rhs for this to be assigned to.
  Transformation& operator=(const Transformation & rhs);

  /// \brief Apply a small update with delta being Nx1.
  /// \tparam Derived_delta Deducible matrix type.
  /// @param[in] delta The Nx1 minimal update.
  /// \return True on success.
  template<typename Derived_delta>
  bool oplus(const Eigen::MatrixBase<Derived_delta> & delta)  {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived_delta, N);
    parameters_ += delta;
    updateCache();
    return true;
  }

  /// \brief Apply a small update with delta being Nx1 --
  ///        the Jacobian is a 7 by N matrix (change in pose wrt the joint parameters)
  /// @param[in] delta The Nx1 minimal update.
  /// @param[out] jacobian The output Jacobian.
  /// \return True on success.
  template<typename Derived_delta, typename Derived_jacobian>
  bool oplus(const Eigen::MatrixBase<Derived_delta> & delta,
             const Eigen::MatrixBase<Derived_jacobian> & jacobian) {
    EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived_delta, N);
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived_jacobian, 7, N);
    if (!oplus(delta)) {
      return false;
    }
    return oplusJacobian(jacobian);
  }

  /// \brief Get the Jacobian of the oplus operation (a 7 by N matrix).
  /// @param[out] jacobian The output Jacobian.
  /// \return True on success.
  template<typename Derived_jacobian>
  bool oplusJacobian(
      const Eigen::MatrixBase<Derived_jacobian> & jacobian) const {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived_jacobian, 7, N);

    // For each transformation in the chain, working backward
    for (auto i = N - 1; i >= 0; --i) {
      // calculate the Jacobian of the change wrt theta
      const auto omega = dhChain_[i].omega();

    }

    // First, get the 7x6 Jacobian wrt the 6x1 delta obtained from the N joint angles and 3N constant DH parameters
    Eigen::Matrix<double, 7, 6> baseJacobian;
    if (!overallT().oplusJacobian(baseJacobian)) {
      return false;
    }

    // Then use chain rule


    return true;
  }

  /// \brief Gets the jacobian dx/dChi,
  ///        i.e. lift the minimal Jacobian to a full one (as needed by ceres).
  // @param[out] jacobian The output lift Jacobian (N by N identity matrix).
  /// \return True on success.
  template<typename Derived_jacobian>
  bool liftJacobian(const Eigen::MatrixBase<Derived_jacobian> & jacobian) const {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived_jacobian, N, N);
    const_cast<Eigen::MatrixBase<Derived_jacobian>&>(jacobian).setIdentity();
    return true;
  }

 protected:
  /// \brief Update the cache of intermediate transformations
  void updateCache() {
    // Calculate adjacent transforms first
    cached_T[0].push_back(T_SA_);
    for (auto link = 0; link < N; ++link) {
      cached_T[link + 1].push_back(transformationFromDh(dhChain_[link]));
    }

    // Calculate transforms spaced by 2, then 3, and so on
    for (auto jump = 2; jump <= N + 1; ++jump) {
      for (auto start = 0; start <= N + 1 - jump; ++start) {
        const auto end = start + jump;
        cached_T[start].push_back(cached_T[start].back() * cached_T[end - 1].front());
      }
    }

    // Calculate T_SC = T_SE * T_EC
    cachedT_SC_ = cached_T.front().back() * T_EC_;
  }

  Transformation T_SA_; ///< Static transform from IMU (S) to base of the kinematic chain (A)
  Transformation T_EC_; ///< Static transform from end effector (E) to camera frame (C)
  std::array<DhParameters, N> dhChain_; ///< chain of DH parameters comprising a transform from A to E. theta is initial
  Eigen::Matrix<double, N, 1> parameters_; ///< changing theta value from each dh joint
  Transformation cachedT_SC_; ///< The cached overall transformation T_SC.

  /// Cache of intermediate transformations. cache_T[a, b] is the transform from the ath to (a+b+1)th frame
  /// cached_T[0, 0] is T_SA, cached_T[1, 0] is T_AL(1), cached_T[0,N+1] is T_SC
  /// Only forward transformations are stored, and no transformation ending in C is stored
  std::array<std::vector<okvis::kinematics::Transformation,
                          Eigen::aligned_allocator<okvis::kinematics::Transformation>>, N + 1> cached_T;
};

}  // namespace kinematics
}  // namespace okvis

#endif /* INCLUDE_OKVIS_GIMBALTRANSFORMATION_HPP_ */
