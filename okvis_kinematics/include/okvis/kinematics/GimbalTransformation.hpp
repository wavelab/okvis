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

#include "okvis/kinematics/TransformationBase.hpp"
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
 *   - `N` sets of four DH parameters, together composing T_AE
 *   - T_EC, static transform from end effector (E) to C
 */
template <int N>
class GimbalTransformation : public TransformationBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /// \brief Default constructor: initialises an identity transformation.
  GimbalTransformation();

  /// \brief Construct with given base and end transformations and DH parameters
  template <typename... DhArgs>
  GimbalTransformation(Transformation T_SA, Transformation T_EC, DhArgs... dh);

  /// \brief Construct copying other GimbalTransformation
  GimbalTransformation(const GimbalTransformation&) = default;

  /// \brief Convert to simple Transformation type
  explicit operator Transformation() const override {
    return this->cachedT_SC_;
  }

  /// \brief Parameter setting, all 7.
  /// \tparam Derived_coeffs Deducible matrix type.
  /// @param[in] parameters The parameters as theta angles in radians.
  template<typename Derived>
  bool setParameters(const Eigen::MatrixBase<Derived> & parameters);

  /// \brief Return a copy of overall transformation, T_SC
  Transformation overallT() const;

  /// \brief The overall homogeneous transformation matrix, T_SC
  Eigen::Matrix4d T() const override;

  /// \brief Returns the rotation matrix (cached).
  const Eigen::Matrix3d & C() const override;

  /// \brief Returns the translation vector r_AB (represented in frame A).
  const Eigen::Map<Eigen::Vector3d> & r() const override;

  /// \brief Returns the Quaternion q_AB (as an Eigen Quaternion).
  const Eigen::Map<Eigen::Quaterniond> & q() const override;

  /// \brief Get the upper 3x4 part of the homogeneous transformation matrix T_AB.
  Eigen::Matrix<double, 3, 4> T3x4() const override;

  /// \brief The parameters as theta angles in radians.
  const Eigen::Matrix<double, N, 1> & parameters() const;

  /// \brief Get the parameters --- support for ceres.
  /// \warning USE WITH CARE!
  const double* parameterPtr() const override;

  /// \brief Set this to a random transformation.
  void setRandom() override;

  /// \brief Returns a copy of the transformation inverted.
  Transformation inverse() const override;

  // operator* (group operator)
  /// \brief Multiplication with another transformation object.
  /// @param[in] rhs The right-hand side transformation for this to be multiplied with.
  Transformation operator*(const TransformationBase & rhs) const override {
    return cachedT_SC_ * rhs;
  }

  /// \brief Transform a direction as v_A = C_AB*v_B (with rhs = hp_B)..
  /// \warning This only applies the rotation!
  /// @param[in] rhs The right-hand side direction for this to be multiplied with.
  Eigen::Vector3d operator*(const Eigen::Vector3d & rhs) const override  {
    return cachedT_SC_ * rhs;
  }

  /// \brief Transform a homogenous point as hp_B = T_AB*hp_B (with rhs = hp_B).
  /// @param[in] rhs The right-hand side direction for this to be multiplied with.
  Eigen::Vector4d operator*(const Eigen::Vector4d & rhs) const override  {
    return cachedT_SC_ * rhs;
  }

  /// \brief Assignment -- copy. Takes care of proper caching.
  /// @param[in] rhs The rhs for this to be assigned to.
  GimbalTransformation& operator=(const GimbalTransformation & rhs) = default;

  /// \brief Apply a small update with delta being Nx1.
  /// \tparam Derived_delta Deducible matrix type.
  /// @param[in] delta The Nx1 minimal update.
  /// \return True on success.
  template<typename Derived_delta>
  bool oplus(const Eigen::MatrixBase<Derived_delta> & delta);

  /// \brief Apply a small update with delta being Nx1 --
  ///        the Jacobian is a 7 by N matrix (change in pose wrt the joint parameters)
  /// @param[in] delta The Nx1 minimal update.
  /// @param[out] jacobian The output Jacobian.
  /// \return True on success.
  /// \note Note output parameter is cast to non-const: see https://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html
  template<typename Derived_delta, typename Derived_jacobian>
  bool oplus(const Eigen::MatrixBase<Derived_delta> & delta,
             const Eigen::MatrixBase<Derived_jacobian> & jacobian);

  /// \brief Get the Jacobian of the tangent vector w.r.t. the joint angles (a 6 by N matrix).
  /// @param[out] jacobian The output Jacobian.
  /// \return True on success.
  /// \note Note output parameter is cast to non-const: see https://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html
  template<typename Derived_jacobian>
  bool oplusMinimalJacobian(
      const Eigen::MatrixBase<Derived_jacobian> & jacobian) const;

  /// \brief Get the Jacobian of the oplus operation (a 7 by N matrix).
  /// @param[out] jacobian The output Jacobian.
  /// \return True on success.
  /// \note Note output parameter is cast to non-const: see https://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html
  template<typename Derived_jacobian>
  bool oplusJacobian(
      const Eigen::MatrixBase<Derived_jacobian> & jacobian) const;

  /// \brief Gets the jacobian dx/dChi,
  ///        i.e. lift the minimal Jacobian to a full one (as needed by ceres).
  // @param[out] jacobian The output lift Jacobian (N by 7 matrix).
  /// \return True on success.
  template<typename Derived_jacobian>
  bool liftJacobian(const Eigen::MatrixBase<Derived_jacobian> & jacobian) const;

 protected:
  /// \brief initialize the cache of intermediate transformations with identity
  void initCache();

  /// \brief Update the cache of intermediate transformations
  void updateCache();

  Transformation T_SA_; ///< Static transform from IMU (S) to base of the kinematic chain (A)
  Transformation T_EC_; ///< Static transform from end effector (E) to camera frame (C)
  std::array<DhParameters, N> dhChain_; ///< chain of DH parameters  a transform from A to E. theta is initial
  Eigen::Matrix<double, N, 1> parameters_; ///< changing theta value from each dh joint
  Transformation cachedT_SC_; ///< The cached overall transformation T_SC.

  /// Cache of intermediate transformations. cache_T[a, b] is the transform from the ath to (a+b+1)th frame
  /// cached_T[0, 0] is T_SA, cached_T[1, 0] is T_AL(1), cached_T[0,N+1] is T_SC
  /// Only forward transformations are stored
  std::array<std::vector<okvis::kinematics::Transformation,
                         Eigen::aligned_allocator<okvis::kinematics::Transformation>>, N + 2> cached_T;
};

}  // namespace kinematics
}  // namespace okvis

#include "implementation/GimbalTransformation.hpp"

#endif /* INCLUDE_OKVIS_GIMBALTRANSFORMATION_HPP_ */
