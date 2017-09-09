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
 * @brief Header implementation file for the GimbalTransformation class.
 * @author Leo Koppel
 */

/// \brief okvis Main namespace of this package.
namespace okvis {

/// \brief kinematics Namespace for kinematics functionality, i.e. transformations and stuff.
namespace kinematics {

template <int N>
GimbalTransformation<N>::GimbalTransformation() {
  initCache();
  updateCache();
}

template <int N>
template <typename... DhArgs>
GimbalTransformation<N>::GimbalTransformation(Transformation T_SA, Transformation T_EC, DhArgs... dh) :
    T_SA_{std::move(T_SA)},
    T_EC_{std::move(T_EC)},
    dhChain_{dh...}
{
  static_assert(N == sizeof...(dh), "Wrong number of DH parameter structs passed to contructor.");

  for (auto i = 0u; i < N; ++i) {
    parameters_[i] = dhChain_[i].theta;
  }
  initCache();
  updateCache();
}

template <int N>
template<typename Derived>
bool GimbalTransformation<N>::setParameters(const Eigen::MatrixBase<Derived> & parameters) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, N);
  parameters_ = parameters;
  updateCache();
  return true;
}

template <int N>
Transformation GimbalTransformation<N>::overallT() const {
  return cachedT_SC_;
}

template <int N>
Eigen::Matrix4d GimbalTransformation<N>::T() const {
  return cachedT_SC_.T();
}

template <int N>
const Eigen::Matrix3d & GimbalTransformation<N>::C() const {
  return cachedT_SC_.C();
}

template <int N>
const Eigen::Map<Eigen::Vector3d> & GimbalTransformation<N>::r() const  {
  return cachedT_SC_.r();
}

template <int N>
const Eigen::Map<Eigen::Quaterniond> & GimbalTransformation<N>::q() const  {
  return cachedT_SC_.q();
}

template <int N>
Eigen::Matrix<double, 3, 4> GimbalTransformation<N>::T3x4() const  {
  return overallT().T3x4();
}

template <int N>
const Eigen::Matrix<double, N, 1> & GimbalTransformation<N>::parameters() const
{
  return parameters_;
}

template <int N>
const double* GimbalTransformation<N>::parameterPtr() const
{
  return &parameters_[0];
}

template <int N>
void GimbalTransformation<N>::setRandom() {
  for (auto& dh : dhChain_) {
    dh.setRandom();
  }
  T_SA_.setRandom();
  T_EC_.setRandom();
  updateCache();
}

template <int N>
Transformation GimbalTransformation<N>::inverse() const {
  return cachedT_SC_.inverse();
}

template <int N>
template<typename Derived_delta>
bool GimbalTransformation<N>::oplus(const Eigen::MatrixBase<Derived_delta> & delta)  {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived_delta, N);
  parameters_ += delta;
  updateCache();
  return true;
}

template <int N>
template<typename Derived_delta, typename Derived_jacobian>
bool GimbalTransformation<N>::oplus(const Eigen::MatrixBase<Derived_delta> & delta,
           const Eigen::MatrixBase<Derived_jacobian> & jacobian) {
  EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived_delta, N);
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived_jacobian, 7, N);
  if (!oplus(delta)) {
    return false;
  }
  return oplusJacobian(jacobian);
}

template <int N>
template<typename Derived_jacobian>
bool GimbalTransformation<N>::oplusMinimalJacobian(
    const Eigen::MatrixBase<Derived_jacobian> & jacobian) const {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived_jacobian, 6, N);
  auto &J_SC_theta = const_cast<Eigen::MatrixBase<Derived_jacobian> &>(jacobian);
  using Scalar = typename Derived_jacobian::Scalar;

  // Each DH link affects only column of the jacobian
  // For each link i the jacobian wrt theta_i is
  //   dp_SC/dp_Si * dp_Si / dp_(i-1)i * dp_(i-1)i / dtheta
  for (auto i = 0; i < N; ++i) {

    // Here L = frame after link i, K = frame before link i
    const auto& T_SC = cachedT_SC_;
    const auto& T_SL = cached_T[0][i + 1];
    const auto& T_KL = cached_T[i + 1][0];

    Eigen::Matrix<Scalar, 6, 6> Jleft_SC_SL;
    Eigen::Matrix<Scalar, 6, 6> Jright_SL_KL;
    Eigen::Matrix<Scalar, 6, 1> J_KL_theta;

    T_SC.composeLeftJacobian(T_SL, Jleft_SC_SL);
    T_SL.composeRightJacobian(T_KL, Jright_SL_KL);
    dhChain_[i].thetaMinimalJacobian(J_KL_theta);

    J_SC_theta.col(i) = Jleft_SC_SL * Jright_SL_KL * J_KL_theta;
  }
  return true;
}

template <int N>
template<typename Derived_jacobian>
bool GimbalTransformation<N>::oplusJacobian(
    const Eigen::MatrixBase<Derived_jacobian> & jacobian) const {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived_jacobian, 7, N);
  auto &J_SC_theta = const_cast<Eigen::MatrixBase<Derived_jacobian> &>(jacobian);
  using Scalar = typename Derived_jacobian::Scalar;

  // First, get the 6xN Jacobian wrt thetha
  Eigen::Matrix<Scalar, 6, N> J_min_theta;
  if (!this->oplusMinimalJacobian(J_min_theta)) {
    return false;
  }

  // Then convert to 7x6 using chain rule
  Eigen::Matrix<Scalar, 7, 6> J_SC_min;
  if (!cachedT_SC_.oplusJacobian(J_SC_min)) {
    return false;
  }

  J_SC_theta = J_SC_min * J_min_theta;
  return true;
}

template <int N>
template<typename Derived_jacobian>
bool GimbalTransformation<N>::liftJacobian(const Eigen::MatrixBase<Derived_jacobian> & jacobian) const {
  EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived_jacobian, N, 7);
  const_cast<Eigen::MatrixBase<Derived_jacobian>&>(jacobian).setIdentity();
  return false;
}

// This method resizes the cache. It should not be resized elsewhere
template <int N>
void GimbalTransformation<N>::initCache() {
  for (auto i = 0; i < N + 1; ++i) {
    const auto num = N + 1 - i;  // number of cached transforms up to E
    cached_T[i].resize(num);
  }
}

template <int N>
void GimbalTransformation<N>::updateCache() {
  // Update theta part of dh
  for (auto i = 0u; i < N; ++i) {
    dhChain_[i].theta = parameters_[i];
  }

  // Calculate adjacent transforms first
  cached_T[0][0] = T_SA_;
  for (auto link = 0; link < N; ++link) {
    cached_T[link + 1][0] = transformationFromDh(dhChain_[link]);
  }

  // Calculate transforms spaced by 2, then 3, and so on
  for (auto jump = 2; jump <= N + 1; ++jump) {
    for (auto start = 0; start <= N + 1 - jump; ++start) {
      const auto end = start + jump;
      cached_T[start][jump - 1] = cached_T[start][jump - 2] * cached_T[end - 1][0];
    }
  }

  // Calculate T_SC = T_SE * T_EC
  cachedT_SC_ = cached_T.front().back() * T_EC_;
}

}  // namespace kinematics
}  // namespace okvis
