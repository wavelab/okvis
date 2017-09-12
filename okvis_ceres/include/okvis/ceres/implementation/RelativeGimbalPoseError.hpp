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
 * @file implementation/RelativeGimbalPoseError.hpp
 * @brief Header implementation file for the RelativeGimbalPoseError class.
 * @author Leo Koppel
 */

#include "okvis/ceres/GimbalAnglesError.hpp"

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief ceres Namespace for ceres-related functionality implemented in okvis.
namespace ceres {

// Construct with measurement and variance.
template <int N>
RelativeGimbalPoseError<N>::RelativeGimbalPoseError(kinematics::GimbalTransformation<N> gimbalTransformation,
                                                    double translationVariance,
                                                    double rotationVariance)
    : gimbalTransformation_{std::move(gimbalTransformation)} {
  information_t information;
  information.setZero();
  information.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() * 1.0
      / translationVariance;
  information.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity() * 1.0
      / rotationVariance;

  setInformation(information);
}

// Set the information.
template <int N>
void RelativeGimbalPoseError<N>::setInformation(const information_t & information) {
  information_ = information;
  covariance_ = information.inverse();
  // perform the Cholesky decomposition on order to obtain the correct error weighting
  Eigen::LLT<information_t> lltOfInformation(information_);
  squareRootInformation_ = lltOfInformation.matrixL().transpose();
}

// This evaluates the error term and additionally computes the Jacobians.
template <int N>
bool RelativeGimbalPoseError<N>::Evaluate(double const* const * parameters,
                                 double* residuals, double** jacobians) const {
  return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, NULL);
}

// This evaluates the error term and additionally computes
// the Jacobians in the minimal internal representation.
template <int N>
bool RelativeGimbalPoseError<N>::EvaluateWithMinimalJacobians(
    double const* const * parameters, double* residuals, double** jacobians,
    double** jacobiansMinimal) const {

  // compute error
  const auto thetas0 = Eigen::Map<const Eigen::Matrix<double, N, 1>>{parameters[0]};
  const auto thetas1 = Eigen::Map<const Eigen::Matrix<double, N, 1>>{parameters[1]};
  auto T_SC_0 = gimbalTransformation_;
  auto T_SC_1 = gimbalTransformation_;
  T_SC_0.setParameters(thetas0);
  T_SC_1.setParameters(thetas1);

  // delta pose
  okvis::kinematics::Transformation dp = T_SC_0 * T_SC_1.inverse();
  // get the error
  Eigen::Matrix<double, 6, 1> error;
  const Eigen::Vector3d dtheta = 2 * dp.q().coeffs().head<3>();
  error.head<3>() = T_SC_0.r() - T_SC_1.r();
  error.tail<3>() = dtheta;

  // weigh it
  // @todo weigh wrt noise in angles ?
  Eigen::Map<Eigen::Matrix<double, 6, 1> > weighted_error(residuals);
  weighted_error = squareRootInformation_ * error;

  // compute Jacobian
  //@todo complete
  if (jacobians && jacobians[0]) {
    Eigen::Map<Eigen::Matrix<double, 6, N, Eigen::RowMajor> > J0(jacobians[0]);
    if (!T_SC_0.oplusMinimalJacobian(J0)) {
      return false;
    }
    if (jacobiansMinimal && jacobiansMinimal[0]) {
      // Put the same thing into minimal jacobian
      Eigen::Map<Eigen::Matrix<double, 6, N, Eigen::RowMajor> > J0_minimal(jacobiansMinimal[0]);
      J0_minimal = J0;
    }
  }

  return true;
}

}  // namespace ceres
}  // namespace okvis
