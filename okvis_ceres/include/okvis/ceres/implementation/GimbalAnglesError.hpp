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
 * @file implementation/GimbalAnglesError.hpp
 * @brief Header implementation file for the GimbalAnglesError class.
 * @author Leo Koppel
 */

#include "okvis/ceres/GimbalAnglesError.hpp"

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief ceres Namespace for ceres-related functionality implemented in okvis.
namespace ceres {

// Construct with measurement and variance.
template <int N>
GimbalAnglesError<N>::GimbalAnglesError(
    const okvis::kinematics::GimbalTransformation<N> &measurement,
    double theta_variance) {
  setMeasurement(measurement);
  // @todo avoid config file mishaps
  if (theta_variance == 0) {
    LOG(FATAL) << "GimbalAnglesError() called with theta_variance == 0. Set some absolute translation variance.";
  }

  // Calculate information of measurement, given variance of the parameters
  Eigen::Matrix<double, 6, N> J;
  measurement.oplusMinimalJacobian(J);  // jacobian of pose measurement wrt parameters

  Eigen::Matrix<double, N, N> theta_cov = Eigen::Matrix<double, N, N>::Identity() * theta_variance;
  setCovariance(J * theta_cov * J.transpose());
}

// Set the information.
template <int N>
void GimbalAnglesError<N>::setCovariance(const covariance_t &covariance) {
  covariance_ = covariance;
  // perform the Cholesky decomposition on order to obtain the correct error weighting
  squareRootInformation_ = covariance_t{covariance.llt().matrixL()}.inverse();
}

// This evaluates the error term and additionally computes the Jacobians.
template <int N>
bool GimbalAnglesError<N>::Evaluate(double const* const * parameters, double* residuals,
                         double** jacobians) const {
    return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, nullptr);
}

// This evaluates the error term and additionally computes
// the Jacobians in the minimal internal representation.
template <int N>
bool GimbalAnglesError<N>::EvaluateWithMinimalJacobians(double const* const * parameters,
                                             double* residuals,
                                             double** jacobians,
                                             double** jacobiansMinimal) const {
  // Expect one parameter block of N parameters
  Eigen::Map<const Eigen::Matrix<double, 1, N>> thetas{parameters[0]};

  // compute error
  auto T_SC = measurement_;
  if(!T_SC.setParameters(thetas)) { return false; }
  // delta pose
  okvis::kinematics::Transformation dp = measurement_ * T_SC.inverse();
  // get the error
  Eigen::Matrix<double, 6, 1> error;
  const Eigen::Vector3d dtheta = 2 * dp.q().coeffs().head<3>();
  error.head<3>() = measurement_.r() - T_SC.r();
  error.tail<3>() = dtheta;

  // weigh it
  Eigen::Map<Eigen::Matrix<double, 6, 1> > weighted_error(residuals);
  // @todo! weigh error properly
  // currently covariance is singular and squareRootInformation is not usable
  // weighted_error = squareRootInformation_ * error;
  weighted_error = error;

  // compute Jacobian
  if (jacobians && jacobians[0]) {
    Eigen::Map<Eigen::Matrix<double, 6, N, Eigen::RowMajor> > J_error_theta(jacobians[0]);
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> J_error_T_SC;
    J_error_T_SC.setIdentity();
    J_error_T_SC *= -1.0;
    J_error_T_SC.block<3, 3>(3, 3) = -okvis::kinematics::plus(dp.q())
        .topLeftCorner<3, 3>();
    // @todo
//    J_error_T_SC = (squareRootInformation_ * J_error_T_SC).eval();

    Eigen::Matrix<double, 6, N, Eigen::RowMajor> J_T_SC_theta;
    if (!T_SC.oplusMinimalJacobian(J_T_SC_theta)) {
      return false;
    }

    J_error_theta = J_error_T_SC * J_T_SC_theta;

    if (jacobiansMinimal && jacobiansMinimal[0]) {
      // Put the same thing into minimal jacobian
      Eigen::Map<Eigen::Matrix<double, 6, N, Eigen::RowMajor> > J0_minimal(jacobiansMinimal[0]);
      J0_minimal = J_error_theta;
    }
  }
  return true;
}

}  // namespace ceres
}  // namespace okvis
