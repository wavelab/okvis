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

// Construct with measurement and information matrix.
template <int N>
GimbalAnglesError<N>::GimbalAnglesError(
    const okvis::kinematics::GimbalTransformation<N> &measurement,
    const information_t & information)  {
  setMeasurement(measurement);
  setInformation(information);
}

// Construct with measurement and variance.
template <int N>
GimbalAnglesError<N>::GimbalAnglesError(
    const okvis::kinematics::GimbalTransformation<N> &measurement,
    double variance) {
  setMeasurement(measurement);
  information_t information;
  information = covariance_t::Identity() * 1.0 / variance;
  setInformation(information);
}

// Set the information.
template <int N>
void GimbalAnglesError<N>::setInformation(const information_t & information) {
  information_ = information;
  covariance_ = information.inverse();
  // perform the Cholesky decomposition on order to obtain the correct error weighting
  Eigen::LLT<information_t> lltOfInformation(information_);
  squareRootInformation_ = lltOfInformation.matrixL().transpose();
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
  weighted_error = squareRootInformation_ * error;

  // compute Jacobian
  if (jacobians && jacobians[0]) {
    Eigen::Map<Eigen::Matrix<double, 6, N, Eigen::RowMajor> > J0(jacobians[0]);
    if (!T_SC.oplusMinimalJacobian(J0)) {
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
