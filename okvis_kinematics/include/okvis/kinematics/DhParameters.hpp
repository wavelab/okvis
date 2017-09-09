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
 * @brief Header file for the DhParameters class.
 * @author Leo Koppel
 */

#ifndef INCLUDE_OKVIS_DHPARAMETERS_HPP_
#define INCLUDE_OKVIS_DHPARAMETERS_HPP_

#include "okvis/kinematics/Transformation.hpp"

/// \brief okvis Main namespace of this package.
namespace okvis {

/// \brief kinematics Namespace for kinematics functionality, i.e. transformations and stuff.
namespace kinematics {

/// \brief Set of Denavit–Hartenberg parameters for one frame
struct DhParameters
{
  double theta = 0;  ///< Joint rotation [rad] (value used for initialization)
  double d = 0;      ///< Link offset [m]
  double a = 0;      ///< Link length [m]
  double alpha = 0;  ///< Twist angle [rad]

  /// \brief set parameters to random values with arbitrary bounds
  void setRandom() {
    Eigen::Vector4d r = Eigen::Vector4d::Random();
    theta = r[0] * M_PI;
    alpha = r[1] * M_PI;
    a = r[2];
    d = r[3];
  }

  /// \brief jacobian of the tangent vector w.r.t. the joint angle theta
  /// The vector has the translation part first, followed by rotation
  /// \note output parameter is cast to non-const: see https://eigen.tuxfamily.org/dox/TopicFunctionTakingEigenTypes.html
  template <typename Derived_jacobian>
  bool thetaMinimalJacobian(const Eigen::MatrixBase<Derived_jacobian> & jacobian) const {
    EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived_jacobian, 6, 1);
    auto &jacobian_out = const_cast<Eigen::MatrixBase<Derived_jacobian> &>(jacobian);
    jacobian_out << -a*sin(theta), a*cos(theta), 0, 0, 0, 1;
    return true;
  }
};

/// \brief Compute homogeneous transformation from parameters
inline Transformation transformationFromDh(const DhParameters &dh) {
  const auto ctheta = cos(dh.theta);
  const auto stheta = sin(dh.theta);
  const auto calpha = cos(dh.alpha);
  const auto salpha = sin(dh.alpha);
  Eigen::Matrix4d T;
  T << ctheta, -stheta*calpha, stheta*salpha, dh.a*ctheta,
       stheta, ctheta*calpha, -ctheta*salpha, dh.a*stheta,
       0, salpha, calpha, dh.d,
       0, 0, 0, 1;
  return Transformation{T};
}

}  // namespace kinematics
}  // namespace okvis

#endif /* INCLUDE_OKVIS_DHPARAMETERS_HPP_ */
