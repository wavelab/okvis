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
 * @brief Header file for the TransformationBase class.
 * @author Leo Koppel
 */

#ifndef INCLUDE_OKVIS_TRANSFORMATIONBASE_HPP_
#define INCLUDE_OKVIS_TRANSFORMATIONBASE_HPP_

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>


/// \brief okvis Main namespace of this package.
namespace okvis {

/// \brief kinematics Namespace for kinematics functionality, i.e. transformations and stuff.
namespace kinematics {

// Forward declare, used as return type
class Transformation;

/// \brief Base for homogeneous transformations with different internal parametrizations.
/// This relates a frame A and B: T_AB
class TransformationBase
{
 public:
  virtual ~TransformationBase() = default;

  /// \brief The underlying homogeneous transformation matrix.
  virtual Eigen::Matrix4d T() const = 0;

  /// \brief Returns the rotation matrix (cached).
  virtual const Eigen::Matrix3d & C() const = 0;

  /// \brief Returns the translation vector r_AB (represented in frame A).
  virtual const Eigen::Map<Eigen::Vector3d> & r() const = 0;

  /// \brief Returns the Quaternion q_AB (as an Eigen Quaternion).
  virtual const Eigen::Map<Eigen::Quaterniond> & q() const = 0;

  /// \brief Get the upper 3x4 part of the homogeneous transformation matrix T_AB.
  virtual Eigen::Matrix<double, 3, 4> T3x4() const = 0;

  /// \brief Get the parameters --- support for ceres.
  /// \warning USE WITH CARE!
  virtual const double* parameterPtr() const = 0;

  /// \brief Returns a copy of the transformation inverted.
  virtual Transformation inverse() const = 0;

  // operator* (group operator)
  /// \brief Multiplication with another transformation object.
  /// @param[in] rhs The right-hand side transformation for this to be multiplied with.
  virtual Transformation operator*(const Transformation & rhs) const = 0;

  /// \brief Transform a direction as v_A = C_AB*v_B (with rhs = hp_B)..
  /// \warning This only applies the rotation!
  /// @param[in] rhs The right-hand side direction for this to be multiplied with.
  virtual Eigen::Vector3d operator*(const Eigen::Vector3d & rhs) const = 0;

  /// \brief Transform a homogenous point as hp_B = T_AB*hp_B (with rhs = hp_B).
  /// @param[in] rhs The right-hand side direction for this to be multiplied with.
  virtual Eigen::Vector4d operator*(const Eigen::Vector4d & rhs) const = 0;
};

}  // namespace kinematics
}  // namespace okvis

#endif /* INCLUDE_OKVIS_TRANSFORMATIONBASE_HPP_ */
