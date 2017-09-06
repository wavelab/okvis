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
 * @file AnglesParameterBlock.hpp
 * @brief Header file for the AnglesParameterBlock class.
 * @author Leo Koppel
 */

#ifndef INCLUDE_OKVIS_CERES_ANGLESPARAMETERBLOCK_HPP_
#define INCLUDE_OKVIS_CERES_ANGLESPARAMETERBLOCK_HPP_

#include <Eigen/Core>
#include <okvis/ceres/ParameterBlockSized.hpp>
#include <okvis/Time.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief ceres Namespace for ceres-related functionality implemented in okvis.
namespace ceres {

/// @brief Wraps the parameter block for a series of angle estimates
template <int Dim>
class AnglesParameterBlock : public ParameterBlockSized<Dim, Dim, Eigen::Matrix<double, Dim, 1>> {
 public:

  /// \brief The estimate type (a vector, representing angles in radians).
  typedef Eigen::Matrix<double, Dim, 1> estimate_t;

  /// \brief The base class type.
  typedef ParameterBlockSized<Dim,Dim,estimate_t> base_t;

  /// \brief Default constructor (assumes not fixed).
  AnglesParameterBlock() : base_t::ParameterBlockSized() {
    this->setFixed(false);
  }

  /// \brief Constructor with estimate and time.
  /// @param[in] estimate The initial estimate angles.
  /// @param[in] id The (unique) ID of this block.
  /// @param[in] timestamp The timestamp of this state.
  AnglesParameterBlock(const estimate_t& estimate, uint64_t id, const okvis::Time& timestamp) {
    this->setEstimate(estimate);
    this->setId(id);
    this->setTimestamp(timestamp);
    this->setFixed(false);
  }

  /// \brief Trivial destructor.
  virtual ~AnglesParameterBlock() = default;

  // setters
  /// @brief Set estimate of this parameter block.
  /// @param[in] estimate The estimate to set this to.
  virtual void setEstimate(const estimate_t& estimate) {
    auto map = Eigen::Map<estimate_t>(this->parameters_);
    map = estimate;
  }

  /// @param[in] timestamp The timestamp of this state.
  void setTimestamp(const okvis::Time& timestamp){timestamp_=timestamp;}

  // getters
  /// @brief Get estimate.
  /// \return The estimate.
  virtual estimate_t estimate() const {
      auto map = Eigen::Map<const estimate_t>(this->parameters_);
      return map;
  }

  /// \brief Get the time.
  /// \return The timestamp of this state.
  okvis::Time timestamp() const {return timestamp_;}

  // minimal internal parameterization
  // x0_plus_Delta=Delta_Chi[+]x0
  /// \brief Generalization of the addition operation,
  ///        x_plus_delta = Plus(x, delta)
  ///        with the condition that Plus(x, 0) = x.
  /// @param[in] x0 Variable.
  /// @param[in] Delta_Chi Perturbation.
  /// @param[out] x0_plus_Delta Perturbed x.
  virtual void plus(const double* x0, const double* Delta_Chi, double* x0_plus_Delta) const {
    // Regular addition
    const auto x = Eigen::Map<const estimate_t>{x0};
    const auto delta = Eigen::Map<const estimate_t>{Delta_Chi};
    auto result = Eigen::Map<estimate_t>{x0_plus_Delta};

    result = x + delta;
  }

  /// \brief The jacobian of Plus(x, delta) w.r.t delta at delta = 0.
  /// @param[in] x0 Variable.
  /// @param[out] jacobian The Jacobian.
  virtual void plusJacobian(const double* x0, double* jacobian) const {
    // Regular addition
    auto result = Eigen::Map<Eigen::Matrix<double, Dim, Dim>>{jacobian};
    result.setIdentity();
  }

  // Delta_Chi=x0_plus_Delta[-]x0
  /// \brief Computes the minimal difference between a variable x and a perturbed variable x_plus_delta
  /// @param[in] x0 Variable.
  /// @param[in] x0_plus_Delta Perturbed variable.
  /// @param[out] Delta_Chi Minimal difference.
  /// \return True on success.
  virtual void minus(const double* x0, const double* x0_plus_Delta, double* Delta_Chi) const {
    // Regular subtraction
    const auto x = Eigen::Map<const estimate_t>{x0};
    const auto x_plus_delta = Eigen::Map<const estimate_t>{x0_plus_Delta};
    auto result = Eigen::Map<estimate_t>{Delta_Chi};

    result = x_plus_delta - x;
  }

  /// \brief Computes the Jacobian from minimal space to naively overparameterised space as used by ceres.
  /// @param[in] x0 Variable.
  /// @param[out] jacobian the Jacobian (dimension minDim x dim).
  /// \return True on success.
  virtual void liftJacobian(const double* x0, double* jacobian) const {
    // No overparametrization going on here
    auto result = Eigen::Map<Eigen::Matrix<double, Dim, Dim>>{jacobian};
    result.setIdentity();
  }

  /// @brief Return parameter block type as string
  virtual std::string typeInfo() const {return "AnglesParameterBlock";}

 private:
  okvis::Time timestamp_; ///< Time of this state.
};

}  // namespace ceres
}  // namespace okvis

#endif /* INCLUDE_OKVIS_CERES_ANGLESPARAMETERBLOCK_HPP_ */
