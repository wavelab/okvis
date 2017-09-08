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
 * @file GimbalAnglesError.hpp
 * @brief Header file for the GimbalAnglesError class.
 * @author Leo Koppel
 */

#ifndef INCLUDE_OKVIS_CERES_GIMBALANGLESERROR_HPP_
#define INCLUDE_OKVIS_CERES_GIMBALANGLESERROR_HPP_

#include <okvis/ceres/PoseError.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {
/// \brief ceres Namespace for ceres-related functionality implemented in okvis.
namespace ceres {

/// \brief Absolute error of a pose based on joint angles in a kinematic chain
template <int Dim>
class GimbalAnglesError : public ::ceres::SizedCostFunction<6 /* number of residuals */,
    Dim /* size of first parameter */>, public ErrorInterface {
 public:

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  OKVIS_DEFINE_EXCEPTION(Exception,std::runtime_error)

  /// \brief The base class type.
  typedef ::ceres::SizedCostFunction<6, Dim> base_t;

  /// \brief The number of residuals (6).
  static const int kNumResiduals = 6;

  /// \brief The information matrix type (6x6).
  typedef Eigen::Matrix<double, Dim, Dim> information_t;

  /// \brief The covariance matrix type (same as information).
  typedef Eigen::Matrix<double, Dim, Dim> covariance_t;

  /// \brief Default constructor.
  GimbalAnglesError() = default;

  /// \brief Construct with measurement and information matrix.
  /// @param[in] measurement The measurement.
  /// @param[in] information The information (weight) matrix.
  GimbalAnglesError(const okvis::kinematics::Transformation & measurement,
            const information_t & information) {
    setMeasurement(measurement);
    setInformation(information);
  }

  /// \brief Construct with measurement and variance.
  /// @param[in] measurement The measurement.
  /// @param[in] translationVariance The translation variance.
  /// @param[in] rotationVariance The rotation variance.
  GimbalAnglesError(const okvis::kinematics::Transformation & measurement,
            double variance) {
    setMeasurement(measurement);

    information_t information;
    information = covariance_t::Identity() * 1.0 / variance;
    setInformation(information);
  }

  /// \brief Trivial destructor.
  virtual ~GimbalAnglesError() = default;

  /// \name Setters
  /// \{

  /// \brief Set the measurement.
  /// @param[in] measurement The measurement.
  void setMeasurement(const okvis::kinematics::Transformation & measurement) {
    measurement_ = measurement;
  }

  /// \brief Set the information.
  /// @param[in] information The information (weight) matrix.
  void setInformation(const information_t & information) {
    information_ = information;
    covariance_ = information.inverse();
    // perform the Cholesky decomposition on order to obtain the correct error weighting
    Eigen::LLT<information_t> lltOfInformation(information_);
    squareRootInformation_ = lltOfInformation.matrixL().transpose();
  }

  /// \}
  /// \name Getters
  /// \{

  /// \brief Get the measurement.
  /// \return The measurement vector.
  const okvis::kinematics::Transformation& measurement() const {
    return measurement_;
  }

  /// \brief Get the information matrix.
  /// \return The information (weight) matrix.
  const information_t& information() const {
    return information_;
  }

  /// \brief Get the covariance matrix.
  /// \return The inverse information (covariance) matrix.
  const information_t& covariance() const {
    return covariance_;
  }

  /// \}

  // error term and Jacobian implementation
  /**
    * @brief This evaluates the error term and additionally computes the Jacobians.
    * @param parameters Pointer to the parameters (see ceres)
    * @param residuals Pointer to the residual vector (see ceres)
    * @param jacobians Pointer to the Jacobians (see ceres)
    * @return success of th evaluation.
    */
  virtual bool Evaluate(double const* const * parameters, double* residuals,
                        double** jacobians) const {
    return EvaluateWithMinimalJacobians(parameters, residuals, jacobians, NULL);
  }

  /**
   * @brief This evaluates the error term and additionally computes
   *        the Jacobians in the minimal internal representation.
   * @param parameters Pointer to the parameters (see ceres)
   * @param residuals Pointer to the residual vector (see ceres)
   * @param jacobians Pointer to the Jacobians (see ceres)
   * @param jacobiansMinimal Pointer to the minimal Jacobians (equivalent to jacobians).
   * @return Success of the evaluation.
   */
  virtual bool EvaluateWithMinimalJacobians(double const* const * parameters,
                                            double* residuals,
                                            double** jacobians,
                                            double** jacobiansMinimal) const {
    return true;
  }

  // sizes
  /// \brief Residual dimension.
  size_t residualDim() const {
    return kNumResiduals;
  }

  /// \brief Number of parameter blocks.
  size_t parameterBlocks() const {
    return base_t::parameter_block_sizes().size();
  }

  /// \brief Dimension of an individual parameter block.
  size_t parameterBlockDim(size_t parameterBlockId) const {
    return base_t::parameter_block_sizes().at(parameterBlockId);
  }

  /// @brief Return parameter block type as string
  virtual std::string typeInfo() const {
    return "GimbalAnglesError";
  }

 protected:

  // the measurement
  okvis::kinematics::Transformation measurement_; ///< The pose measurement.

  // weighting related
  information_t information_; ///< The 6x6 information matrix.
  information_t squareRootInformation_; ///< The 6x6 square root information matrix.
  covariance_t covariance_; ///< The 6x6 covariance matrix.

};

}  // namespace ceres
}  // namespace okvis

#endif /* INCLUDE_OKVIS_CERES_GIMBALANGLESERROR_HPP_ */
