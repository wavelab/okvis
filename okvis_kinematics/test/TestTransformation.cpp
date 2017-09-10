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
 *  Created on: Dec 5, 2014
 *      Author: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

#include <okvis/kinematics/Transformation.hpp>
#include <okvis/kinematics/GimbalTransformation.hpp>
#include <iostream>
#include "gtest/gtest.h"

template <typename Transformation, int N>
class TransformationTestFixture {
 public:
  void testOperations() {
    Transformation T_AB;
    T_AB.setRandom();
    Transformation T_BC;
    T_BC.setRandom();

    // Test inverse
    EXPECT_LT(
        ((T_AB * T_AB.inverse()).T() - Eigen::Matrix4d::Identity()).norm(),
        1e-8);

    // Test composition
    EXPECT_LT(((T_AB * T_BC).T() - T_AB.T() * T_BC.T()).norm(), 1e-8);

  }

  void testOplus(const Transformation& T_AB) {
    // Test oplus
    const double dp = 1.0e-6;
    Eigen::Matrix<double, 7, N, (N > 1) ? Eigen::RowMajor : 0> jacobian_numDiff;
    for (size_t i = 0; i < N; ++i) {
      Transformation T_AB_p = T_AB;
      Transformation T_AB_m = T_AB;
      Eigen::Matrix<double, N, 1> dp_p;
      Eigen::Matrix<double, N, 1> dp_m;
      dp_p.setZero();
      dp_m.setZero();
      dp_p[i] = dp;
      dp_m[i] = -dp;
      T_AB_p.oplus(dp_p);
      T_AB_m.oplus(dp_m);
      /*jacobian_numDiff.block<7, 1>(0, i) = (T_AB_p.parameters()
          - T_AB_m.parameters()) / (2.0 * dp);*/
      jacobian_numDiff.template block<3, 1>(0, i) = (T_AB_p.r() - T_AB_m.r())
          / (2.0 * dp);
      jacobian_numDiff.template block<4, 1>(3, i) = (T_AB_p.q().coeffs()
          - T_AB_m.q().coeffs()) / (2.0 * dp);
    }
    Eigen::Matrix<double, 7, N, (N > 1) ? Eigen::RowMajor : 0> jacobian;
    T_AB.oplusJacobian(jacobian);
    //std::cout << jacobian << std::endl;
    //std::cout << jacobian_numDiff << std::endl;
    EXPECT_LT((jacobian - jacobian_numDiff).norm(), 1e-8);
    // also check lift Jacobian: dChi/dx*dx/dChi == 1
    Eigen::Matrix<double, N, 7, Eigen::RowMajor> lift_jacobian;
    if (T_AB.liftJacobian(lift_jacobian)) {
      // @todo clean up - currently ignore this test for GimbalTransformation
      EXPECT_LT(
          (lift_jacobian * jacobian - Eigen::Matrix<double, N, N>::Identity())
              .norm(), 1e-8);
    }
  }
};

TEST(Transformation, construction) {
  // Test construction
  okvis::kinematics::Transformation T_AB{};

  okvis::kinematics::Transformation T_AB_alternative(T_AB.T());
  EXPECT_TRUE((T_AB.T() - T_AB_alternative.T()).norm() < 1e-8);
  okvis::kinematics::Transformation T_AB_alternative2(T_AB.r(), T_AB.q());
  EXPECT_TRUE((T_AB.T() - T_AB_alternative2.T()).norm() < 1e-8);

  // Test =
  okvis::kinematics::Transformation T_AB_alternative3;
  T_AB_alternative3 = T_AB;
  EXPECT_TRUE((T_AB.T() - T_AB_alternative3.T()).norm() < 1e-8);

  // Test setters
  okvis::kinematics::Transformation T_AB_alternative4;
  T_AB_alternative4.set(T_AB.r(), T_AB.q());
  EXPECT_TRUE((T_AB.T() - T_AB_alternative4.T()).norm() < 1e-8);
  okvis::kinematics::Transformation T_AB_alternative5;
  T_AB_alternative5.set(T_AB.T());
  EXPECT_TRUE((T_AB.T() - T_AB_alternative5.T()).norm() < 1e-8);
}

TEST(Transformation, operations) {
  TransformationTestFixture<okvis::kinematics::Transformation, 6> fixture{};
  for (size_t i = 0; i < 100; ++i) {
    fixture.testOperations();
  }
}

TEST(Transformation, oplus) {
  TransformationTestFixture<okvis::kinematics::Transformation, 6> fixture{};
  for (size_t i = 0; i < 100; ++i) {
    okvis::kinematics::Transformation T_AB;
    T_AB.setRandom();
    fixture.testOplus(T_AB);
  }
}

TEST(Transformation, compositionJacobians) {
  okvis::kinematics::Transformation T_AB, T_BC;
  T_AB.setRandom();
  T_BC.setRandom();

  // Do composition
  const auto T_AC = T_AB * T_BC;
  Eigen::Matrix<double, 6, 6> jacobian_left, jacobian_right;
  T_AB.composeLeftJacobian(T_BC, jacobian_left);
  T_AB.composeRightJacobian(T_BC, jacobian_right);

  // Test jacobians against forward difference
  Eigen::Matrix<double, 6, 6> left_num_diff, right_num_diff;
  const double dp = 1.0e-6;
  for (size_t i = 0; i < 6; ++i) {
    auto T_AB_p = T_AB;
    auto T_BC_p = T_BC;

    Eigen::Matrix<double, 6, 1> dp_p;
    dp_p.setZero();
    dp_p[i] = dp;

    T_AB_p.oplus(dp_p);
    const auto res_left = T_AB_p * T_BC;
    left_num_diff.block<3, 1>(0, i) = (res_left.r() - T_AC.r()) / dp;
    // no minus operation, so implement minimal difference between quaternions
    left_num_diff.block<3, 1>(3, i) =
        okvis::kinematics::logMap(res_left.q() * T_AC.q().inverse()) / dp * 2;


    T_BC_p.oplus(dp_p);
    const auto res_right = T_AB * T_BC_p;
    right_num_diff.block<3, 1>(0, i) = (res_right.r() - T_AC.r()) / dp;
    right_num_diff.block<3, 1>(3, i) =
        okvis::kinematics::logMap(res_right.q() * T_AC.q().inverse()) / dp * 2;
  }

  EXPECT_LT((jacobian_left - left_num_diff).norm(), 1e-5);
  EXPECT_LT((jacobian_right - right_num_diff).norm(), 1e-5);
}

TEST(GimbalTransformation, constructDefault) {
  okvis::kinematics::GimbalTransformation<2> T_SC;
  EXPECT_TRUE(T_SC.T().isIdentity(1e-8));
}

TEST(GimbalTransformation, constructFromDh) {
  okvis::kinematics::Transformation T_SA, T_EC;
  okvis::kinematics::DhParameters dh1, dh2;
  T_SA.setRandom();
  T_EC.setRandom();
  dh1.setRandom();
  dh2.setRandom();
  okvis::kinematics::GimbalTransformation<2> T_SC{T_SA, T_EC, dh1, dh2};

  okvis::kinematics::Transformation expected =
      T_SA * transformationFromDh(dh1) * transformationFromDh(dh2) * T_EC;

  EXPECT_LT((T_SC.overallT().T() - expected.T()).norm(), 1e-8);
  EXPECT_LT((T_SC.T() - expected.T()).norm(), 1e-8);
}

TEST(GimbalTransformation, setParameters) {
  okvis::kinematics::DhParameters dh1, dh2;
  dh1.setRandom();
  dh2 = dh1;
  dh2.theta = M_PI;

  okvis::kinematics::GimbalTransformation<1> T_SC{{}, {}, dh1};

  auto expected = transformationFromDh(dh1);
  EXPECT_LT((T_SC.T() - expected.T()).norm(), 1e-8);

  auto params = Eigen::Matrix<double, 1, 1>{M_PI};
  T_SC.setParameters(params);
  EXPECT_EQ(params, T_SC.parameters());

  expected = transformationFromDh(dh2);
  EXPECT_LT((T_SC.T() - expected.T()).norm(), 1e-8);

}

TEST(GimbalTransformation, operations) {
  TransformationTestFixture<
      okvis::kinematics::GimbalTransformation<2>, 2> fixture{};
  for (size_t i = 0; i < 100; ++i) {
    fixture.testOperations();
  }
}

TEST(GimbalTransformation, oplus) {
  TransformationTestFixture<
      okvis::kinematics::GimbalTransformation<2>, 2> fixture{};
  okvis::kinematics::GimbalTransformation<2> T_SC;
  for (size_t i = 0; i < 100; ++i) {
    T_SC.setRandom();
    fixture.testOplus(T_SC);
  }
}

TEST(GimbalTransformation, oplusWithLongChain) {
  TransformationTestFixture<
      okvis::kinematics::GimbalTransformation<25>, 25> fixture{};
  okvis::kinematics::GimbalTransformation<25> T_SC;
  for (size_t i = 0; i < 5; ++i) {
    T_SC.setRandom();
    fixture.testOplus(T_SC);
  }
}

TEST(Transformation, testLogMap) {
  for (size_t i = 0; i < 100; ++i) {
    // Quaterniond::UnitRandom is only in newer Eigen, so use Transformation
    okvis::kinematics::Transformation T;
    T.setRandom();
    const auto q = T.q();

    const auto vec = okvis::kinematics::logMap(q);
    const auto q2 = okvis::kinematics::deltaQ(vec);

    EXPECT_TRUE(q2.isApprox(q));
  }
}
