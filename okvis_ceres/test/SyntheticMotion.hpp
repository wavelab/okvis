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
 *********************************************************************************/

#ifndef INCLUDE_OKVIS_TEST_MOTION_HELPERS_HPP_
#define INCLUDE_OKVIS_TEST_MOTION_HELPERS_HPP_

#include <okvis/ceres/ImuError.hpp>

inline double sinc_test(double x){
  if(fabs(x)>1e-10) {
    return sin(x)/x;
  }
  else{
    static const double c_2=1.0/6.0;
    static const double c_4=1.0/120.0;
    static const double c_6=1.0/5040.0;
    const double x_2 = x*x;
    const double x_4 = x_2*x_2;
    const double x_6 = x_2*x_2*x_2;
    return 1.0 - c_2*x_2 + c_4*x_4 - c_6*x_6;
  }
}

struct SyntheticMotion {
  okvis::ImuMeasurementDeque imuMeasurements;
  std::vector<okvis::SpeedAndBias> speedAndBiases;
  std::vector<okvis::kinematics::Transformation> T_WSs;
  std::vector<okvis::Time> times;
};

inline SyntheticMotion generateMotion(const okvis::ImuParameters &imuParameters) {
  SyntheticMotion out;

  // generate random motion
  const double w_omega_S_x = Eigen::internal::random(0.1,10.0); // circular frequency
  const double w_omega_S_y = Eigen::internal::random(0.1,10.0); // circular frequency
  const double w_omega_S_z = Eigen::internal::random(0.1,10.0); // circular frequency
  const double p_omega_S_x = Eigen::internal::random(0.0,M_PI); // phase
  const double p_omega_S_y = Eigen::internal::random(0.0,M_PI); // phase
  const double p_omega_S_z = Eigen::internal::random(0.0,M_PI); // phase
  const double m_omega_S_x = Eigen::internal::random(0.1,1.0); // magnitude
  const double m_omega_S_y = Eigen::internal::random(0.1,1.0); // magnitude
  const double m_omega_S_z = Eigen::internal::random(0.1,1.0); // magnitude
  const double w_a_W_x = Eigen::internal::random(0.1,10.0);
  const double w_a_W_y = Eigen::internal::random(0.1,10.0);
  const double w_a_W_z = Eigen::internal::random(0.1,10.0);
  const double p_a_W_x = Eigen::internal::random(0.1,M_PI);
  const double p_a_W_y = Eigen::internal::random(0.1,M_PI);
  const double p_a_W_z = Eigen::internal::random(0.1,M_PI);
  const double m_a_W_x = Eigen::internal::random(0.1,10.0);
  const double m_a_W_y = Eigen::internal::random(0.1,10.0);
  const double m_a_W_z = Eigen::internal::random(0.1,10.0);

  // generate randomized measurements - duration 10 seconds
  const double duration = 1.0;
  okvis::kinematics::Transformation T_WS;
  //T_WS.setRandom();

  // time increment
  const double dt=1.0/double(imuParameters.rate); // time discretization

  // states
  Eigen::Quaterniond q=T_WS.q();
  Eigen::Vector3d r=T_WS.r();
  okvis::SpeedAndBias speedAndBias;
  speedAndBias.setZero();
  Eigen::Vector3d v=speedAndBias.head<3>();

  for(size_t i=0; i<size_t(duration*imuParameters.rate); ++i){

    double time = double(i)/imuParameters.rate;


    Eigen::Vector3d omega_S(m_omega_S_x*sin(w_omega_S_x*time+p_omega_S_x),
                            m_omega_S_y*sin(w_omega_S_y*time+p_omega_S_y),
                            m_omega_S_z*sin(w_omega_S_z*time+p_omega_S_z));
    Eigen::Vector3d a_W(m_a_W_x*sin(w_a_W_x*time+p_a_W_x),
                        m_a_W_y*sin(w_a_W_y*time+p_a_W_y),
                        m_a_W_z*sin(w_a_W_z*time+p_a_W_z));

    //omega_S.setZero();
    //a_W.setZero();

    Eigen::Quaterniond dq;

    // propagate orientation
    const double theta_half = omega_S.norm()*dt*0.5;
    const double sinc_theta_half = sinc_test(theta_half);
    const double cos_theta_half = cos(theta_half);
    dq.vec()=sinc_theta_half*0.5*dt*omega_S;
    dq.w()=cos_theta_half;
    q = q * dq;

    // propagate speed
    v+=dt*a_W;

    // propagate position
    r+=dt*v;

    // T_WS
    T_WS = okvis::kinematics::Transformation(r,q);

    // speedAndBias - v only, obviously, since this is the Ground Truth
    speedAndBias.head<3>()=v;

    // generate measurements
    Eigen::Vector3d gyr = omega_S + imuParameters.sigma_g_c/sqrt(dt)
        *Eigen::Vector3d::Random();
    Eigen::Vector3d acc = T_WS.inverse().C()*(a_W+Eigen::Vector3d(0,0,imuParameters.g)) + imuParameters.sigma_a_c/sqrt(dt)
        *Eigen::Vector3d::Random();
    out.imuMeasurements.push_back(okvis::ImuMeasurement(okvis::Time(time),okvis::ImuSensorReadings(gyr,acc)));
    out.speedAndBiases.push_back(speedAndBias);
    out.T_WSs.push_back(T_WS);
    out.times.emplace_back(time);
  }

  return out;
}




#endif  // INCLUDE_OKVIS_TEST_MOTION_HELPERS_HPP_
