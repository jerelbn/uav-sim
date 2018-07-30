#include "ekf/ekf.h"
#include <random>
#include <chrono>

int main()
{
  // Numerical differentiation step size and error tolerance
  double eps = 1e-5;
  double tol = 1e-3;

  // Random variables
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine rng(seed);
  std::normal_distribution<double> dist(0.0, 0.5);

  // Nominal state, input, keyframe pose, and body-to-camera pose
  ekf::State x;
  Eigen::Vector3d p_bc, p_ik;
  Eigen::Matrix<double, ekf::NUM_INPUTS, 1> u;
  common::randomNormalMatrix(x.p, dist, rng);
  x.q = common::Quaternion(dist, rng);
  common::randomNormalMatrix(x.v, dist, rng);
  common::randomNormalMatrix(x.bg, dist, rng);
  common::randomNormalMatrix(x.ba, dist, rng);
  common::randomNormalMatrix(u, dist, rng);
  common::Quaternion q_bc(dist, rng);
  common::Quaternion q_ik(dist, rng);
  common::randomNormalMatrix(p_bc, dist, rng);
  common::randomNormalMatrix(p_ik, dist, rng);

  // Propagation Jacobian
  ekf::dxMatrix F_numerical, F_analytical;
  ekf::dxVector dxp, dxm;
  for (int j = 0; j < 1000; ++j)
  {
    F_numerical.setZero();
    for (int i = 0; i < ekf::NUM_DOF; ++i)
    {
      ekf::dxVector delta;
      delta.setZero();
      delta(i) = eps;
      ekf::State xp = x + delta;
      ekf::State xm = x + -delta;
      ekf::EKF::f(dxp, xp, u);
      ekf::EKF::f(dxm, xm, u);
      F_numerical.col(i) = (dxp - dxm) / (2.0 * eps);
    }
    ekf::EKF::getF(F_analytical, x, u);
    common::TEST("Propagation Jacobian", tol, F_numerical, F_analytical);
  }

  // Translation Measurement Jacobian
  common::Quaternion ht, hq; // placeholder
  Eigen::Matrix<double, 5, ekf::NUM_DOF> H; // placeholder
  common::Quaternion htp, htm;
  Eigen::Matrix<double, 2, ekf::NUM_DOF> Ht_numerical, Ht_analytical;
  for (int j = 0; j < 1000; ++j)
  {
    Ht_numerical.setZero();
    for (int i = 0; i < ekf::NUM_DOF; ++i)
    {
      ekf::dxVector delta;
      delta.setZero();
      delta(i) = eps;
      ekf::State xp = x + delta;
      ekf::State xm = x + -delta;
      ekf::EKF::imageH(htp, hq, H, xp, q_bc, p_bc, q_ik, p_ik);
      ekf::EKF::imageH(htm, hq, H, xm, q_bc, p_bc, q_ik, p_ik);
      Ht_numerical.col(i) = common::Quaternion::log_uvec(htp, htm) / (2.0 * eps);
    }
    ekf::EKF::imageH(ht, hq, H, x, q_bc, p_bc, q_ik, p_ik);
    Ht_analytical = H.topRows(2);
    common::TEST("Translation Measurement Jacobian", tol, Ht_numerical, Ht_analytical);
  }

  // Rotation Measurement Jacobian
  common::Quaternion hqp, hqm;
  Eigen::Matrix<double, 3, ekf::NUM_DOF> Hq_numerical, Hq_analytical;
  for (int j = 0; j < 1000; ++j)
  {
    Hq_numerical.setZero();
    for (int i = 0; i < ekf::NUM_DOF; ++i)
    {
      ekf::dxVector delta;
      delta.setZero();
      delta(i) = eps;
      ekf::State xp = x + delta;
      ekf::State xm = x + -delta;
      ekf::EKF::imageH(ht, hqp, H, xp, q_bc, p_bc, q_ik, p_ik);
      ekf::EKF::imageH(ht, hqm, H, xm, q_bc, p_bc, q_ik, p_ik);
      Hq_numerical.col(i) = (hqp - hqm) / (2.0 * eps);
    }
    ekf::EKF::imageH(ht, hq, H, x, q_bc, p_bc, q_ik, p_ik);
    Hq_analytical = H.bottomRows(3);
    common::TEST("Rotation Measurement Jacobian", tol, Hq_numerical, Hq_analytical);
  }
}
