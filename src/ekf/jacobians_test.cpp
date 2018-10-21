#include "ekf/ekf.h"
#include <random>
#include <chrono>

int main()
{
  // Numerical differentiation step size and error tolerance
  double eps = 1e-5;
  double tol = 1e-3;
  int iters = 1000;

  // Random variables
  unsigned seed = chrono::system_clock::now().time_since_epoch().count();
  default_random_engine rng(seed);
  normal_distribution<double> dist(0.0, 0.5);

  // Nominal state, gyro, accel, keyframe pose, and body-to-camera pose
  ekf::State x;
  Matrix<double,6,1> n;
  n.setOnes();
  Vector3d p_bc, p_ik, gyro, acc;
  common::randomNormalMatrix(x.p, dist, rng);
  x.q = common::Quaterniond(dist, rng);
  common::randomNormalMatrix(x.v, dist, rng);
  common::randomNormalMatrix(x.bg, dist, rng);
  common::randomNormalMatrix(x.ba, dist, rng);
  x.qk = common::Quaterniond(dist, rng);
  common::randomNormalMatrix(gyro, dist, rng);
  common::randomNormalMatrix(acc, dist, rng);
  common::Quaterniond q_bc(dist, rng);
  common::randomNormalMatrix(p_bc, dist, rng);
  common::randomNormalMatrix(p_ik, dist, rng);

  // Propagation Jacobians
  {
    ekf::dxMatrix F_numerical, F_analytical;
    ekf::gMatrix G_numerical, G_analytical;
    ekf::dxVector dxp, dxm;
    for (int j = 0; j < iters; ++j)
    {
      F_numerical.setZero();
      for (int i = 0; i < ekf::NUM_DOF; ++i)
      {
        ekf::dxVector delta;
        delta.setZero();
        delta(i) = eps;
        ekf::State xp = x + delta;
        ekf::State xm = x + -delta;
        ekf::EKF::f(xp, gyro, acc, dxp);
        ekf::EKF::f(xm, gyro, acc, dxm);
        F_numerical.col(i) = (dxp - dxm) / (2.0 * eps);
      }
      G_numerical.setZero();
      for (int i = 0; i < ekf::NUM_INPUTS; ++i)
      {
        ekf::uVector delta;
        delta.setZero();
        delta(i) = eps;
        ekf::uVector np = n + delta;
        ekf::uVector nm = n - delta;
        dxp.segment<3>(ekf::DPX) = x.q.inv().rot(x.v);
        dxp.segment<3>(ekf::DQX) = gyro - x.bg - np.segment<3>(ekf::UWX);
        dxp.segment<3>(ekf::DVX) = acc - x.ba - np.segment<3>(ekf::UAX) + common::gravity * x.q.rot(common::e3) -
                                   (gyro - x.bg - np.segment<3>(ekf::UWX)).cross(x.v);
        dxp.segment<3>(ekf::DGX).setZero();
        dxp.segment<3>(ekf::DAX).setZero();
        dxm.segment<3>(ekf::DPX) = x.q.inv().rot(x.v);
        dxm.segment<3>(ekf::DQX) = gyro - x.bg - nm.segment<3>(ekf::UWX);
        dxm.segment<3>(ekf::DVX) = acc - x.ba - nm.segment<3>(ekf::UAX) + common::gravity * x.q.rot(common::e3) -
                                   (gyro - x.bg - nm.segment<3>(ekf::UWX)).cross(x.v);
        dxm.segment<3>(ekf::DGX).setZero();
        dxm.segment<3>(ekf::DAX).setZero();
        G_numerical.col(i) = (dxp - dxm) / (2.0 * eps);
      }
      ekf::EKF::getFG(x, gyro, F_analytical, G_analytical);
      if(!common::TEST("Propagation state Jacobian", tol, F_numerical, F_analytical)) break;
      if(!common::TEST("Propagation noise Jacobian", tol, G_numerical, G_analytical)) break;
    }
  }

  // Measurement Jacobian
  {
    common::Quaterniond ht, hq;
    common::Quaterniond htp, hqp, htm, hqm;
    Matrix<double,5,ekf::NUM_DOF> H_numerical, H_analytical, H;
    for (int j = 0; j < iters; ++j)
    {
      H_numerical.setZero();
      for (int i = 0; i < ekf::NUM_DOF; ++i)
      {
        ekf::dxVector delta;
        delta.setZero();
        delta(i) = eps;
        ekf::State xp = x + delta;
        ekf::State xm = x + -delta;
        ekf::EKF::getH(xp, q_bc, p_bc, htp, hqp, H);
        ekf::EKF::getH(xm, q_bc, p_bc, htm, hqm, H);
        H_numerical.block<2,1>(0,i) = common::Quaterniond::log_uvec(htp, htm) / (2.0 * eps);
        H_numerical.block<3,1>(2,i) = (hqp - hqm) / (2.0 * eps);
      }
      ekf::EKF::getH(x, q_bc, p_bc, ht, hq, H_analytical);
      if (!common::TEST("Measurement Jacobian", tol, H_numerical, H_analytical)) break;
    }
  }

  // Keyframe Reset Jacobian
  {
    ekf::State x_newp, x_newm;
    ekf::dxMatrix N_numerical, N_analytical;
    for (int j = 0; j < iters; ++j)
    {
      N_numerical.setZero();
      for (int i = 0; i < ekf::NUM_DOF; ++i)
      {
        ekf::dxVector delta;
        delta.setZero();
        delta(i) = eps;
        ekf::State xp = x + delta;
        ekf::State xm = x + -delta;
        ekf::EKF::stateReset(xp, x_newp);
        ekf::EKF::stateReset(xm, x_newm);
        N_numerical.col(i) = (x_newp - x_newm) / (2.0 * eps);
      }
      ekf::EKF::getN(x, N_analytical);
      if(!common::TEST("Keyframe Reset Jacobian", tol, N_numerical, N_analytical)) break;
    }
  }
} // main
