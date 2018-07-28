#include "ekf/ekf.h"
#include <random>
#include <chrono>

int main()
{
  // Numerical differentiation step size and error tolerance
  double eps = 1e-6;
  double tol = 1e-6;

  // Random variables
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine rng(seed);
  std::normal_distribution<double> dist(0.0, 0.5);

  // Nominal state
  ekf::State x;
  common::randomNormalMatrix(x.p, dist, rng);
  x.q = common::Quaternion(dist, rng);
  common::randomNormalMatrix(x.v, dist, rng);
  common::randomNormalMatrix(x.bg, dist, rng);
  common::randomNormalMatrix(x.ba, dist, rng);

  // Random input
  Eigen::Matrix<double, ekf::NUM_INPUTS, 1> u;
  common::randomNormalMatrix(u, dist, rng);

  // Create EKF object to call its functions
  ekf::EKF ekf;

  // Propagation Jacobian
  ekf::dxMatrix F_numerical, F_analytical;
  ekf::dxVector dxp, dxm;
  F_numerical.setZero();
  for (int i = 0; i < ekf::NUM_DOF; ++i)
  {
    ekf::dxVector delta;
    delta.setZero();
    delta(i) = eps;
    ekf::State xp = x + delta;
    ekf::State xm = x + -delta;
    ekf.f(dxp, xp, u);
    ekf.f(dxm, xm, u);
    F_numerical.col(i) = (dxp - dxm) / (2.0 * eps);
  }
  ekf.getF(F_analytical, x, u);
  common::TEST("Propagation Jacobian", tol, F_numerical, F_analytical);
}
