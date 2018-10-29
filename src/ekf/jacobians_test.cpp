#include "ekf/ekf.h"
#include <random>
#include <chrono>

using namespace std;
using namespace Eigen;

int main()
{
  // Numerical differentiation step size and error tolerance
  double eps = 1e-5;
  double tol = 0.01;
  int iters = 1000;

  // Random variables
  unsigned seed = chrono::system_clock::now().time_since_epoch().count();
  default_random_engine rng(seed);
  normal_distribution<double> dist(0.0, 0.5);

  for (int j = 0; j < iters; ++j)
  {
    // Nominal state, gyro, accel, keyframe pose, and body-to-camera pose
    ekf::State<double> x;
    Vector2d zt_err;
    Vector3d p_bc, p, pk, t_dir, zq_err;
    ekf::uVector imu, noise;
    noise.setZero();
    common::randomNormalMatrix(p, dist, rng);
    common::randomNormalMatrix(pk, dist, rng);
    common::randomNormalMatrix(zt_err, dist, rng);
    common::randomNormalMatrix(zq_err, dist, rng);
    t_dir.normalize();
    x.t.setP(p);
    x.tk.setP(pk);
    x.t.setQ(common::Quaterniond(dist, rng));
    x.tk.setQ(common::Quaterniond(dist, rng));
    common::randomNormalMatrix(x.v, dist, rng);
    common::randomNormalMatrix(x.bg, dist, rng);
    common::randomNormalMatrix(x.ba, dist, rng);
    common::randomNormalMatrix(imu, dist, rng);
    common::Quaterniond q_bc(dist, rng);
    common::randomNormalMatrix(p_bc, dist, rng);

    common::Quaterniond ht, hq;
    ekf::rel_pose_model(x, q_bc, p_bc, ht, hq);

    ekf::State<double> x_reset = x;
    ekf::state_reset_model(x_reset);

    // Propagation Jacobian
    ekf::dxMatrix F_numerical, F_analytical;
    ekf::uMatrix G_numerical, G_analytical;
    ekf::dxVector dxp, dxm;
    F_numerical.setZero();
    for (int i = 0; i < ekf::NUM_DOF; ++i)
    {
      ekf::dxVector delta;
      delta.setZero();
      delta(i) = eps;
      ekf::State<double> xp = x + delta;
      ekf::State<double> xm = x + -delta;
      ekf::dynamics(xp, imu, dxp);
      ekf::dynamics(xm, imu, dxm);
      F_numerical.col(i) = (dxp - dxm) / (2.0 * eps);
    }
    G_numerical.setZero();
    for (int i = 0; i < ekf::NUM_INPUTS; ++i)
    {
      ekf::uVector delta;
      delta.setZero();
      delta(i) = eps;
      ekf::uVector np = noise + delta;
      ekf::uVector nm = noise + -delta;
      ekf::dynamics(x, imu, np, dxp);
      ekf::dynamics(x, imu, nm, dxm);
      G_numerical.col(i) = (dxp - dxm) / (2.0 * eps);
    }
    ekf::getFG(x.toEigen(), imu, F_analytical, G_analytical);
    if (!common::TEST("Propagation Jacobian", tol, F_numerical, F_analytical)) break;
    if (!common::TEST("Input Noise Jacobian", tol, G_numerical, G_analytical)) break;

    // Measurement Jacobian
    common::Quaterniond htp, htm, hqp, hqm;
    Matrix<double, 5, ekf::NUM_DOF> H_numerical, H_analytical;
    H_numerical.setZero();
    for (int i = 0; i < ekf::NUM_DOF; ++i)
    {
      ekf::dxVector delta;
      delta.setZero();
      delta(i) = eps;
      ekf::State<double> xp = x + delta;
      ekf::State<double> xm = x + -delta;
      ekf::rel_pose_model(xp, q_bc, p_bc, htp, hqp);
      ekf::rel_pose_model(xm, q_bc, p_bc, htm, hqm);
      H_numerical.block<2,1>(0,i) = common::Quaterniond::log_uvec(htp, htm) / (2.0 * eps);
      H_numerical.block<3,1>(2,i) = (hqp - hqm) / (2.0 * eps);
    }
    ekf::getH(x.toEigen(), ht, hq, q_bc, p_bc, H_analytical);
    if(!common::TEST("Measurement Jacobian", tol, H_numerical, H_analytical)) break;

    // Keyframe Reset Jacobian
    Matrix<double, ekf::NUM_DOF, ekf::NUM_DOF> N_numerical, N_analytical;
    N_numerical.setZero();
    for (int i = 0; i < ekf::NUM_DOF; ++i)
    {
      ekf::dxVector delta;
      delta.setZero();
      delta(i) = eps;
      ekf::State<double> xp = x + delta;
      ekf::State<double> xm = x + -delta;
      ekf::state_reset_model(xp);
      ekf::state_reset_model(xm);
      N_numerical.col(i) = (xp - xm) / (2.0 * eps);
    }
    ekf::getN(x_reset.toEigen(), N_analytical);
    if(!common::TEST("Keyframe Reset Jacobian", tol, N_numerical, N_analytical)) break;
  }
}
