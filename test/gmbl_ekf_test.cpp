#include <iostream>
#include <gtest/gtest.h>
#include "gmbl_ekf.h"

using namespace std;
using namespace Eigen;


namespace gmbl_ekf
{


#define NUM_ITERS 10000
#define TOL 1e-4
#define EPS 1e-5
#define VAR 1.0
#define IDX dxMatrix::Identity()
#define IDU uMatrix::Identity()
#define RAND_INIT 1

#define EXPECT_MATRIX_CLOSE(m1, m2, tol)\
{\
  EXPECT_EQ(m1.rows(), m2.rows());\
  EXPECT_EQ(m1.cols(), m2.cols());\
  for (int i = 0; i < m1.rows(); ++i)\
  {\
    for (int j = 0; j < m1.cols(); ++j)\
    {\
      EXPECT_NEAR(m1(i,j), m2(i,j), tol);\
    }\
  }\
}


void f(const Stated &x, const uVector &u, const uVector &eta, dxVector &dx)
{
  dx.setZero();
  dx.segment<3>(DV) = x.sa * x.q.rota(u.segment<3>(UA) - eta.segment<3>(UA)) + common::gravity * common::e3;
  dx.segment<3>(DQ) = u.segment<3>(UG) - x.bg - eta.segment<3>(UG);
}


TEST(GimbalEKF, ModelJacobian)
{
  if (RAND_INIT)
    srand((unsigned)time(NULL));
  for (int iter = 0; iter < NUM_ITERS; ++iter)
  {
    // Create random state and input vectors
    Stated x(VAR * xVector::Random());
    Vector3d euler = VAR * Vector3d::Random();
    x.q = quat::Quatd(euler(0), euler(1), euler(2));
    uVector u = VAR * uVector::Random();

    // Numerical Jacobian
    dxMatrix Fn;
    for (int i = 0; i < Fn.cols(); ++i)
    {
      // Poke the state
      Stated xp = x + EPS * IDX.col(i);
      Stated xm = x + -EPS * IDX.col(i);

      // State derivatives
      dxVector xdotp, xdotm;
      EKF::f(xp, u, xdotp);
      EKF::f(xm, u, xdotm);

      // Move one of the attitude errors to the other's tangent space
      xdotm.segment<3>(DQ) = xm.q.rota(xdotm.segment<3>(DQ));
      xdotm.segment<3>(DQ) = xp.q.rotp(xdotm.segment<3>(DQ));

      // Derivative of xdot w.r.t. x
      Fn.col(i) = (xdotp - xdotm) / (2.0 * EPS);
    }

    // Analytical Jacobian
    dxMatrix Fa;
    EKF::getF(x, u, Fa);

    // Make sure matrices are close to the same
    EXPECT_MATRIX_CLOSE(Fn, Fa, TOL)
  }
}


TEST(GimbalEKF, InputNoiseJacobian)
{
  if (RAND_INIT)
    srand((unsigned)time(NULL));
  for (int iter = 0; iter < NUM_ITERS; ++iter)
  {
    // Create random state and input vectors
    Stated x(VAR * xVector::Random());
    Vector3d euler = VAR * Vector3d::Random();
    x.q = quat::Quatd(euler(0), euler(1), euler(2));
    uVector u = VAR * uVector::Random();
    uVector eta = VAR * uVector::Random();

    // Numerical Jacobian
    nuMatrix Gn;
    for (int i = 0; i < Gn.cols(); ++i)
    {
      // Poke the noise vector
      uVector etap = eta + EPS * IDU.col(i);
      uVector etam = eta + -EPS * IDU.col(i);

      // State derivatives
      dxVector xdotp, xdotm;
      f(x, u, etap, xdotp);
      f(x, u, etam, xdotm);

      // Derivative of xdot w.r.t. eta
      Gn.col(i) = (xdotp - xdotm) / (2.0 * EPS);
    }

    // Analytical Jacobian
    nuMatrix Ga;
    EKF::getG(x, u, Ga);

    // Make sure matrices are close to the same
    EXPECT_MATRIX_CLOSE(Gn, Ga, TOL)
  }
}


TEST(GimbalEKF, GPSJacobian)
{
  if (RAND_INIT)
    srand((unsigned)time(NULL));
  for (int iter = 0; iter < NUM_ITERS; ++iter)
  {
    // Create random state and input vectors
    Stated x(VAR * xVector::Random());
    Vector3d euler = VAR * Vector3d::Random();
    x.q = quat::Quatd(euler(0), euler(1), euler(2));

    Vector2d py = VAR * Vector2d::Random();
    quat::Quatd q_ecef2ned = quat::Quatd::from_euler(0, py(0), py(1));

    // Numerical Jacobian
    Matrix<double, 3, NUM_DOF> Hn;
    for (int i = 0; i < Hn.cols(); ++i)
    {
      // Poke the state
      Stated xp = x + EPS * IDX.col(i);
      Stated xm = x + -EPS * IDX.col(i);

      // GPS model vectors
      Vector3d hp, hm;
      EKF::h_gps(xp, q_ecef2ned, hp);
      EKF::h_gps(xm, q_ecef2ned, hm);

      // Derivative of model w.r.t. x
      Hn.col(i) = (hp - hm) / (2.0 * EPS);
    }

    // Analytical Jacobian
    Matrix<double, 3, NUM_DOF> Ha;
    EKF::getH_gps(x, q_ecef2ned.R(), Ha);

    // Make sure matrices are close to the same
    EXPECT_MATRIX_CLOSE(Hn, Ha, TOL)
  }
}


TEST(GimbalEKF, MagJacobian)
{
  if (RAND_INIT)
    srand((unsigned)time(NULL));
  for (int iter = 0; iter < NUM_ITERS; ++iter)
  {
    // Create random state and input vectors
    Stated x(VAR * xVector::Random());
    Vector3d euler = VAR * Vector3d::Random();
    x.q = quat::Quatd(euler(0), euler(1), euler(2));

    Vector2d py = M_PI/2 * Vector2d::Random();
    quat::Quatd q_ecef2ned = quat::Quatd::from_euler(0, py(0), py(1));
    Vector3d pos_ecef = VAR * 1e6 * Vector3d::Random();

    Vector3d mnp_ecef = (WGS84::lla2ecef(Vector3d(common::MNP_lat*M_PI/180.0, common::MNP_lon*M_PI/180.0, 0.0))).normalized();
    Vector3d axis = (common::e3.cross(mnp_ecef)).normalized();
    double angle = common::angDiffBetweenVecs(common::e3, mnp_ecef);
    quat::Quatd q_ecef2mnp = quat::Quatd::from_axis_angle(axis, angle);

    // Numerical Jacobian
    Matrix<double, 1, NUM_DOF> Hn;
    for (int i = 0; i < Hn.cols(); ++i)
    {
      // Poke the state
      Stated xp = x + EPS * IDX.col(i);
      Stated xm = x + -EPS * IDX.col(i);

      // GPS model vectors
      double hp, hm;
      EKF::h_mag(xp, pos_ecef, q_ecef2ned, mnp_ecef, q_ecef2mnp, hp);
      EKF::h_mag(xm, pos_ecef, q_ecef2ned, mnp_ecef, q_ecef2mnp, hm);

      // Derivative of model w.r.t. x
      Hn(i) = (hp - hm) / (2.0 * EPS);
    }

    // Analytical Jacobian
    Matrix<double, 1, NUM_DOF> Ha;
    EKF::getH_mag(x, pos_ecef, q_ecef2ned, mnp_ecef, q_ecef2mnp, Ha);

    // Make sure matrices are close to the same
    EXPECT_MATRIX_CLOSE(Hn, Ha, TOL)
  }
}


} // namespace gmbl_ekf