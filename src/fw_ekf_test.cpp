#include "fw_ekf.h"

using namespace std;
using namespace ekf;


template<int R, int C>
Matrix<double,R,C> roundMat(const Matrix<double,R,C>& m, int decimals)
{
  Matrix<double,R,C> n;
  for (int i = 0; i < R; ++i)
    for (int j = 0; j < C; ++j)
      n(i,j) = common::round2dec(m(i,j), decimals);
  return n;
}


// Derivative of q + delta w.r.t. delta
Matrix<double,4,3> dqpd_dd(const quat::Quatd& q)
{
  Matrix<double,4,3> m;
  m << -q.x(), -q.y(), -q.z(),
        q.w(), -q.z(),  q.y(),
        q.z(),  q.w(), -q.x(),
       -q.y(),  q.x(),  q.w();
  m *= 0.5;
  return m;
}


// Derivative of dq w.r.t. q
Matrix<double,3,4> ddq_dq(const quat::Quatd& q)
{
  Matrix<double,3,4> m;
  m << -q.x(),  q.w(),  q.z(), -q.y(),
       -q.y(), -q.z(),  q.w(),  q.x(),
       -q.z(),  q.y(), -q.x(),  q.w();
  m *= 2.0;
  return m;
}


// Derivative of state + state_delta w.r.t. state_delta
Matrix<double,NUM_STATES,NUM_DOF> dxpd_dd(const Stated& x)
{
  Matrix<double,NUM_STATES,NUM_DOF> dx;
  dx.setZero();
  dx.block<3,3>(P,DP).setIdentity();
  dx.block<3,3>(V,DV).setIdentity();
  dx.block<4,3>(Q,DQ) = dqpd_dd(x.q);
  dx.block<3,3>(BA,DBA).setIdentity();
  dx.block<3,3>(BG,DBG).setIdentity();
  dx(BB,DBB) = 1.0;
  return dx;
}


// Derivative of dstate w.r.t. state
Matrix<double,NUM_DOF,NUM_STATES> ddx_dx(const Stated& x)
{
  Matrix<double,NUM_DOF,NUM_STATES> dx;
  dx.setZero();
  dx.block<3,3>(DP,P).setIdentity();
  dx.block<3,3>(DV,V).setIdentity();
  dx.block<3,4>(DQ,Q) = ddq_dq(x.q);
  dx.block<3,3>(DBA,BA).setIdentity();
  dx.block<3,3>(DBG,BG).setIdentity();
  dx(DBB,BB) = 1.0;
  return dx;
}


void f(const Stated &x, const uVector &u, xVector &dx)
{
  Vector3d acc = u.segment<3>(UA) - x.ba;
  Vector3d omega = u.segment<3>(UG) - x.bg;
  quat::Quatd q_omega;
  q_omega.arr_ =  Vector4d(0, 0.5 * omega(0), 0.5 * omega(1), 0.5 * omega(2));

  dx.setZero();
  dx.segment<3>(P) = x.q.rota(x.v);
  dx.segment<3>(V) = acc + common::gravity * x.q.rotp(common::e3) - omega.cross(x.v);
  dx.segment<4>(Q) = (x.q * q_omega).elements();
}


void f_tilde(const Stated &x_hat, const dxVector &x_tilde, const uVector& u, const double& dt, dxVector &x_tilde_dot)
{
  Stated x = x_hat + x_tilde;

  dxVector xdot, xdot_hat;
  EKF::f(x, u, xdot);
  EKF::f(x_hat, u, xdot_hat);

  Stated xp = x + xdot * dt;
  Stated xm = x + -xdot * dt;
  Stated x_hatp = x_hat + xdot_hat * dt;
  Stated x_hatm = x_hat + -xdot_hat * dt;

  dxVector x_tildep = xp - x_hatp;
  dxVector x_tildem = xm - x_hatm;

  x_tilde_dot = (x_tildep - x_tildem) / (2.0 * dt);
}


void getF(const Stated& x, const uVector& u, dxMatrix& F)
{
  F.setZero();
  F.block<3,3>(DP,DV) = x.q.inverse().R();
  F.block<3,3>(DP,DQ) = -x.q.inverse().R() * common::skew(x.v);
  F.block<3,3>(DV,DV) = -common::skew(u.segment<3>(UG) - x.bg);
  F.block<3,3>(DV,DQ) = common::skew(common::gravity * x.q.rotp(common::e3));
  F.block<3,3>(DV,DBA) = -common::I_3x3;
  F.block<3,3>(DV,DBG) = -common::skew(x.v);
  F.block<3,3>(DQ,DQ) = -common::skew(u.segment<3>(UG) - x.bg);
  F.block<3,3>(DQ,DBG) = -common::I_3x3;
}


void getG(const Stated& x, const uVector& u, nuMatrix& G)
{
  G.setZero();
  G.block<3,3>(DV,UA) = -common::I_3x3;
  G.block<3,3>(DV,UG) = -common::skew(x.v);
  G.block<3,3>(DQ,UG) = -common::I_3x3;
}



int main()
{
//  srand((unsigned)time(NULL));
  dxMatrix Idx = dxMatrix::Identity();
  Matrix<double,NUM_STATES,NUM_STATES> Ix = Matrix<double,NUM_STATES,NUM_STATES>::Identity();
  double eps = 1e-5;
  double var = 1e-3;

  Stated x_hat(var * xVector::Random());
  Vector3d euler = var * Vector3d::Random();
  x_hat.q = quat::Quatd(euler(0), euler(1), euler(2));
  uVector u = var * uVector::Random();
  dxVector x_tilde = var * dxVector::Random();
  Stated x = x_hat + x_tilde;

  // Error state Jacobian
  dxMatrix F_err;
  for (int i = 0; i < F_err.cols(); ++i)
  {
    // Poke the error state
    dxVector x_tildep = x_tilde + eps * Idx.col(i);
    dxVector x_tildem = x_tilde + -eps * Idx.col(i);

    // Error state derivatives
    dxVector x_tilde_dotp, x_tilde_dotm;
    f_tilde(x_hat, x_tildep, u, eps, x_tilde_dotp);
    f_tilde(x_hat, x_tildem, u, eps, x_tilde_dotm);

    // Derivative of x_tilde_dot w.r.t. x_tilde
    F_err.col(i) = (x_tilde_dotp - x_tilde_dotm) / (2.0 * eps);
  }

  // State Jacobian
  Matrix<double,NUM_STATES,NUM_STATES> F;
  for (int i = 0; i < F.cols(); ++i)
  {
    // Poke the error state
    Stated xp = Stated(x_hat.toEigen() + eps * Ix.col(i));
    Stated xm = Stated(x_hat.toEigen() + -eps * Ix.col(i));

    // State derivatives
    xVector xdotp, xdotm;
    f(xp, u, xdotp);
    f(xm, u, xdotm);

    // Derivative of x_tilde_dot w.r.t. x_tilde
    F.col(i) = (xdotp - xdotm) / (2.0 * eps);
  }
  dxMatrix F_min = ddx_dx(x) * F * dxpd_dd(x);

  // Analytical Jacobian
  dxMatrix Fa;
  getF(x_hat, u, Fa);

  cout << "\n\nF_err = \n" << F_err << "\n\n";
//  cout << "F_min = \n" << F_min << "\n\n";
  cout << "Fa = \n" << Fa << "\n\n";
//  cout << "Error = \n" << roundMat<NUM_DOF,NUM_DOF>(F_err - F_min, 1e6) << "\n\n";
//  cout << "\nError Numerical =  " << (F_err - F_min).norm() << "\n\n";
//  cout << "\nError Analytical = " << (F_err - Fa).norm() << "\n\n";

  return 0;
}
