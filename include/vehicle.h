#pragma once

#include <eigen3/Eigen/Eigen>
#include "geometry/quat.h"

using namespace Eigen;


namespace vehicle
{

// State Indices
enum
{
  P = 0, // Position
  V = 3, // Velocity
  LA = 6, // Linear acceleration
  Q = 9, // Attitude
  W = 13, // Angular rate
  AA = 16, // Angular acceleration
  NUM_STATES = 19
};

// State Derivative Indices
enum
{
  DP = 0,
  DV = 3,
  DQ = 6,
  DW = 9,
  NUM_DOF = 12
};

typedef Matrix<double, NUM_STATES, 1> xVector;
typedef Matrix<double, NUM_DOF, 1> dxVector;

struct State
{

  Vector3d p;
  Vector3d v;
  Vector3d lin_accel;
  quat::Quatd q;
  Vector3d omega;
  Vector3d ang_accel;

  State()
  {
    p.setZero();
    v.setZero();
    lin_accel.setZero();
    omega.setZero();
    ang_accel.setZero();
  }

  State(const xVector &x0)
  {
    p = x0.segment<3>(P);
    v = x0.segment<3>(V);
    lin_accel = x0.segment<3>(LA);
    q = quat::Quatd(x0.segment<4>(Q));
    omega = x0.segment<3>(W);
    ang_accel = x0.segment<3>(AA);
  }

  State operator+(const dxVector &delta) const
  {
    State x;
    x.p = p + delta.segment<3>(DP);
    x.v = v + delta.segment<3>(DV);
    x.q = q + delta.segment<3>(DQ);
    x.omega = omega + delta.segment<3>(DW);
    return x;
  }

  void operator+=(const dxVector &delta)
  {
    *this = *this + delta;
  }

  Matrix<double, NUM_STATES, 1> toEigen() const
  {
    Matrix<double, NUM_STATES, 1> x;
    x << p, v, lin_accel, q.elements(), omega, ang_accel;
    return x;
  }

  Matrix<double, NUM_DOF, 1> minimal() const
  {
    Matrix<double, NUM_DOF, 1> x;
    x << p, v, lin_accel, quat::Quatd::log(q), omega, ang_accel;
    return x;
  }

};


// 4th order integration for truth of each vehicle
template<int U> // command vector size
void rk4(std::function<void(const State&, const Matrix<double,U,1>&, const Vector3d&, dxVector&)> f,
                            const double& dt, const State& x, const Matrix<double,U,1>& u,
                            const Vector3d& vw, vehicle::dxVector& dx)
{
  dxVector k1, k2, k3, k4;
  f(x, u, vw, k1);
  f(x + k1 * dt / 2, u, vw, k2);
  f(x + k2 * dt / 2, u, vw, k3);
  f(x + k3 * dt, u, vw, k4);
  dx = (k1 + 2 * k2 + 2 * k3 + k4) * dt / 6.0;
}

} // namespace vehicle


// Command vectors to be shared with controllers
namespace quadrotor
{

enum
{
  THRUST,
  TAUX,
  TAUY,
  TAUZ,
  COMMAND_SIZE
};
typedef Matrix<double, COMMAND_SIZE, 1> uVector;

} // namespace quadrotor
