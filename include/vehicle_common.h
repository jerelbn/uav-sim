#pragma once

#include <eigen3/Eigen/Eigen>
#include "geometry/quat.h"

using namespace Eigen;


namespace vehicle
{

// State Indices
enum
{
  PX, // Position
  PY,
  PZ,
  VX, // Velocity
  VY,
  VZ,
  AX, // Acceleration
  AY,
  AZ,
  QW, // Attitude
  QX,
  QY,
  QZ,
  WX, // Angular rate
  WY,
  WZ,
  GX, // Angular acceleration
  GY,
  GZ,
  NUM_STATES
};

// State Derivative Indices
enum
{
  DPX,
  DPY,
  DPZ,
  DVX,
  DVY,
  DVZ,
  DQX,
  DQY,
  DQZ,
  DWX,
  DWY,
  DWZ,
  NUM_DOF
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
    p = x0.segment<3>(PX);
    v = x0.segment<3>(VX);
    lin_accel = x0.segment<3>(AX);
    q = quat::Quatd(x0.segment<4>(QW));
    omega = x0.segment<3>(WX);
    ang_accel = x0.segment<3>(GX);
  }

  State operator+(const dxVector &delta) const
  {
    State x;
    x.p = p + delta.segment<3>(DPX);
    x.v = v + delta.segment<3>(DVX);
    x.q = q + delta.segment<3>(DQX);
    x.omega = omega + delta.segment<3>(DWX);
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

}


namespace quadrotor
{

typedef Matrix<double, 4, 1> commandVector;

// Input indices
enum
{
  THRUST,
  TAUX,
  TAUY,
  TAUZ
};

}


namespace sensors
{

typedef enum
{
  IMU,
  CAM
} measurement_type_t;

typedef struct
{
  measurement_type_t type;
  MatrixXd z;
  MatrixXd R;
} measurement_t;


typedef std::vector<measurement_t, aligned_allocator<measurement_t> > measurement_list_t;

}
