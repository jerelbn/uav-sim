#pragma once

#include <eigen3/Eigen/Eigen>


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

typedef Eigen::Matrix<double, NUM_STATES, 1> xVector;
typedef Eigen::Matrix<double, NUM_DOF, 1> dxVector;

struct State
{

  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Vector3d lin_accel;
  common::Quaternion<double> q;
  Eigen::Vector3d omega;
  Eigen::Vector3d ang_accel;

  State()
  {
    p.setZero();
    v.setZero();
    lin_accel.setZero();
    q = common::Quaternion<double>();
    omega.setZero();
    ang_accel.setZero();
  }

  State(const xVector &x0)
  {
    p = x0.segment<3>(PX);
    v = x0.segment<3>(VX);
    lin_accel = x0.segment<3>(AX);
    q = common::Quaternion<double>(Eigen::Vector4d(x0.segment<4>(QW)));
    omega = x0.segment<3>(WX);
    ang_accel = x0.segment<3>(GX);
  }

  State operator+(const dxVector &delta) const
  {
    State x;
    x.p = p + delta.segment<3>(DPX);
    x.v = v + delta.segment<3>(DVX);
    x.q = q + Eigen::Vector3d(delta.segment<3>(DQX));
    x.omega = omega + delta.segment<3>(DWX);
    return x;
  }

  void operator+=(const dxVector &delta)
  {
    *this = *this + delta;
  }

  Eigen::Matrix<double, NUM_STATES, 1> toEigen() const
  {
    Eigen::Matrix<double, NUM_STATES, 1> x;
    x << p, v, lin_accel, q.toEigen(), omega, ang_accel;
    return x;
  }

  Eigen::Matrix<double, NUM_DOF, 1> minimal() const
  {
    Eigen::Matrix<double, NUM_DOF, 1> x;
    x << p, v, lin_accel, common::Quaternion<double>::log(q), omega, ang_accel;
    return x;
  }

};

}


namespace quadrotor
{

typedef Eigen::Matrix<double, 4, 1> commandVector;

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
  Eigen::MatrixXd z;
  Eigen::MatrixXd R;
} measurement_t;


typedef std::vector<measurement_t, Eigen::aligned_allocator<measurement_t> > measurement_list_t;

}
