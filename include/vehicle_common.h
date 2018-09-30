#pragma once

#include <eigen3/Eigen/Eigen>


namespace vehicle
{

typedef Eigen::Matrix<double, 16, 1> xVector;
typedef Eigen::Matrix<double, 12, 1> dxVector;

// State Indices
enum
{
  PX = 0,
  PY = 1,
  PZ = 2,
  QW = 3,
  QX = 4,
  QY = 5,
  QZ = 6,
  VX = 7,
  VY = 8,
  VZ = 9,
  WX = 10,
  WY = 11,
  WZ = 12,
  AX = 13,
  AY = 14,
  AZ = 15,
  NUM_STATES
};

// State Derivative Indices
enum
{
  DPX = 0,
  DPY = 1,
  DPZ = 2,
  DQX = 3,
  DQY = 4,
  DQZ = 5,
  DVX = 6,
  DVY = 7,
  DVZ = 8,
  DWX = 9,
  DWY = 10,
  DWZ = 11,
  DAX = 12,
  DAY = 13,
  DAZ = 14,
  NUM_DOF
};

struct State
{

  Eigen::Vector3d p;
  common::Quaternion q;
  Eigen::Vector3d v;
  Eigen::Vector3d omega;
  Eigen::Vector3d accel;

  State()
  {
    p.setZero();
    q = common::Quaternion();
    v.setZero();
    omega.setZero();
    accel.setZero();
  }

  State(const xVector &x0)
  {
    p = x0.segment<3>(PX);
    q = common::Quaternion(x0.segment<4>(QW));
    v = x0.segment<3>(VX);
    omega = x0.segment<3>(WX);
    accel = x0.segment<3>(AX);
  }

  State operator+(const dxVector &delta) const
  {
    State x;
    x.p = p + delta.segment<3>(DPX);
    x.q = q + delta.segment<3>(DQX);
    x.v = v + delta.segment<3>(DVX);
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
    x << p, q.toEigen(), v, omega, accel;
    return x;
  }

  Eigen::Matrix<double, NUM_DOF, 1> minimal() const
  {
    Eigen::Matrix<double, NUM_DOF, 1> x;
    x << p, common::Quaternion::log(q), v, omega, accel;
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
