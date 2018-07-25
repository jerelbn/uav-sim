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

  // Derivative indices
  DVX = 6,
  DVY = 7,
  DVZ = 8,
  DWX = 9,
  DWY = 10,
  DWZ = 11,
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
