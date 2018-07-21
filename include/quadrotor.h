#include "common_cpp/common.h"

namespace quadrotor
{


typedef Eigen::Matrix<double, 16, 1> xVector;
typedef Eigen::Matrix<double, 12, 1> dxVector;
typedef Eigen::Matrix<double, 4, 1> commandVector;

// State Indexes
enum {
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

  DVX = 6, // Derivative indices
  DVY = 7,
  DVZ = 8,
  DWX = 9,
  DWY = 10,
  DWZ = 11,
};

// Input indexes
enum {
  THRUST,
  TAUX,
  TAUY,
  TAUZ
};

class Quadrotor
{

public:

  Quadrotor();
  Quadrotor(std::string filename);

  void load(std::string filename);
  void run(const double t, const double dt, const Eigen::Vector3d& vw);


  const xVector& get_state() const { return x_; }

private:

  void f(const xVector& x, const commandVector& u, dxVector& dx, const Eigen::Vector3d& vw);
  void propagate(const double dt, const commandVector& u, const Eigen::Vector3d& vw);
  void updateAccel(const commandVector& u, const Eigen::Vector3d& vw);

  xVector x_, x2_, x3_, x4_;
  dxVector dx_, k1_, k2_, k3_, k4_;

  bool accurate_integration_;
  double mass_, max_thrust_;
  Eigen::Matrix3d inertia_matrix_, inertia_inv_;
  Eigen::Matrix3d linear_drag_matrix_;
  Eigen::Matrix3d angular_drag_matrix_;
  Eigen::Vector3d v_rel_;

};


}
