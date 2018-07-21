#pragma once

#include <fstream>
#include <experimental/filesystem>

#include "common_cpp/common.h"
#include "controller.h"

namespace quadrotor
{


class Quadrotor
{

public:

  Quadrotor();
  Quadrotor(const std::string filename);
  ~Quadrotor();

  void load(std::string filename);
  void run(const double t, const double dt, const Eigen::Vector3d& vw);


  const xVector& get_state() const { return x_; }

private:

  void f(const xVector& x, const commandVector& u, dxVector& dx, const Eigen::Vector3d& vw);
  void propagate(const double dt, const commandVector& u, const Eigen::Vector3d& vw);
  void updateAccel(const commandVector& u, const Eigen::Vector3d& vw);

  void log(const double t);

  controller::Controller controller_;

  xVector x_, x2_, x3_, x4_;
  dxVector dx_, k1_, k2_, k3_, k4_;
  commandVector u_;

  bool accurate_integration_;
  double mass_, max_thrust_;
  Eigen::Matrix3d inertia_matrix_, inertia_inv_;
  Eigen::Matrix3d linear_drag_matrix_;
  Eigen::Matrix3d angular_drag_matrix_;
  Eigen::Vector3d v_rel_;

  std::string directory_;
  std::ofstream true_state_log_;
  std::ofstream command_log_;

};


}
