#pragma once

#include <fstream>
#include <experimental/filesystem>

#include "common_cpp/common.h"
#include "controller.h"
#include "sensors.h"
#include "ekf/ekf.h"

namespace quadrotor
{


class Quadrotor
{

public:

  Quadrotor();
  Quadrotor(const std::string &filename);
  ~Quadrotor();

  void load(const std::string &filename);
  void run(const double &t, const double &dt, const Eigen::Vector3d& vw, const Eigen::MatrixXd &lm);

  const vehicle::State& get_true_state() const { return x_; }

private:

  void f(const vehicle::State& x, const commandVector& u, vehicle::dxVector& dx, const Eigen::Vector3d& vw);
  void propagate(const double &dt, const commandVector& u, const Eigen::Vector3d& vw);
  void updateAccel(const commandVector& u, const Eigen::Vector3d& vw);
  void log(const double &t);

  controller::Controller controller_;
  sensors::Sensors sensors_;
  ekf::EKF ekf_;

  vehicle::State x_, x2_, x3_, x4_;
  vehicle::dxVector dx_, k1_, k2_, k3_, k4_;
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
