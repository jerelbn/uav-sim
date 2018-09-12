#pragma once

#include <fstream>

#include "common_cpp/common.h"
#include "controller.h"
#include "sensors.h"
#include "ekf/ekf.h"
#include "environment.h"

namespace quadrotor
{


class Quadrotor
{

public:

  Quadrotor();
  Quadrotor(const std::string &filename);
  ~Quadrotor();

  void load(const std::string &filename);
  void run(const double &t, const environment::Environment& env);

  const vehicle::State& getTrueState() const { return x_; }

private:

  void f(const vehicle::State& x, const commandVector& u, vehicle::dxVector& dx, const Eigen::Vector3d& vw);
  void propagate(const double &dt, const commandVector& u, const Eigen::Vector3d& vw);
  void updateAccel(const commandVector& u, const Eigen::Vector3d& vw);
  void getOtherVehicles(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& all_vehicle_positions);
  void estimateTarget(const double &t, const Eigen::Vector3d &z);
  void log(const double &t);

  controller::Controller controller_;
  sensors::Sensors sensors_;
  ekf::EKF ekf_;

  vehicle::State x_, x2_, x3_, x4_;
  vehicle::dxVector dx_, k1_, k2_, k3_, k4_;
  commandVector u_;

  bool accurate_integration_, control_using_estimates_;
  double mass_, max_thrust_, t_prev_;
  Eigen::Matrix3d inertia_matrix_, inertia_inv_;
  Eigen::Matrix3d linear_drag_matrix_;
  Eigen::Vector3d linear_drag_;
  Eigen::Matrix3d angular_drag_matrix_;
  Eigen::Vector3d v_rel_;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > other_vehicle_positions;
  bool use_random_seed_;
  std::default_random_engine rng_;

  // Target estimation parameters
  bool use_target_truth_, bearing_only_;
  double kz_, tt_prev_;
  Eigen::Vector3d z_, vz_, target_noise_;
  std::normal_distribution<double> target_noise_dist_;
  Eigen::Matrix<double, 6, 1> xt_, ut_;
  Eigen::Matrix<double, 6, 6> Pt_, A_, Q_;
  Eigen::Matrix<double, 3, 3> R_;
  Eigen::Matrix<double, 3, 6> C_;

  std::string directory_;
  std::ofstream true_state_log_;
  std::ofstream command_log_;
  std::ofstream target_log_;

};


}
