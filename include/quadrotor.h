#pragma once

#include <fstream>

#include "common_cpp/common.h"
#include "quad_control.h"
#include "sensors.h"
#include "environment.h"
#include "quad_vi_ekf.h"

using namespace Eigen;


namespace quadrotor
{


class Quadrotor
{

public:

  Quadrotor();
  Quadrotor(const std::string &filename, const environment::Environment& env, const bool &use_random_seed, const int& id);
  ~Quadrotor();

  void load(const std::string &filename, const environment::Environment &env, const bool &use_random_seed);
  void run(const double &t, const environment::Environment& env);

  const vehicle::Stated& getState() const { return x_; }

  int id_;
  std::string name_;

private:

  void f(const vehicle::Stated& x, const uVector& u,
         const Eigen::Vector3d& vw, vehicle::dxVector& dx);
  void propagate(const double &dt, const uVector& u, const Eigen::Vector3d& vw);
  void updateAccels(const uVector& u, const Eigen::Vector3d& vw);
  void getOtherVehicles(const std::vector<Eigen::Vector3d,
                        Eigen::aligned_allocator<Eigen::Vector3d> >& all_vehicle_positions);
  void log(const double &t);

  Controller controller_;
  sensors::Sensors sensors_;
  qviekf::EKF ekf_;

  vehicle::Stated x_;
  vehicle::dxVector dx_;
  uVector u_;

  bool accurate_integration_, control_using_estimates_;
  double mass_, max_thrust_, t_prev_;
  Eigen::Matrix3d inertia_matrix_, inertia_inv_;
  Eigen::Matrix3d linear_drag_matrix_;
  Eigen::Matrix3d angular_drag_matrix_;
  Eigen::Vector3d v_rel_;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > other_vehicle_positions_;

  std::ofstream state_log_;
  std::ofstream euler_log_;

};


} // namespace quadrotor
