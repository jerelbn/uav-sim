#pragma once

#include <fstream>

#include "vehicle.h"
#include "common_cpp/common.h"
#include "common_cpp/logger.h"

using namespace Eigen;


namespace quadrotor
{


class Quadrotor
{

public:

  Quadrotor();
  Quadrotor(const std::string &filename, const std::default_random_engine& rng);
  ~Quadrotor();

  void load(const std::string &filename, const std::default_random_engine& rng);
  void propagate(const double &dt, const uVector& u, const Eigen::Vector3d& vw);
  void updateAccelerations(const uVector& u, const Eigen::Vector3d& vw);

  const std::string& name() const { return name_; }
  const vehicle::Stated& x() const { return x_; }


private:

  void f(const vehicle::Stated& x, const uVector& u,
         const Eigen::Vector3d& vw, vehicle::dxVector& dx);
  void log(const double &t);

  std::string name_;
  vehicle::Stated x_;
  vehicle::dxVector dx_;

  double mass_, max_thrust_, t_prev_;
  Eigen::Matrix3d inertia_matrix_, inertia_inv_;
  Eigen::Matrix3d linear_drag_matrix_;
  Eigen::Matrix3d angular_drag_matrix_;
  Eigen::Vector3d v_rel_;

  common::Logger state_log_;

};


} // namespace quadrotor
