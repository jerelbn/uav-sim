#pragma once

#include <fstream>

#include "common_cpp/common.h"
#include "common_cpp/logger.h"
#include "vehicle.h"
#include "sensors.h"
#include "environment.h"

using namespace Eigen;


namespace gimbal
{


class Gimbal
{

public:

  Gimbal();
  Gimbal(const std::string &filename, const std::default_random_engine& rng);
  ~Gimbal();

  void load(const std::string &filename, const std::default_random_engine& rng);
  void propagate(const double &t, const Vector3d& u, const quat::Quatd& q_Ib);
  void updateAccelerations(const Vector3d& u);

  const std::string& name() const { return name_; }
  const vehicle::Stated& x() const { return x_; }

private:

  void log(const double &t);
  void f(const vehicle::Stated& x, const Vector3d& u, const Vector3d& vw, vehicle::dxVector& dx);
  void saturateRollPitch();
  Matrix3d R_euler_to_body(const double& roll, const double& pitch);

  std::string name_;

  // Descriptions of state members:
  //   p: position body to gimbal in body frame
  //   v: velocity of gimbal w.r.t. body (always zero)
  //   lin_accel: acceleration of gimbal w.r.t. body (always zero)
  //   q: rotation body to gimbal
  //   omega: angular rate of gimbal w.r.t. body in gimbal frame
  //   ang_accel: angular acceleration of gimbal w.r.t. body in gimbal frame
  vehicle::Stated x_;
  vehicle::dxVector dx_;

  double t_prev_, mass_, omega_f_;
  double max_roll_, max_pitch_;
  Matrix3d inertia_matrix_, inertia_inv_;
  Vector3d p_gcg_; // center of gravity offset
  Matrix3d Kf_; // friction drop-off gains
  Vector3d dummy_; // dummy variable for wind used in integration
  quat::Quatd q_Ib_; // aircraft attitude needed for C.G. torque calculation in dynamics

  common::Logger state_log_;
  common::Logger euler_log_;

};


} // namespace gimbal
