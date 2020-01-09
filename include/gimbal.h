#pragma once

#include <fstream>

#include "common_cpp/common.h"
#include "common_cpp/logger.h"
#include "vehicle.h"
#include "sensors.h"
#include "environment.h"
#include "gmbl_ctrl_pid.h"
#include "gmbl_ekf.h"

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
  void update(const double &t, const vehicle::Stated& aircraft_state, const sensors::Sensors &aircraft_sensors, environment::Environment& env);

  std::string name_;

private:

  void log(const double &t);
  void f(const vehicle::Stated& x, const Vector3d& u, const Vector3d& vw, vehicle::dxVector& dx);
  void saturateRollPitch(const Vector3d& omega_bI_b, const Vector3d& omegadot_bI_b);
  Matrix3d R_euler_to_body(const double& roll, const double& pitch);

  gmbl_ctrl_pid::Controller ctrl_;
  sensors::Sensors sensors_;
  gmbl_ekf::EKF ekf_;

  // Descriptions of state members:
  //   p: position body to gimbal in body frame
  //   v: velocity of gimbal w.r.t. body (always zero)
  //   lin_accel: acceleration of gimbal w.r.t. body (always zero)
  //   q: rotation body to gimbal
  //   omega: angular rate of gimbal w.r.t. body in gimbal frame
  //   ang_accel: angular acceleration of gimbal w.r.t. body in gimbal frame
  vehicle::Stated x_;
  vehicle::dxVector dx_;
  vehicle::Stated aircraft_state_;
  sensors::Sensors aircraft_sensors_;

  bool accurate_integration_;
  double t_prev_, mass_, omega_f_;
  double max_roll_, max_pitch_;
  Matrix3d inertia_matrix_, inertia_inv_;
  Vector3d p_gcg_; // center of gravity offset
  Matrix3d Kf_; // friction drop-off gains
  Vector3d vw_; // dummy variable for wind used in integration
  Vector3d u_; // dummy variable for control torque

  common::Logger state_log_;
  common::Logger euler_log_;

};


} // namespace gimbal
