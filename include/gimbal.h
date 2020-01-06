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
  Gimbal(const std::string &filename, const bool& use_random_seed);
  ~Gimbal();

  void load(const std::string &filename, const bool& use_random_seed);
  void update(const double &t, const vehicle::Stated& aircraft_state, const sensors::Sensors &aircraft_sensors, environment::Environment& env);

  std::string name_;

private:

  void log(const double &t);
  void f(const vehicle::Stated& x, const Vector3d& u, const Vector3d& vw, vehicle::dxVector& dx);

  gmbl_ctrl_pid::Controller ctrl_;
  sensors::Sensors sensors_;
  gmbl_ekf::EKF ekf_;

  // State members description:
  //   position inertial to gimbal in inertial frame
  //   velocity of gimbal w.r.t. inertial in inertial frame
  //   acceleration of gimbal w.r.t. inertial in inertial frame
  //   rotation inertial to gimbal
  //   angular rate of gimbal w.r.t. inertial in gimbal frame
  //   angular acceleration of gimbal w.r.t. inertial in gimbal frame
  vehicle::Stated x_, aircraft_state_;
  vehicle::dxVector dx_;
  quat::Quatd q_bg_;
  sensors::Sensors aircraft_sensors_;

  bool accurate_integration_;
  double t_prev_, mass_, omega_f_;
  double max_roll_, max_pitch_;
  Matrix3d inertia_matrix_, inertia_inv_;
  Vector3d p_bg_; // translation body to gimbal in body frame
  Vector3d p_gcg_; // center of gravity offset
  Matrix3d Kf_; // friction drop-off gains
  Vector3d omega_aircraft_; // need this to calculate friction torque (inertial frame)
  Vector3d vw_; // dummy variable for wind used in integration
  Vector3d u_; // dummy variable for control torque

  common::Logger state_log_;
  common::Logger euler_log_;

};


} // namespace gimbal
