#pragma once

#include <fstream>

#include "common_cpp/common.h"
#include "common_cpp/logger.h"
#include "vehicle.h"


namespace gmbl_ctrl_pid
{

class Controller
{

public:

  Controller();
  Controller(const std::string &filename, const std::string &name);
  ~Controller();

  void load(const std::string &filename, const std::string &name);
  void computeControl(const double& t, const vehicle::Stated& x_Ib, const vehicle::Stated& x_bg, const Eigen::Vector3d& cmd_dir_I);

  const Vector3d& u() const { return u_; }

private:

  int update_rate_;
  bool initialized_;
  double t_prev_;
  Eigen::Vector3d u_, euler_c_;

  common::Logger motor_command_log_;
  common::Logger euler_command_log_;

  double max_roll_torque_;
  double max_pitch_torque_;
  double max_yaw_torque_;

  double roll_rate_kp_, roll_rate_ki_, roll_rate_kd_;
  double pitch_rate_kp_, pitch_rate_ki_, pitch_rate_kd_;
  double yaw_rate_kp_, yaw_rate_ki_, yaw_rate_kd_;

  double roll_kp_, roll_ki_, roll_kd_;
  double pitch_kp_, pitch_ki_, pitch_kd_;
  double yaw_kp_, yaw_ki_, yaw_kd_;

  common::PID<double> roll_rate_;
  common::PID<double> pitch_rate_;
  common::PID<double> yaw_rate_;

  common::PID<double> roll_;
  common::PID<double> pitch_;
  common::PID<double> yaw_;

  void log(const double &t);

};

} // namespace gmbl_ctrl_pid
