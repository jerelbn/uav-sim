#include "gmbl_ctrl_pid.h"


namespace gmbl_ctrl_pid
{


Controller::Controller() :
  prev_time_(0),
  initialized_(false)
{}


Controller::~Controller()
{
  command_log_.close();
}


void Controller::load(const std::string& filename, const std::string& name)
{
  // Load parameters
  common::get_yaml_node("controller_update_rate", filename, update_rate_);

  common::get_yaml_node("max_roll_torque", filename, max_roll_torque_);
  common::get_yaml_node("max_pitch_torque", filename, max_pitch_torque_);
  common::get_yaml_node("max_yaw_torque", filename, max_yaw_torque_);

  common::get_yaml_node("roll_rate_kp", filename, roll_rate_kp_);
  common::get_yaml_node("roll_rate_ki", filename, roll_rate_ki_);
  common::get_yaml_node("roll_rate_kd", filename, roll_rate_kd_);
  common::get_yaml_node("pitch_rate_kp", filename, pitch_rate_kp_);
  common::get_yaml_node("pitch_rate_ki", filename, pitch_rate_ki_);
  common::get_yaml_node("pitch_rate_kd", filename, pitch_rate_kd_);
  common::get_yaml_node("yaw_rate_kp", filename, yaw_rate_kp_);
  common::get_yaml_node("yaw_rate_ki", filename, yaw_rate_ki_);
  common::get_yaml_node("yaw_rate_kd", filename, yaw_rate_kd_);

  common::get_yaml_node("roll_kp", filename, roll_kp_);
  common::get_yaml_node("roll_ki", filename, roll_ki_);
  common::get_yaml_node("roll_kd", filename, roll_kd_);
  common::get_yaml_node("pitch_kp", filename, pitch_kp_);
  common::get_yaml_node("pitch_ki", filename, pitch_ki_);
  common::get_yaml_node("pitch_kd", filename, pitch_kd_);
  common::get_yaml_node("yaw_kp", filename, yaw_kp_);
  common::get_yaml_node("yaw_ki", filename, yaw_ki_);
  common::get_yaml_node("yaw_kd", filename, yaw_kd_);
  
  // Initialize controllers
  roll_rate_.init(roll_rate_kp_, roll_rate_ki_, roll_rate_kd_, max_roll_torque_, -max_roll_torque_, 0.5);
  pitch_rate_.init(pitch_rate_kp_, pitch_rate_ki_, pitch_rate_kd_, max_pitch_torque_, -max_pitch_torque_, 0.5);
  yaw_rate_.init(yaw_rate_kp_, yaw_rate_ki_, yaw_rate_kd_, max_yaw_torque_, -max_yaw_torque_, 0.5);
  
  roll_.init(roll_kp_, roll_ki_, roll_kd_, max_roll_torque_, -max_roll_torque_, 0.5);
  pitch_.init(pitch_kp_, pitch_ki_, pitch_kd_, max_pitch_torque_, -max_pitch_torque_, 0.5);
  yaw_.init(yaw_kp_, yaw_ki_, yaw_kd_, max_yaw_torque_, -max_yaw_torque_, 0.5);

  // Initialize loggers
  std::stringstream ss_c;
  ss_c << "/tmp/" << name << "_motor_command.log";
  command_log_.open(ss_c.str());
}


void Controller::computeControl(const double& t, const Eigen::Vector3d& omega, const quat::Quatd& q_bg, Eigen::Vector3d& u)
{
  double dt = common::round2dec(t - prev_time_, 6);
  if (t == 0 || dt >= 1.0 / update_rate_)
  {
    // Copy the current state and time
    prev_time_ = t;

    quat::Quatd q_g2_g = quat::Quatd(q_bg.roll(), 0, 0);
    quat::Quatd q_g1_g = quat::Quatd(q_bg.roll(), q_bg.pitch(), 0);

    double omega_roll = common::e1.dot(omega);
    double omega_pitch = common::e2.dot(q_g2_g.rota(omega));
    double omega_yaw = common::e3.dot(q_g1_g.rota(omega));

    double torque_roll_rate = roll_rate_.run(dt, omega_roll, 0, true);
    double torque_pitch_rate = pitch_rate_.run(dt, omega_pitch, 0, true);
    double torque_yaw_rate = yaw_rate_.run(dt, omega_yaw, 0, true);

    u(0) = torque_roll_rate;
    u(1) = torque_pitch_rate;
    u(2) = torque_yaw_rate;
  }

  // Log all data
  log(t, u);
}


void Controller::log(const double &t, const Eigen::Vector3d& u)
{
  // Write data to binary files and plot in another program
  command_log_.write((char*)&t, sizeof(double));
  command_log_.write((char*)u.data(), u.rows() * sizeof(double));
}


} // namespace gmbl_ctrl_pid
