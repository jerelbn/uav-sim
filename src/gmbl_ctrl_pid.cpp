#include "gmbl_ctrl_pid.h"


namespace gmbl_ctrl_pid
{


Controller::Controller() :
  t_prev_(0),
  initialized_(false)
{
  u_.setZero();
}


Controller::Controller(const std::string& filename, const std::string& name) :
  t_prev_(0),
  initialized_(false)
{
  u_.setZero();
  load(filename, name);
}


Controller::~Controller() {}


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

  euler_c_.setZero();

  // Initialize loggers
  std::stringstream ss_c, ss_e;
  ss_c << "/tmp/" << name << "_motor_command.log";
  ss_e << "/tmp/" << name << "_euler_command.log";
  motor_command_log_.open(ss_c.str());
  euler_command_log_.open(ss_e.str());
}


void Controller::computeControl(const double& t, const vehicle::Stated& x_Ib, const vehicle::Stated& x_bg, const Eigen::Vector3d& cmd_dir_I)
{
  double dt = common::round2dec(t - t_prev_, 6);
  if (t == 0 || dt >= 1.0 / update_rate_)
  {
    // Copy the current state and time
    t_prev_ = t;

    // Roll and pitch rotations
    quat::Quatd q_g2_g = quat::Quatd(x_bg.q.roll(), 0, 0);
    quat::Quatd q_g1_g = quat::Quatd(x_bg.q.roll(), x_bg.q.pitch(), 0);
    
    // Calculate commanded Euler angles
    double roll_c = 0;

    Eigen::Matrix3d P_xy = Eigen::Matrix3d::Identity() - common::e3 * common::e3.transpose();
    Eigen::Vector3d dir_xy = (P_xy * cmd_dir_I).normalized();
    double yaw_c = atan2(dir_xy(1), dir_xy(0));

    quat::Quatd q_yaw = quat::Quatd::from_euler(0,0,yaw_c);
    Eigen::Vector3d dir_p = q_yaw.rotp(cmd_dir_I);
    double pitch_c = atan2(-dir_p(2), dir_p(0));

    euler_c_ << roll_c, pitch_c, yaw_c;
    
    // Current states of gimbal axes
    quat::Quatd q_Ig = x_Ib.q * x_bg.q;
    Vector3d omega = x_Ib.omega + x_bg.omega;

    double roll = q_Ig.roll();
    double pitch = q_Ig.pitch();
    double yaw = q_Ig.yaw();
    
    double omega_roll = common::e1.dot(omega);
    double omega_pitch = common::e2.dot(q_g2_g.rota(omega));
    double omega_yaw = common::e3.dot(q_g1_g.rota(omega));

    // Add 180 deg to yaw and yaw_c to avoid wrapping issue in PID error calculation
    yaw += M_PI;
    yaw_c += M_PI;

    // Calculate torques via PID control
    double torque_roll = roll_.run(dt, roll, roll_c, true, omega_roll);
    double torque_pitch = pitch_.run(dt, pitch, pitch_c, true, omega_pitch);
    double torque_yaw = yaw_.run(dt, yaw, yaw_c, true, omega_yaw);

    double torque_roll_rate = roll_rate_.run(dt, omega_roll, 0, true);
    double torque_pitch_rate = pitch_rate_.run(dt, omega_pitch, 0, true);
    double torque_yaw_rate = yaw_rate_.run(dt, omega_yaw, 0, true);

    // Populate output
    u_(0) = torque_roll + torque_roll_rate;
    u_(1) = torque_pitch + torque_pitch_rate;
    u_(2) = torque_yaw + torque_yaw_rate;
  }

  // Log all data
  log(t);
}


void Controller::log(const double &t)
{
  // Write data to binary files and plot in another program
  motor_command_log_.log(t);
  motor_command_log_.logMatrix(u_);
  euler_command_log_.log(t);
  euler_command_log_.logMatrix(euler_c_);
}


} // namespace gmbl_ctrl_pid
