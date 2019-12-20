#include "gmbl_ctrl_pid.h"


namespace gmbl_ctrl_pid
{


Controller::Controller() :
  prev_time_(0),
  initialized_(false)
{}


Controller::~Controller()
{
  motor_command_log_.close();
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

  euler_c_.setZero();

  // Initialize loggers
  std::stringstream ss_c, ss_e;
  ss_c << "/tmp/" << name << "_motor_command.log";
  ss_e << "/tmp/" << name << "_euler_command.log";
  motor_command_log_.open(ss_c.str());
  euler_command_log_.open(ss_e.str());
}


void Controller::computeControl(const double& t, const quat::Quatd& q, const Eigen::Vector3d& cmd_dir_I, 
                                const Eigen::Vector3d& omega, const quat::Quatd& q_bg, Eigen::Vector3d& u)
{
  double dt = common::round2dec(t - prev_time_, 6);
  if (t == 0 || dt >= 1.0 / update_rate_)
  {
    // Copy the current state and time
    prev_time_ = t;

    // Roll and pitch rotations
    quat::Quatd q_g2_g = quat::Quatd(q_bg.roll(), 0, 0);
    quat::Quatd q_g1_g = quat::Quatd(q_bg.roll(), q_bg.pitch(), 0);
    
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
    double roll = q.roll();
    double pitch = q.pitch();
    double yaw = q.yaw();
    
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
    u(0) = torque_roll + torque_roll_rate;
    u(1) = torque_pitch + torque_pitch_rate;
    u(2) = torque_yaw + torque_yaw_rate;
  }

  // Log all data
  log(t, u);
}


void Controller::log(const double &t, const Eigen::Vector3d& u)
{
  // Write data to binary files and plot in another program
  motor_command_log_.write((char*)&t, sizeof(double));
  motor_command_log_.write((char*)u.data(), u.rows() * sizeof(double));
  euler_command_log_.write((char*)&t, sizeof(double));
  euler_command_log_.write((char*)euler_c_.data(), euler_c_.rows() * sizeof(double));
}


} // namespace gmbl_ctrl_pid
