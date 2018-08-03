#include "controller.h"


namespace controller
{


Controller::Controller() :
  prev_time_(0),
  initialized_(false)
{
  dhat_.setZero();
}


Controller::Controller(const std::string filename) :
  prev_time_(0),
  initialized_(false)
{
  dhat_.setZero();
  load(filename);
}


void Controller::load(const std::string filename)
{
  std::vector<double> loaded_wps;
  if (common::get_yaml_node("waypoints", filename, loaded_wps))
  {
    int num_waypoints = std::floor(loaded_wps.size()/4.0);
    waypoints_ = Eigen::Map<Eigen::MatrixXd>(loaded_wps.data(), 4, num_waypoints);
    current_waypoint_id_ = 0;
  }

  Eigen::Vector3d Kp_diag, Kd_diag, Kv_diag;
  if (common::get_yaml_eigen("Kp", filename, Kp_diag))
    K_p_ = Kp_diag.asDiagonal();
  if (common::get_yaml_eigen("Kd", filename, Kd_diag))
    K_d_ = Kd_diag.asDiagonal();
  if (common::get_yaml_eigen("Kv", filename, Kv_diag))
    K_v_ = Kv_diag.asDiagonal();

  common::get_yaml_node("throttle_eq", filename, throttle_eq_);
  common::get_yaml_node("mass", filename, mass_);
  common::get_yaml_node("max_thrust", filename, max_thrust_);
  common::get_yaml_node("waypoint_threshold", filename, waypoint_threshold_);
  common::get_yaml_node("waypoint_velocity_threshold", filename, waypoint_velocity_threshold_);

  common::get_yaml_node("roll_kp", filename, roll_.kp_);
  common::get_yaml_node("roll_ki", filename, roll_.ki_);
  common::get_yaml_node("roll_kd", filename, roll_.kd_);
  common::get_yaml_node("pitch_kp", filename, pitch_.kp_);
  common::get_yaml_node("pitch_ki", filename, pitch_.ki_);
  common::get_yaml_node("pitch_kd", filename, pitch_.kd_);
  common::get_yaml_node("yaw_rate_kp", filename, yaw_rate_.kp_);
  common::get_yaml_node("yaw_rate_ki", filename, yaw_rate_.ki_);
  common::get_yaml_node("yaw_rate_kd", filename, yaw_rate_.kd_);
  common::get_yaml_node("max_tau_x", filename, roll_.max_);
  common::get_yaml_node("max_tau_y", filename, pitch_.max_);
  common::get_yaml_node("max_tau_z", filename, yaw_rate_.max_);
  common::get_yaml_node("max_roll", filename, max_.roll);
  common::get_yaml_node("max_pitch", filename, max_.pitch);
  common::get_yaml_node("max_yaw_rate", filename, max_.yaw_rate);
  common::get_yaml_node("max_throttle", filename, max_.throttle);
  common::get_yaml_node("max_vel", filename, max_.vel);

}


void Controller::computeControl(const vehicle::State &x, const double t, quadrotor::commandVector& u)
{
  // Copy the current state
  Eigen::Vector3d euler = x.q.euler();
  xhat_.pn = x.p(0);
  xhat_.pe = x.p(1);
  xhat_.pd = x.p(2);
  xhat_.u = x.v(0);
  xhat_.v = x.v(1);
  xhat_.w = x.v(2);
  xhat_.phi = euler(0);
  xhat_.theta = euler(1);
  xhat_.psi = euler(2);
  xhat_.p = x.omega(0);
  xhat_.q = x.omega(1);
  xhat_.r = x.omega(2);

  xc_.t = t;

  // Refresh the waypoint
  updateWaypointManager();

  double dt = t - prev_time_;
  prev_time_ = t;

  if (dt < 0.0001)
  {
    u.setZero();
    return;
  }

  // get data that applies to both position and velocity control
  Eigen::Matrix3d R_v_to_v1 = common::R_v_to_v1(euler(2)); // rotation from vehicle to vehicle-1 frame
  Eigen::Matrix3d R_v1_to_b = common::R_v_to_b(euler(0), euler(1), 0.0); // rotation from vehicle-1 to body frame
  static Eigen::Vector3d k = common::e3;

  Eigen::Vector3d phat(xhat_.pn, xhat_.pe, xhat_.pd); // position estimate
  Eigen::Vector3d pc(xc_.pn, xc_.pe, xc_.pd); // position command
  Eigen::Vector3d vc = R_v_to_v1 * K_p_ * (pc-phat); // velocity command

  // store velocity command
  xc_.u = vc(0);
  xc_.v = vc(1);
  xc_.w = vc(2);

  // get yaw rate direction and allow it to saturate
  xc_.r = xc_.psi - xhat_.psi;

  // angle wrapping
  common::wrapAngle(xc_.r, M_PI);

  // get velocity command and enforce max velocity
  double vmag = vc.norm();
  if (vmag > max_.vel)
    vc = vc*max_.vel/vmag;

  Eigen::Vector3d vhat_b(xhat_.u,xhat_.v,xhat_.w); // body velocity estimate
  Eigen::Vector3d vhat = R_v1_to_b.transpose()*vhat_b; // vehicle-1 velocity estimate
  dhat_ = dhat_ - K_d_*(vc-vhat)*dt; // update disturbance estimate
  Eigen::Vector3d k_tilde = throttle_eq_ * (k - (1.0 / common::gravity) * (K_v_ * (vc - vhat) - dhat_));

  // pack up throttle command
  xc_.throttle = k.transpose() * R_v1_to_b * k_tilde;

  Eigen::Vector3d kd = (1.0 / xc_.throttle) * k_tilde; // desired body z direction
  kd = kd / kd.norm(); // need direction only
  double tilt_angle = acos(k.transpose() * kd); // desired tilt

  // get shortest rotation to desired tilt
  common::Quaternion q_c;
  if (tilt_angle > 1e-6)
  {
    Eigen::Vector3d k_cross_kd = k.cross(kd);
    q_c = common::Quaternion::exp(tilt_angle * k_cross_kd / k_cross_kd.norm());
  }

  // pack up attitude commands
  xc_.phi = q_c.roll();
  xc_.theta = q_c.pitch();

  // Create Roll and Pitch Command
  xc_.throttle = common::saturate(xc_.throttle, max_.throttle, 0.0);
  xc_.phi = common::saturate(xc_.phi, max_.roll, -max_.roll);
  xc_.theta = common::saturate(xc_.theta, max_.pitch, -max_.pitch);
  xc_.r = common::saturate(xc_.r, max_.yaw_rate, -max_.yaw_rate);

  // Calculate the Final Output Torques using PID
  u(quadrotor::THRUST) = xc_.throttle;
  u(quadrotor::TAUX) = roll_.run(dt, xhat_.phi, xc_.phi, false, xhat_.p);
  u(quadrotor::TAUY) = pitch_.run(dt, xhat_.theta, xc_.theta, false, xhat_.q);
  u(quadrotor::TAUZ) = yaw_rate_.run(dt, xhat_.r, xc_.r, false);
}

Controller::PID::PID() :
  kp_(0.0f),
  ki_(0.0f),
  kd_(0.0f),
  max_(1.0f),
  integrator_(0.0f),
  differentiator_(0.0f),
  prev_x_(0.0f),
  tau_(0.05)
{}

void Controller::PID::init(float kp, float ki, float kd, float max, float min, float tau)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
  max_ = max;
  tau_ = tau;
}

float Controller::PID::run(float dt, float x, float x_c, bool update_integrator)
{
  float xdot;
  if (dt > 0.0001f)
  {
    // calculate D term (use dirty derivative if we don't have access to a measurement of the derivative)
    // The dirty derivative is a sort of low-pass filtered version of the derivative.
    //// (Include reference to Dr. Beard's notes here)
    differentiator_ = (2.0f * tau_ - dt) / (2.0f * tau_ + dt) * differentiator_
        + 2.0f / (2.0f * tau_ + dt) * (x - prev_x_);
    xdot = differentiator_;
  }
  else
  {
    xdot = 0.0f;
  }
  prev_x_ = x;

  return run(dt, x, x_c, update_integrator, xdot);
}

float Controller::PID::run(float dt, float x, float x_c, bool update_integrator, float xdot)
{
  // Calculate Error
  float error = x_c - x;

  // Initialize Terms
  float p_term = error * kp_;
  float i_term = 0.0f;
  float d_term = 0.0f;

  // If there is a derivative term
  if (kd_ > 0.0f)
  {
    d_term = kd_ * xdot;
  }

  //If there is an integrator term and we are updating integrators
  if ((ki_ > 0.0f) && update_integrator)
  {
    // integrate
    integrator_ += error * dt;
    // calculate I term
    i_term = ki_ * integrator_;
  }

  // sum three terms
  float u = p_term - d_term + i_term;

  // Integrator anti-windup
  float u_sat = (u > max_) ? max_ : (u < -1.0 * max_) ? -1.0 * max_ : u;
  if (u != u_sat && fabs(i_term) > fabs(u - p_term + d_term) && ki_ > 0.0f)
    integrator_ = (u_sat - p_term + d_term)/ki_;

  // Set output
  return u_sat;
}


void Controller::updateWaypointManager()
{
  if (!initialized_)
  {
    initialized_ = true;
    Eigen::Map<Eigen::Vector4d> new_waypoint(waypoints_.block<4,1>(0, 0).data());
    xc_.pn = new_waypoint(PX);
    xc_.pe = new_waypoint(PY);
    xc_.pd = new_waypoint(PZ);
    xc_.psi = new_waypoint(PSI);
  }

  // Find the distance to the desired waypoint
  Eigen::Vector4d current_waypoint = waypoints_.block<4,1>(0, current_waypoint_id_);
  Eigen::Vector4d error;
  error(PX) = current_waypoint(PX) - xhat_.pn;
  error(PY) = current_waypoint(PY) - xhat_.pe;
  error(PZ) = current_waypoint(PZ) - xhat_.pd;
  error(PSI) = current_waypoint(PSI) - xhat_.psi;

  // Angle wrapping on heading
  if (error(PSI) > M_PI)
    error(PSI) -= 2.0 * M_PI;
  else if (error(PSI) < -M_PI)
    error(PSI) += 2.0 * M_PI;

  Eigen::Vector3d current_velocity(xhat_.u, xhat_.v, xhat_.w);

  if (error.norm() < waypoint_threshold_ && current_velocity.norm() < waypoint_velocity_threshold_)
  {
    // increment waypoint
    current_waypoint_id_ = (current_waypoint_id_ + 1) % waypoints_.cols();

    // Update The commanded State
    Eigen::Map<Eigen::Vector4d> new_waypoint(waypoints_.block<4,1>(0, current_waypoint_id_).data());
    xc_.pn = new_waypoint(PX);
    xc_.pe = new_waypoint(PY);
    xc_.pd = new_waypoint(PZ);
    xc_.psi = new_waypoint(PSI);
  }
}

}
