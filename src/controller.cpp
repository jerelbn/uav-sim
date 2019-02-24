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
  // Initialize random number generator
  common::get_yaml_node("use_random_seed", filename, use_random_seed_);
  int seed;
  if (use_random_seed_)
    seed = std::chrono::system_clock::now().time_since_epoch().count();
  else
    seed = 0;
  rng_ = std::default_random_engine(seed);
  srand(seed);

  common::get_yaml_node("throttle_eq", filename, throttle_eq_);
  common::get_yaml_node("mass", filename, mass_);
  common::get_yaml_node("max_thrust", filename, max_thrust_);

  Vector3d Kp_diag, Kd_diag, Kv_diag;
  if (common::get_yaml_eigen("Kp", filename, Kp_diag))
    K_p_ = Kp_diag.asDiagonal();
  if (common::get_yaml_eigen("Kd", filename, Kd_diag))
    K_d_ = Kd_diag.asDiagonal();
  if (common::get_yaml_eigen("Kv", filename, Kv_diag))
    K_v_ = Kv_diag.asDiagonal();

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

  common::get_yaml_node("path_type", filename, path_type_);

  std::vector<double> loaded_wps;
  if (common::get_yaml_node("waypoints", filename, loaded_wps))
  {
    int num_waypoints = std::floor(loaded_wps.size()/4.0);
    waypoints_ = Map<MatrixXd>(loaded_wps.data(), 4, num_waypoints);
    current_waypoint_id_ = 0;
  }
  common::get_yaml_node("waypoint_threshold", filename, waypoint_threshold_);
  common::get_yaml_node("waypoint_velocity_threshold", filename, waypoint_velocity_threshold_);

  double traj_north_period, traj_east_period, traj_alt_period, traj_yaw_period;
  common::get_yaml_node("traj_delta_north", filename, traj_delta_north_);
  common::get_yaml_node("traj_delta_east", filename, traj_delta_east_);
  common::get_yaml_node("traj_delta_alt", filename, traj_delta_alt_);
  common::get_yaml_node("traj_delta_yaw", filename, traj_delta_yaw_);
  common::get_yaml_node("traj_nom_north", filename, traj_nom_north_);
  common::get_yaml_node("traj_nom_east", filename, traj_nom_east_);
  common::get_yaml_node("traj_nom_alt", filename, traj_nom_alt_);
  common::get_yaml_node("traj_nom_yaw", filename, traj_nom_yaw_);
  common::get_yaml_node("traj_north_period", filename, traj_north_period);
  common::get_yaml_node("traj_east_period", filename, traj_east_period);
  common::get_yaml_node("traj_alt_period", filename, traj_alt_period);
  common::get_yaml_node("traj_yaw_period", filename, traj_yaw_period);
  traj_north_freq_ = 2.0 * M_PI / traj_north_period;
  traj_east_freq_ = 2.0 * M_PI / traj_east_period;
  traj_alt_freq_ = 2.0 * M_PI / traj_alt_period;
  traj_yaw_freq_ = 2.0 * M_PI / traj_yaw_period;

  common::get_yaml_node("circ_rd", filename, circ_rd_);
  common::get_yaml_node("circ_hd", filename, circ_hd_);
  common::get_yaml_node("circ_kr", filename, circ_kr_);
  common::get_yaml_node("circ_kp", filename, circ_kp_);
  common::get_yaml_node("circ_kh", filename, circ_kh_);

  double target_noise_stdev;
  common::get_yaml_node("target_gain", filename, kz_);
  common::get_yaml_node("target_velocity_gain", filename, kvz_);
  common::get_yaml_eigen<Vector3d>("target_z0", filename, z_);
  common::get_yaml_eigen<Vector3d>("target_vz0", filename, vz_);
  common::get_yaml_node("bearing_only", filename, bearing_only_);
  common::get_yaml_node("use_target_truth", filename, use_target_truth_);
  common::get_yaml_node("target_noise_stdev", filename, target_noise_stdev);
  target_noise_dist_ = std::normal_distribution<double>(0.0, target_noise_stdev);
  target_noise_.setZero();

  // Initialize loggers
  target_log_.open("/tmp/target.bin");
}


void Controller::computeControl(const vehicle::State &x, const double t, quadrotor::uVector& u, const Vector3d& pt)
{
  // Copy the current state
  Vector3d euler = x.q.euler();
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

  double dt = t - prev_time_;
  prev_time_ = t;

  if (dt < 0.0001)
  {
    u.setZero();
    return;
  }

  if (path_type_ == 0 || path_type_ == 1)
  {
    // Refresh the waypoint or trajectory
    if (path_type_ == 0)
      updateWaypointManager();
    else
      updateTrajectoryManager();

    // get data that applies to both position and velocity control
    Matrix3d R_v_to_v1 = common::R_v_to_v1(euler(2)); // rotation from vehicle to vehicle-1 frame
    Matrix3d R_v1_to_b = common::R_v_to_b(euler(0), euler(1), 0.0); // rotation from vehicle-1 to body frame
    static Vector3d k = common::e3;

    Vector3d phat(xhat_.pn, xhat_.pe, xhat_.pd); // position estimate
    Vector3d pc(xc_.pn, xc_.pe, xc_.pd); // position command
    Vector3d vc = R_v_to_v1 * K_p_ * (pc-phat); // velocity command

    // store velocity command
    xc_.u = vc(0);
    xc_.v = vc(1);
    xc_.w = vc(2);

    // get yaw rate direction and allow it to saturate
    xc_.r = common::wrapAngle(xc_.psi - xhat_.psi, M_PI); // wrap angle

    // get velocity command and enforce max velocity
    double vmag = vc.norm();
    if (vmag > max_.vel)
      vc = vc*max_.vel/vmag;

    Vector3d vhat_b(xhat_.u,xhat_.v,xhat_.w); // body velocity estimate
    Vector3d vhat = R_v1_to_b.transpose()*vhat_b; // vehicle-1 velocity estimate
    dhat_ = dhat_ - K_d_*(vc-vhat)*dt; // update disturbance estimate
    Vector3d k_tilde = throttle_eq_ * (k - (1.0 / common::gravity) * (K_v_ * (vc - vhat) - dhat_));

    // pack up throttle command
    xc_.throttle = k.transpose() * R_v1_to_b * k_tilde;

    Vector3d kd = (1.0 / xc_.throttle) * k_tilde; // desired body z direction
    kd = kd / kd.norm(); // need direction only
    double tilt_angle = acos(k.transpose() * kd); // desired tilt

    // get shortest rotation to desired tilt
    quat::Quatd q_c;
    if (tilt_angle > 1e-6)
    {
      Vector3d k_cross_kd = k.cross(kd);
      q_c = quat::Quatd::exp(tilt_angle * k_cross_kd / k_cross_kd.norm());
    }

    // pack up attitude commands
    xc_.phi = q_c.roll();
    xc_.theta = q_c.pitch();
  }
  else if (path_type_ == 2)
  {
    // Create noisy bearing measurement of target and full vector measurement
    if (!use_target_truth_)
      common::randomNormal(target_noise_, target_noise_dist_, rng_);
    Vector3d z_true = x.q.rotp(pt - x.p);
    Vector3d z = z_true + target_noise_;
    Vector3d ez = z / z.norm();
    Vector3d z_tilde = z_ - z;

    // Target estimator
    Vector3d zdot(0, 0, 0), vzdot(0, 0, 0);
    if (bearing_only_)
    {
      zdot = -x.v - x.omega.cross(z_) - kz_ * (common::I_3x3 - ez * ez.transpose()) * z_;
    }
    else
    {
      zdot = vz_ - x.omega.cross(z_) - kz_ * z_tilde;
      vzdot = -x.omega.cross(vz_) - x.lin_accel - x.omega.cross(x.v) - kvz_ * z_tilde;
    }
    z_ += zdot * dt;
    vz_ += vzdot * dt;


    // Extract local level frame rotation
    double phi = x.q.roll();
    double theta = x.q.pitch();
    quat::Quatd q_l2b(phi, theta, 0.0);

    // Commanded velocity in the local level reference frame
    static const Matrix3d IPe3 = common::I_3x3 - common::e3 * common::e3.transpose();
    static const Matrix3d Pe3 = common::e3 * common::e3.transpose();
    double r = (IPe3 * q_l2b.rota(z_)).norm();
    double h = (Pe3 * q_l2b.rota(z_)).norm();

    Vector3d er = IPe3 * q_l2b.rota(ez);
    er /= er.norm();
    Vector3d ep = common::e3.cross(er);
    ep /= ep.norm();

    double r_tilde = r - circ_rd_;
    double h_tilde = h - circ_hd_;
    Vector3d vc;
    if (bearing_only_)
      vc = circ_kr_ * r_tilde * er + circ_kp_ * ep + circ_kh_ * h_tilde * common::e3;
    else
      vc = circ_kr_ * r_tilde * er + circ_kp_ * ep + circ_kh_ * h_tilde * common::e3 + q_l2b.rota(vz_ + x.v);
    double vmag = vc.norm();
    if (vmag > max_.vel)
      vc *= max_.vel / vmag;
    xc_.u = vc(0);
    xc_.v = vc(1);
    xc_.w = vc(2);

    // Commanded yaw rate
    xc_.r = 10.0 * common::e3.transpose() * common::e1.cross(ez);

    Vector3d vtilde = vc - q_l2b.rota(x.v);
    Vector3d omega_l = common::e3 * common::e3.transpose() * q_l2b.rota(x.omega);
    Vector3d vl = q_l2b.rota(x.v);

    // Commanded throttle
    xc_.throttle = throttle_eq_ * common::e3.transpose() * q_l2b.rotp(common::e3
                   - 1.0 / common::gravity * (omega_l.cross(vl) + K_v_ * vtilde));

    // Commanded body axes in the inertial frame
    Vector3d kbc = common::e3 - 1.0 / common::gravity * (omega_l.cross(vl) + K_v_ * vtilde);
    kbc /= kbc.norm();
    Vector3d jbc = kbc.cross(q_l2b.rota(ez));
    jbc /= jbc.norm();
    Vector3d ibc = jbc.cross(kbc);
    ibc /= ibc.norm();

    // Build commanded attitude
    Matrix3d Rc; // Should be inertial to body rotation
    Rc << ibc, jbc, kbc;

    // Extract roll and pitch angles
    xc_.phi = atan2(-Rc(1,2), Rc(1,1));
    xc_.theta = atan2(-Rc(2,0),Rc(0,0));
  }
  else
    throw std::runtime_error("Undefined path type in controller.");

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
    Map<Vector4d> new_waypoint(waypoints_.block<4,1>(0, 0).data());
    xc_.pn = new_waypoint(PX);
    xc_.pe = new_waypoint(PY);
    xc_.pd = new_waypoint(PZ);
    xc_.psi = new_waypoint(PSI);
  }

  // Find the distance to the desired waypoint
  Vector4d current_waypoint = waypoints_.block<4,1>(0, current_waypoint_id_);
  Vector4d error;
  error(PX) = current_waypoint(PX) - xhat_.pn;
  error(PY) = current_waypoint(PY) - xhat_.pe;
  error(PZ) = current_waypoint(PZ) - xhat_.pd;
  error(PSI) = current_waypoint(PSI) - xhat_.psi;

  // Angle wrapping on heading
  if (error(PSI) > M_PI)
    error(PSI) -= 2.0 * M_PI;
  else if (error(PSI) < -M_PI)
    error(PSI) += 2.0 * M_PI;

  Vector3d current_velocity(xhat_.u, xhat_.v, xhat_.w);

  if (error.norm() < waypoint_threshold_ && current_velocity.norm() < waypoint_velocity_threshold_)
  {
    // increment waypoint
    current_waypoint_id_ = (current_waypoint_id_ + 1) % waypoints_.cols();

    // Update The commanded State
    Map<Vector4d> new_waypoint(waypoints_.block<4,1>(0, current_waypoint_id_).data());
    xc_.pn = new_waypoint(PX);
    xc_.pe = new_waypoint(PY);
    xc_.pd = new_waypoint(PZ);
    xc_.psi = new_waypoint(PSI);
  }
}


void Controller::updateTrajectoryManager()
{
  xc_.pn = traj_nom_north_ + traj_delta_north_ / 2.0 * cos(traj_north_freq_ * xc_.t);
  xc_.pe = traj_nom_east_ + traj_delta_east_ / 2.0 * sin(traj_east_freq_ * xc_.t);
  xc_.pd = -(traj_nom_alt_ + traj_delta_alt_ / 2.0 * sin(traj_alt_freq_ * xc_.t));
  xc_.psi = traj_nom_yaw_ + traj_delta_yaw_ / 2.0 * sin(traj_yaw_freq_ * xc_.t);
}


void Controller::log(const double &t)
{
  // Write data to binary files and plot in another program
  target_log_.write((char*)&t, sizeof(double));
  target_log_.write((char*)z_.data(), z_.rows() * sizeof(double));
  target_log_.write((char*)vz_.data(), vz_.rows() * sizeof(double));
}

}
