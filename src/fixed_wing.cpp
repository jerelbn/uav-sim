#include "fixed_wing.h"

namespace fixedwing
{


FixedWing::FixedWing()  : t_prev_(0.0) {}


FixedWing::FixedWing(const std::string &filename, environment::Environment& env, const std::default_random_engine& rng, const int& id)
  : t_prev_(0.0), id_(id)
{
  load_base(filename);
  load(filename, env, rng);
}


FixedWing::~FixedWing()
{
  state_log_.close();
  euler_log_.close();
}


void FixedWing::load(const std::string &filename, environment::Environment& env, const std::default_random_engine& rng)
{
  // Instantiate Sensors, Controller, and Estimator classes
  common::get_yaml_node("name", filename, name_);
  controller_.load(filename, rng, name_);
  sensors_.load(filename, rng, name_);
  ekf_.load(filename, name_);

  // Load all FixedWing parameters
  common::get_yaml_node("accurate_integration", filename, accurate_integration_);
  common::get_yaml_node("control_using_estimates", filename, control_using_estimates_);

  vehicle::xVector x0;
  common::get_yaml_eigen<vehicle::xVector>("x0", filename, x0);
  x_ = vehicle::Stated(x0);

  // Initialize other classes
  controller_.computeControl(getState(), 0, u_, other_vehicle_positions_[0], env.getWindVel());
  updateAccels(u_, env.getWindVel());
  sensors_.updateMeasurements(0, x_, env);
  ekf_.run(0, sensors_, env.getWindVel(), getState());

  // Compute trim
  bool compute_trim;
  common::get_yaml_node("compute_trim", filename, compute_trim);
  if (compute_trim)
    computeTrim(filename);

  // Initialize loggers and log initial data
  std::stringstream ss_s, ss_e;
  ss_s << "/tmp/" << name_ << "_true_state.log";
  ss_e << "/tmp/" << name_ << "_euler_angles.log";
  state_log_.open(ss_s.str());
  euler_log_.open(ss_e.str());
  log(0);
}


void FixedWing::propagate(const double &t, const uVector& u, const Vector3d& vw)
{
  // Time step
  double dt = t - t_prev_;
  t_prev_ = t;

  // Integration
  if (accurate_integration_)
  {
    // 4th order Runge-Kutta
    vehicle::rk4<COMMAND_SIZE>(std::bind(&FixedWingBase::f<double>, this,
                               std::placeholders::_1,std::placeholders::_2,
                               std::placeholders::_3,std::placeholders::_4),
                               dt, x_, u, vw, dx_);
  }
  else
  {
    // Euler
    f(x_, u, vw, dx_);
    dx_ *= dt;
  }
  x_ += dx_;
}


void FixedWing::run(const double &t, environment::Environment& env)
{
  getOtherVehicles(env.getVehiclePositions());
  propagate(t, u_, env.getWindVel()); // Propagate truth to current time step
  if (control_using_estimates_)
    controller_.computeControl(ekf_.getState(), t, u_, other_vehicle_positions_[0], ekf_.getWind());
  else
    controller_.computeControl(getState(), t, u_, other_vehicle_positions_[0], env.getWindVel());
  updateAccels(u_, env.getWindVel()); // Update true acceleration
  sensors_.updateMeasurements(t, x_, env);
  ekf_.run(t, sensors_, env.getWindVel(), getState());
  log(t); // Log current data
}


void FixedWing::updateAccels(const uVector &u, const Vector3d &vw)
{
  static vehicle::dxVector dx;
  f(x_, u, vw, dx);
  x_.lin_accel = dx.segment<3>(vehicle::DV);
  x_.ang_accel = dx.segment<3>(vehicle::DW);
}


void FixedWing::log(const double &t)
{
  // Write data to binary files and plot in another program
  vehicle::xVector x = x_.toEigen();
  state_log_.write((char*)&t, sizeof(double));
  state_log_.write((char*)x.data(), x.rows() * sizeof(double));
  euler_log_.write((char*)&t, sizeof(double));
  euler_log_.write((char*)x_.q.euler().data(), 3 * sizeof(double));
}


void FixedWing::getOtherVehicles(const environment::mapVec3 &all_vehicle_positions)
{
  other_vehicle_positions_.clear();
  for (auto it = all_vehicle_positions.begin(); it != all_vehicle_positions.end(); ++it)
    if (it->first != name_)
      other_vehicle_positions_.push_back(it->second);
}


void FixedWing::computeTrim(const std::string& filename) const
{
  // Computed trimmed alpha, beta, phi
  double alpha_star(0), beta_star(0), phi_star(0);
  ceres::Problem problem;
  DynamicsCost* cost = new DynamicsCost(filename);
  problem.AddResidualBlock(new DynamicsCostFactor(cost), NULL, &alpha_star, &beta_star, &phi_star);
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // Compute final trimmed state and trimmed command
  DynamicsCost::TrimState x_star;
  uVector u_star;
  cost->computeTrimmedStateAndCommand(alpha_star, beta_star, phi_star, x_star, u_star);

  // Compute throttle linearization parameters
  double C_F_t, C_tau_t;
  computeLinearizedThrottle(x_star, u_star, C_F_t, C_tau_t);

  // Scale trimmed command
  u_star(AIL) /= delta_a_max_;
  u_star(ELE) /= delta_e_max_;
  u_star(RUD) /= delta_r_max_;

  // Show results
  quat::Quatd q(x_star(DynamicsCost::PHI), x_star(DynamicsCost::THETA), 0);
  std::cout << "\n\n";
  std::cout << "v_star =     " << x_star.segment<3>(DynamicsCost::U).transpose() << "\n";
  std::cout << "q_star =     " << q.elements().transpose() << "\n";
  std::cout << "omega_star = " << x_star.segment<3>(DynamicsCost::P).transpose() << "\n";
  std::cout << "u_star =     " << u_star.transpose() << "\n";
  std::cout << "C_F_t =      " << C_F_t << "\n";
  std::cout << "C_tau_t =    " << C_tau_t << "\n";
  std::cout << "\n\n";

  // Save results to file
  YAML::Node node;
  node["v_star"] = std::vector<double>{x_star(DynamicsCost::U), x_star(DynamicsCost::V), x_star(DynamicsCost::W)};
  node["q_star"] = std::vector<double>{q.w(), q.x(), q.y(), q.z()};
  node["omega_star"] = std::vector<double>{x_star(DynamicsCost::P), x_star(DynamicsCost::Q), x_star(DynamicsCost::R)};
  node["u_star"] = std::vector<double>{u_star(AIL), u_star(ELE), u_star(THR), u_star(RUD)};
  node["C_F_t"] = C_F_t;
  node["C_tau_t"] = C_tau_t;

  std::stringstream ss;
  ss << "/tmp/" << name_ << "_trim.log";
  std::ofstream file(ss.str());
  file << node;
  file.close();
}


void FixedWing::computeLinearizedThrottle(const DynamicsCost::TrimState &x, const uVector &cmd, double& C_F_t, double& C_tau_t) const
{
  // Unpack states and commands for readability
  double u = x(DynamicsCost::U);
  double v = x(DynamicsCost::V);
  double w = x(DynamicsCost::W);
  double theta = x(DynamicsCost::THETA);
  double p = x(DynamicsCost::P);
  double q = x(DynamicsCost::Q);
  double r = x(DynamicsCost::R);

  double delta_a = cmd(AIL);
  double delta_e = cmd(ELE);
  double delta_t = cmd(THR);
  double delta_r = cmd(RUD);

  double g = common::gravity;

  double Gamma = Jx_ * Jz_ - Jxz_ * Jxz_;
  double Gamma_1 = (Jxz_ * (Jx_ - Jy_ + Jz_)) / Gamma;
  double Gamma_2 = (Jz_ * (Jz_ - Jy_) + Jxz_ * Jxz_) / Gamma;
  double Gamma_3 = Jz_ / Gamma;
  double Gamma_4 = Jxz_ / Gamma;

  double C_p_0_ = Gamma_3 * C_el_0_ + Gamma_4 * C_n_0_;
  double C_p_beta_ = Gamma_3 * C_el_beta_ + Gamma_4 * C_n_beta_;
  double C_p_p_ = Gamma_3 * C_el_p_ + Gamma_4 * C_n_p_;
  double C_p_r_ = Gamma_3 * C_el_r_ + Gamma_4 * C_n_r_;
  double C_p_delta_a_ = Gamma_3 * C_el_delta_a_ + Gamma_4 * C_n_delta_a_;
  double C_p_delta_r_ = Gamma_3 * C_el_delta_r_ + Gamma_4 * C_n_delta_r_;

  // Trim dynamics assume no wind
  double Va = x.segment<3>(DynamicsCost::U).norm();
  double alpha = atan2(w, u);
  double beta = asin(v / Va);

  C_F_t = mass_ / delta_t * (q * w - r * v + g * sin(theta) - 0.5 * rho_ * Va * Va * wing_S_ / mass_ * (C_X(alpha) + 0.5 * C_X_q(alpha) * wing_c_ * q / Va + C_X_delta_e(alpha) * delta_e));
  C_tau_t = 1.0 / (Gamma_3 * delta_t) * (Gamma_1 * p * q - Gamma_2 * q * r + 0.5 * rho_ * Va * Va * wing_S_ * wing_b_ * (C_p_0_ + C_p_beta_ * beta + 0.5 * wing_b_ / Va * (C_p_p_ * p + C_p_r_ * r) + C_p_delta_a_ * delta_a + C_p_delta_r_ * delta_r));
}


} // namespace fixedwing
