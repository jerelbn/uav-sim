#include "fixed_wing.h"

namespace fixedwing
{


FixedWing::FixedWing()  : t_prev_(0.0) {}


FixedWing::FixedWing(const std::string &filename, const environment::Environment& env, const bool& use_random_seed, const int& id)
  : t_prev_(0.0), id_(id)
{
  load(filename, env, use_random_seed);
}


FixedWing::~FixedWing()
{
  state_log_.close();
  command_log_.close();
}


void FixedWing::load(const std::string &filename, const environment::Environment& env, const bool& use_random_seed)
{
  // Instantiate Sensors, Controller, and Estimator classes
  common::get_yaml_node("name", filename, name_);
  controller_.load(filename, use_random_seed, name_);
  sensors_.load(filename, use_random_seed, name_);

  // Load all FixedWing parameters
  common::get_yaml_node("accurate_integration", filename, accurate_integration_);
  common::get_yaml_node("control_using_estimates", filename, control_using_estimates_);

  vehicle::xVector x0;
  common::get_yaml_eigen<vehicle::xVector>("x0", filename, x0);
  x_ = vehicle::State<double>(x0);

  common::get_yaml_node("mass", filename, mass_);
  common::get_yaml_node("Jx", filename, Jx_);
  common::get_yaml_node("Jy", filename, Jy_);
  common::get_yaml_node("Jz", filename, Jz_);
  common::get_yaml_node("Jxz", filename, Jxz_);
  J_ << Jx_, 0, -Jxz_, 0, Jy_, 0, -Jxz_, 0, Jz_;
  J_inv_ = J_.inverse();

  common::get_yaml_node("rho", filename, rho_);
  common::get_yaml_node("wing_S", filename, wing_S_);
  common::get_yaml_node("wing_b", filename, wing_b_);
  common::get_yaml_node("wing_c", filename, wing_c_);
  common::get_yaml_node("wing_M", filename, wing_M_);
  common::get_yaml_node("wing_epsilon", filename, wing_epsilon_);
  common::get_yaml_node("wing_alpha0", filename, wing_alpha0_);

  common::get_yaml_node("k_motor", filename, k_motor_);
  common::get_yaml_node("k_T_p", filename, k_T_p_);
  common::get_yaml_node("k_Omega", filename, k_Omega_);

  common::get_yaml_node("prop_e", filename, prop_e_);
  common::get_yaml_node("prop_S", filename, prop_S_);
  common::get_yaml_node("prop_C", filename, prop_C_);

  common::get_yaml_node("C_L_0", filename, C_L_0_);
  common::get_yaml_node("C_L_alpha", filename, C_L_alpha_);
  common::get_yaml_node("C_L_beta", filename, C_L_beta_);
  common::get_yaml_node("C_L_p", filename, C_L_p_);
  common::get_yaml_node("C_L_q", filename, C_L_q_);
  common::get_yaml_node("C_L_r", filename, C_L_r_);
  common::get_yaml_node("C_L_delta_a", filename, C_L_delta_a_);
  common::get_yaml_node("C_L_delta_e", filename, C_L_delta_e_);
  common::get_yaml_node("C_L_delta_r", filename, C_L_delta_r_);

  common::get_yaml_node("C_D_0", filename, C_D_0_);
  common::get_yaml_node("C_D_alpha", filename, C_D_alpha_);
  common::get_yaml_node("C_D_beta", filename, C_D_beta_);
  common::get_yaml_node("C_D_p", filename, C_D_p_);
  common::get_yaml_node("C_D_q", filename, C_D_q_);
  common::get_yaml_node("C_D_r", filename, C_D_r_);
  common::get_yaml_node("C_D_delta_a", filename, C_D_delta_a_);
  common::get_yaml_node("C_D_delta_e", filename, C_D_delta_e_);
  common::get_yaml_node("C_D_delta_r", filename, C_D_delta_r_);

  common::get_yaml_node("C_el_0", filename, C_el_0_);
  common::get_yaml_node("C_el_alpha", filename, C_el_alpha_);
  common::get_yaml_node("C_el_beta", filename, C_el_beta_);
  common::get_yaml_node("C_el_p", filename, C_el_p_);
  common::get_yaml_node("C_el_q", filename, C_el_q_);
  common::get_yaml_node("C_el_r", filename, C_el_r_);
  common::get_yaml_node("C_el_delta_a", filename, C_el_delta_a_);
  common::get_yaml_node("C_el_delta_e", filename, C_el_delta_e_);
  common::get_yaml_node("C_el_delta_r", filename, C_el_delta_r_);

  common::get_yaml_node("C_m_0", filename, C_m_0_);
  common::get_yaml_node("C_m_alpha", filename, C_m_alpha_);
  common::get_yaml_node("C_m_beta", filename, C_m_beta_);
  common::get_yaml_node("C_m_p", filename, C_m_p_);
  common::get_yaml_node("C_m_q", filename, C_m_q_);
  common::get_yaml_node("C_m_r", filename, C_m_r_);
  common::get_yaml_node("C_m_delta_a", filename, C_m_delta_a_);
  common::get_yaml_node("C_m_delta_e", filename, C_m_delta_e_);
  common::get_yaml_node("C_m_delta_r", filename, C_m_delta_r_);

  common::get_yaml_node("C_n_0", filename, C_n_0_);
  common::get_yaml_node("C_n_alpha", filename, C_n_alpha_);
  common::get_yaml_node("C_n_beta", filename, C_n_beta_);
  common::get_yaml_node("C_n_p", filename, C_n_p_);
  common::get_yaml_node("C_n_q", filename, C_n_q_);
  common::get_yaml_node("C_n_r", filename, C_n_r_);
  common::get_yaml_node("C_n_delta_a", filename, C_n_delta_a_);
  common::get_yaml_node("C_n_delta_e", filename, C_n_delta_e_);
  common::get_yaml_node("C_n_delta_r", filename, C_n_delta_r_);

  common::get_yaml_node("C_Y_0", filename, C_Y_0_);
  common::get_yaml_node("C_Y_alpha", filename, C_Y_alpha_);
  common::get_yaml_node("C_Y_beta", filename, C_Y_beta_);
  common::get_yaml_node("C_Y_p", filename, C_Y_p_);
  common::get_yaml_node("C_Y_q", filename, C_Y_q_);
  common::get_yaml_node("C_Y_r", filename, C_Y_r_);
  common::get_yaml_node("C_Y_delta_a", filename, C_Y_delta_a_);
  common::get_yaml_node("C_Y_delta_e", filename, C_Y_delta_e_);
  common::get_yaml_node("C_Y_delta_r", filename, C_Y_delta_r_);

  common::get_yaml_node("delta_a_max", filename, delta_a_max_);
  common::get_yaml_node("delta_e_max", filename, delta_e_max_);
  common::get_yaml_node("delta_r_max", filename, delta_r_max_);

  // Compute initial control and corresponding acceleration
  controller_.computeControl(getState(), 0, u_, other_vehicle_positions_[0]);
  updateAccels(u_, env.get_vw());

  // Compute trim
  bool compute_trim;
  common::get_yaml_node("compute_trim", filename, compute_trim);
  if (compute_trim)
    computeTrim(filename);

  // Initialize loggers and log initial data
  std::stringstream ss_ts, ss_c;
  ss_ts << "/tmp/" << name_ << "_true_state.log";
  ss_c << "/tmp/" << name_ << "_command.log";
  state_log_.open(ss_ts.str());
  command_log_.open(ss_c.str());
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
    vehicle::rk4<COMMAND_SIZE>(std::bind(&FixedWing::f, this,
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


void FixedWing::run(const double &t, const environment::Environment& env)
{
  getOtherVehicles(env.getVehiclePositions());
  sensors_.updateMeasurements(t, x_, env.get_points()); // Update sensor measurements
  propagate(t, u_, env.get_vw()); // Propagate truth to next time step
  controller_.computeControl(getState(), t, u_, other_vehicle_positions_[0]); // Update control input with truth
  updateAccels(u_, env.get_vw()); // Update true acceleration
  log(t); // Log current data
}


void FixedWing::f(const vehicle::State<double>& x, const uVector& u, const Vector3d& vw, vehicle::dxVector& dx) const
{

  Vector3d v_r = x.v - x.q.rotp(vw); // velocity w.r.t. air in body frame
  double Va = v_r.norm();
  double alpha = atan2(v_r(2), v_r(0));
  double beta = asin(v_r(1) / Va);

  Vector3d C_F_alpha_beta(C_X(alpha), C_Y_0_ + C_Y_beta_ * beta, C_Z(alpha));
  Matrix3d C_F_omega = Matrix3d::Zero();
  C_F_omega(0,1) = C_X_q(alpha) * wing_c_;
  C_F_omega(1,0) = C_Y_p_ * wing_b_;
  C_F_omega(1,2) = C_Y_r_ * wing_b_;
  C_F_omega(2,1) = C_Z_q(alpha) * wing_c_;
  Matrix<double,3,4> C_F_u = Matrix<double,3,4>::Zero();
  C_F_u(0,1) = C_X_delta_e(alpha) * delta_e_max_;
  C_F_u(1,0) = C_Y_delta_a_ * delta_a_max_;
  C_F_u(1,3) = C_Y_delta_r_ * delta_r_max_;
  C_F_u(2,1) = C_Z_delta_e(alpha) * delta_e_max_;
  Matrix3d C_bc = Vector3d(wing_b_, wing_c_, wing_b_).asDiagonal();
  Vector3d C_tau_alpha_beta(C_el_0_ + C_el_beta_ * beta,
                            C_m_0_ + C_m_alpha_ * alpha,
                            C_n_0_ + C_n_beta_ * beta);
  Matrix3d C_tau_omega = Matrix3d::Zero();
  C_tau_omega(0,0) = C_el_p_ * wing_b_;
  C_tau_omega(0,2) = C_el_r_ * wing_b_;
  C_tau_omega(1,1) = C_m_q_ * wing_c_;
  C_tau_omega(2,0) = C_n_p_ * wing_b_;
  C_tau_omega(2,2) = C_n_r_ * wing_b_;
  Matrix<double,3,4> C_tau_u = Matrix<double,3,4>::Zero();
  C_tau_u(0,0) = C_el_delta_a_ * delta_a_max_;
  C_tau_u(0,3) = C_el_delta_r_ * delta_r_max_;
  C_tau_u(1,1) = C_m_delta_e_ * delta_e_max_;
  C_tau_u(2,0) = C_n_delta_a_ * delta_a_max_;
  C_tau_u(2,3) = C_n_delta_r_ * delta_r_max_;

  Vector3d f_b = mass_ * common::gravity * x.q.rotp(common::e3) + 0.5 * rho_ * Va * Va * wing_S_ *
                 (C_F_alpha_beta + 1.0 / (2.0 * Va) * C_F_omega * x.omega + C_F_u * u) +
                 rho_ * prop_S_ * prop_C_ * (Va + u(THR) * (k_motor_ - Va)) * u(THR) * (k_motor_ - Va) * common::e1;
  Vector3d tau_b = 0.5 * rho_ * Va * Va * wing_S_ * C_bc *
                   (C_tau_alpha_beta + 1.0 / (2.0 * Va) * C_tau_omega * x.omega + C_tau_u * u) -
                   k_T_p_ * k_Omega_ * k_Omega_ * u(THR) * u(THR) * common::e1;

  dx.template segment<3>(vehicle::DP) = x.q.rota(x.v);
  dx.template segment<3>(vehicle::DV) = 1.0 / mass_ * f_b - x.omega.cross(x.v);
  dx.template segment<3>(vehicle::DQ) = x.omega;
  dx.template segment<3>(vehicle::DW) = J_inv_ * (tau_b - x.omega.cross(J_ * x.omega));
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
  vehicle::xVector commanded_state = controller_.getCommandedState().toEigen();
  command_log_.write((char*)&t, sizeof(double));
  command_log_.write((char*)commanded_state.data(), commanded_state.rows() * sizeof(double));
}


void FixedWing::getOtherVehicles(const std::vector<Vector3d, aligned_allocator<Vector3d> > &all_vehicle_positions)
{
  other_vehicle_positions_.clear();
  for (int i = 0; i < all_vehicle_positions.size(); ++i)
    if (i != id_)
      other_vehicle_positions_.push_back(all_vehicle_positions[i]);
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
  TrimState x_star;
  uVector u_star;
  cost->computeTrimmedStateAndCommand(alpha_star, beta_star, phi_star, x_star, u_star);

  // Scale trimmed command
  u_star(AIL) /= delta_a_max_;
  u_star(ELE) /= delta_e_max_;
  u_star(RUD) /= delta_r_max_;

  // Show results
  quat::Quatd q(x_star(PHI), x_star(THETA), 0);
  std::cout << "\n\n";
  std::cout << "v_star =     " << x_star.segment<3>(U).transpose() << "\n";
  std::cout << "q_star =     " << q.elements().transpose() << "\n";
  std::cout << "omega_star = " << x_star.segment<3>(P).transpose() << "\n";
  std::cout << "u_star =     " << u_star.transpose() << "\n";
  std::cout << "\n\n";

  // Save results to file
  YAML::Node node;
  node["v_star"] = std::vector<double>{x_star(U), x_star(V), x_star(W)};
  node["q_star"] = std::vector<double>{q.w(), q.x(), q.y(), q.z()};
  node["omega_star"] = std::vector<double>{x_star(P), x_star(Q), x_star(R)};
  node["u_star"] = std::vector<double>{u_star(AIL), u_star(ELE), u_star(THR), u_star(RUD)};

  std::stringstream ss;
  ss << "/tmp/" << name_ << "_trim.log";
  std::ofstream file(ss.str());
  file << node;
  file.close();
}


} // namespace fixedwing
