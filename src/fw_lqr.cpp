#include "fw_lqr.h"

namespace fixedwing
{


LQR::LQR() : update_count_(0)
{
  A_.setZero();
  B_.setZero();
}


void LQR::init(const std::string& filename, std::default_random_engine& rng)
{
  load_base(filename);

  common::get_yaml_node("lqr_gain_update_iters", filename, gain_matrix_update_iters_);
  common::get_yaml_node("lqr_max_v_error", filename, v_err_max_);
  common::get_yaml_node("lqr_max_q_error", filename, q_err_max_);
  common::get_yaml_node("lqr_max_omega_error", filename, omega_err_max_);

  Vector4d q_ref;
  common::get_yaml_eigen("lqr_v_ref", filename, v_ref_);
  common::get_yaml_eigen("lqr_q_ref", filename, q_ref);
  common::get_yaml_eigen("lqr_omega_ref", filename, omega_ref_);
  common::get_yaml_eigen("lqr_u_ref", filename, u_ref_);
  q_ref_ = quat::Quatd(q_ref);

  common::get_yaml_node("lqr_chi_inf", filename, chi_inf_);
  common::get_yaml_node("lqr_gamma_inf", filename, gamma_inf_);
  common::get_yaml_node("lqr_k_chi", filename, k_chi_);
  common::get_yaml_node("lqr_k_gamma", filename, k_gamma_);

  common::get_yaml_node("lqr_max_roll", filename, max_roll_);
  common::get_yaml_node("lqr_max_pitch", filename, max_pitch_);

  dxVector Q_diag;
  uVector R_diag;
  common::get_yaml_eigen("lqr_Q", filename, Q_diag);
  common::get_yaml_eigen("lqr_R", filename, R_diag);
  Q_ = Q_diag.asDiagonal();
  R_ = R_diag.asDiagonal();
  R_inv_ = R_.inverse();

  bool perturb_aircraft_parameters;
  double perturbation_percentage;
  common::get_yaml_node("lqr_perturb_aircraft_parameters", filename, perturb_aircraft_parameters);
  common::get_yaml_node("lqr_perturbation_percentage", filename, perturbation_percentage);
  if (perturb_aircraft_parameters)
    perturbAircraftParameters(perturbation_percentage, rng);
}


void LQR::computeControl(const vehicle::Stated& x, const Vector3d& vw, const Vector3d& wp_prev,
                         const Vector3d& wp, vehicle::Stated &xc, uVector& u)
{
  // Copy vehicle state components to LQR state
  State x_lqr;
  x_lqr.v = x.v;
  x_lqr.q = x.q;
  x_lqr.omega = x.omega;

  // Get commanded state
  computeCommandState(x_lqr, x.p, vw, wp_prev, wp, xc_);

  // Populate vehicle command vector
  xc.p = wp;
  xc.v = xc_.v;
  xc.q = xc_.q;
  xc.omega = xc_.omega;

  // Calculate control error
  Vector3d v_err = xc_.v - x_lqr.v;
  Vector3d q_err = xc_.q - x_lqr.q;
  Vector3d omega_err = xc_.omega - x_lqr.omega;

  // Create error state
  dxVector x_tilde;
  x_tilde.segment<3>(DV) = common::saturateVector<double,3>(v_err_max_, v_err);
  x_tilde.segment<3>(DQ) = common::saturateVector<double,3>(q_err_max_, q_err);
  x_tilde.segment<3>(DW) = common::saturateVector<double,3>(omega_err_max_, omega_err);

  if (update_count_ % gain_matrix_update_iters_ == 0)
  {
    // Update Jacobians
//    analyticAB(xhat, vw);
    numericalAB(x_lqr, xc_, u, x.p, vw, wp_prev, wp);

    // Update gain matrix
    care_solver.solve(P_, A_, B_, Q_, R_);
    K_ = R_inv_ * B_.transpose() * P_;

    // Reset update counter
    update_count_ = 0;
  }
  update_count_++;

  // Compute control vector
  u = -K_ * x_tilde;
  u(AIL) = common::saturate(u(AIL), 1.0, -1.0);
  u(ELE) = common::saturate(u(ELE), 1.0, -1.0);
  u(THR) = common::saturate(u(THR), 1.0,  0.0);
  u(RUD) = common::saturate(u(RUD), 1.0, -1.0);
}


void LQR::computeCommandState(const State &x, const Vector3d& pos, const Vector3d& vw,
                              const Vector3d& wp_prev, const Vector3d& wp, State& xc) const
{
  // Compute line path parameters
  Vector3d line_dir = (wp - wp_prev).normalized(); // direction
  Vector3d line_err = (common::I_3x3 - line_dir * line_dir.transpose()) * (pos - wp_prev);

  // Attitude
  Vector3d vI = x.q.rota(x.v);
  double chi_l = atan2(line_dir(1), line_dir(0)); // course angle of line path
  double gamma_l = atan(-line_dir(2) / line_dir.head<2>().norm()); // flight path angle of line path
  double chi = atan2(vI(1), vI(0)); // aircraft course angle
  double gamma = atan(-vI(2) / vI.head<2>().norm()); // flight path angle of aircraft
  Matrix3d R_I2l = Matrix3d::Identity(); // rotation from inertial to path frame
  R_I2l(0,0) = cos(chi_l);
  R_I2l(0,1) = sin(chi_l);
  R_I2l(1,0) = -sin(chi_l);
  R_I2l(1,1) = cos(chi_l);

  double chi_ref = chi_l - chi_inf_ * 2.0 / M_PI * atan(k_chi_ * common::e2.dot(R_I2l * line_err));
  double gamma_ref = gamma_l + gamma_inf_ * 2.0 / M_PI * atan(k_gamma_ * common::e3.dot(R_I2l * line_err));

  double phi_ref = common::saturate(1.0 * common::wrapAngle(chi_ref - chi, M_PI), max_roll_, -max_roll_);
  double theta_ref = common::saturate(1.0 * (gamma_ref - gamma), max_pitch_, -max_pitch_);
  double psi_ref = x.q.yaw();

  xc.q = quat::Quatd(phi_ref, theta_ref, psi_ref);

  // Velocity
  double Va_ref = v_ref_.norm();
  xc.v = v_ref_.norm() * common::e1 + x.q.rotp(vw);

  // Angular rate
  double phi_dot_ref = 0.0;
  double theta_dot_ref = 0.0;
  double psi_dot_ref = common::gravity / Va_ref * tan(phi_ref);
  Matrix3d R = Matrix3d::Identity();
  R(0,2) = -sin(phi_ref);
  R(1,1) = cos(phi_ref);
  R(1,2) = sin(phi_ref) * cos(theta_ref);
  R(2,1) = -sin(phi_ref);
  R(2,2) = cos(phi_ref) * cos(theta_ref);
  Vector3d deuler(phi_dot_ref, theta_dot_ref, psi_dot_ref);
  xc.omega = R * deuler;
}


void LQR::analyticAB(const vehicle::Stated &x, const Vector3d &vw)
{
  //
}


void LQR::numericalAB(const State &x, const State &x_ref, const uVector& u,
                      const Vector3d& pos, const Vector3d &vw, const Vector3d& wp_prev, const Vector3d& wp)
{
  double eps = 1e-5;
  dxMatrix Ix = dxMatrix::Identity();
  uMatrix Iu = uMatrix::Identity();

  // Error state
  dxVector x_tilde = x_ref - x;

  for (int i = 0; i < A_.cols(); ++i)
  {
    // Poke the error state
    dxVector x_tildep = x_tilde + eps * Ix.col(i);
    dxVector x_tildem = x_tilde + -eps * Ix.col(i);

    // Error state derivatives
    dxVector x_tilde_dotp, x_tilde_dotm;
    f_tilde(x_ref, x_tildep, u, pos, vw, wp_prev, wp, eps, x_tilde_dotp);
    f_tilde(x_ref, x_tildem, u, pos, vw, wp_prev, wp, eps, x_tilde_dotm);

    // Derivative of x_tilde_dot w.r.t. x_tilde
    A_.col(i) = (x_tilde_dotp - x_tilde_dotm) / (2.0 * eps);
  }

  for (int i = 0; i < B_.cols(); ++i)
  {
    // Poke the command vector
    uVector up = u + eps * Iu.col(i);
    uVector um = u + -eps * Iu.col(i);

    // Error state derivatives
    dxVector x_tilde_dotp, x_tilde_dotm;
    f_tilde(x_ref, x_tilde, up, pos, vw, wp_prev, wp, eps, x_tilde_dotp);
    f_tilde(x_ref, x_tilde, um, pos, vw, wp_prev, wp, eps, x_tilde_dotm);

    // Derivative of x_tilde_dot w.r.t. u_ref
    B_.col(i) = (x_tilde_dotp - x_tilde_dotm) / (2.0 * eps);
  }
}


void LQR::f_lqr(const State& x, const uVector& u, const Vector3d& vw, dxVector& dx) const
{
  Vector3d v_r = x.v - x.q.rotp(vw); // velocity w.r.t. air in body frame
  double Va = v_r.norm();
  double alpha = atan(v_r(2) / v_r(0));
  double beta = asin(v_r(1) / Va);

  Vector3d f_b = mass_ * common::gravity * x.q.rotp(common::e3) + 0.5 * rho_ * Va * Va * wing_S_ *
                 (C_F_alpha_beta(alpha,beta) + 1.0 / (2.0 * Va) * C_F_omega(alpha) * x.omega + C_F_u(alpha) * u) +
                 rho_ * prop_S_ * prop_C_ * (Va + u(THR) * (k_motor_ - Va)) * u(THR) * (k_motor_ - Va) * common::e1;
  Vector3d tau_b = 0.5 * rho_ * Va * Va * wing_S_ * C_bc<double>() *
                   (C_tau_alpha_beta(alpha,beta) + 1.0 / (2.0 * Va) * C_tau_omega<double>() * x.omega + C_tau_u<double>() * u) -
                   k_T_p_ * k_Omega_ * k_Omega_ * u(THR) * u(THR) * common::e1;

  dx.segment<3>(DV) = 1.0 / mass_ * f_b - x.omega.cross(x.v);
  dx.segment<3>(DQ) = x.omega;
  dx.segment<3>(DW) = J_inv_ * (tau_b - x.omega.cross(J_ * x.omega));
}


void LQR::f_tilde(const State &x_ref, const dxVector &x_tilde, const uVector& u,
                  const Vector3d& pos, const Vector3d& vw, const Vector3d& wp_prev,
                  const Vector3d &wp, const double& dt, dxVector &x_tilde_dot) const
{
  // 'True state'
  State x = x_ref + -x_tilde;

  // Derivative at current time
  dxVector dx;
  f_lqr(x, u, vw, dx);

  // Future and previous states
  State xp = x + dx * dt;
  State xm = x + -dx * dt;

  // Future and previous reference states
  State x_refp, x_refm;
  computeCommandState(xp, pos, vw, wp_prev, wp, x_refp);
  computeCommandState(xm, pos, vw, wp_prev, wp, x_refm);

  // Future and previous error states
  dxVector x_tildep = x_refp - xp;
  dxVector x_tildem = x_refm - xm;

  // Error state derivative
  x_tilde_dot = (x_tildep - x_tildem) / (2.0 * dt);
}


void LQR::perturbAircraftParameters(const double& max_percent_err, std::default_random_engine& rng)
{
  double low = 1.0 - max_percent_err;
  double high = 1.0 + max_percent_err;
  std::uniform_real_distribution<double> dist(low,high);

  rho_ *= dist(rng);
  J_ *= dist(rng);
  J_inv_ = J_.inverse();
  wing_S_ *= dist(rng);
  wing_b_ *= dist(rng);
  wing_c_ *= dist(rng);
  wing_M_ *= dist(rng);
  wing_epsilon_ *= dist(rng);
  wing_alpha0_ *= dist(rng);
  k_T_p_ *= dist(rng);
  k_Omega_ *= dist(rng);
  prop_e_ *= dist(rng);
  prop_S_ *= dist(rng);
  prop_C_ *= dist(rng);
  C_L_0_ *= dist(rng);
  C_L_alpha_ *= dist(rng);
  C_L_beta_ *= dist(rng);
  C_L_p_ *= dist(rng);
  C_L_q_ *= dist(rng);
  C_L_r_ *= dist(rng);
  C_L_delta_a_ *= dist(rng);
  C_L_delta_e_ *= dist(rng);
  C_L_delta_r_ *= dist(rng);
  C_D_0_ *= dist(rng);
  C_D_alpha_ *= dist(rng);
  C_D_beta_ *= dist(rng);
  C_D_p_ *= dist(rng);
  C_D_q_ *= dist(rng);
  C_D_r_ *= dist(rng);
  C_D_delta_a_ *= dist(rng);
  C_D_delta_e_ *= dist(rng);
  C_D_delta_r_ *= dist(rng);
  C_el_0_ *= dist(rng);
  C_el_alpha_ *= dist(rng);
  C_el_beta_ *= dist(rng);
  C_el_p_ *= dist(rng);
  C_el_q_ *= dist(rng);
  C_el_r_ *= dist(rng);
  C_el_delta_a_ *= dist(rng);
  C_el_delta_e_ *= dist(rng);
  C_el_delta_r_ *= dist(rng);
  C_m_0_ *= dist(rng);
  C_m_alpha_ *= dist(rng);
  C_m_beta_ *= dist(rng);
  C_m_p_ *= dist(rng);
  C_m_q_ *= dist(rng);
  C_m_r_ *= dist(rng);
  C_m_delta_a_ *= dist(rng);
  C_m_delta_e_ *= dist(rng);
  C_m_delta_r_ *= dist(rng);
  C_n_0_ *= dist(rng);
  C_n_alpha_ *= dist(rng);
  C_n_beta_ *= dist(rng);
  C_n_p_ *= dist(rng);
  C_n_q_ *= dist(rng);
  C_n_r_ *= dist(rng);
  C_n_delta_a_ *= dist(rng);
  C_n_delta_e_ *= dist(rng);
  C_n_delta_r_ *= dist(rng);
  C_Y_0_ *= dist(rng);
  C_Y_alpha_ *= dist(rng);
  C_Y_beta_ *= dist(rng);
  C_Y_p_ *= dist(rng);
  C_Y_q_ *= dist(rng);
  C_Y_r_ *= dist(rng);
  C_Y_delta_a_ *= dist(rng);
  C_Y_delta_e_ *= dist(rng);
  C_Y_delta_r_ *= dist(rng);

  // Perturb these less
  mass_ *= common::saturate(dist(rng), 1.1, 0.9);
  k_motor_ *= dist(rng);
  delta_a_max_ *= common::saturate(dist(rng), 1.1, 0.9);
  delta_e_max_ *= common::saturate(dist(rng), 1.1, 0.9);
  delta_r_max_ *= common::saturate(dist(rng), 1.1, 0.9);
}


} // namespace fixedwing
