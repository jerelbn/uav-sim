#include "fw_lqr.h"

namespace fixedwing
{


LQR::LQR() : update_count_(0)
{
  A_.setZero();
  B_.setZero();
}


void LQR::init(const std::string& filename)
{
  load_base(filename);

  common::get_yaml_node("lqr_gain_update_iters", filename, gain_matrix_update_iters_);
  common::get_yaml_node("lqr_max_p_error", filename, p_err_max_);
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

  Matrix<double,12,1> Q_diag;
  Matrix<double,4,1> R_diag;
  common::get_yaml_eigen("lqr_Q", filename, Q_diag);
  common::get_yaml_eigen("lqr_R", filename, R_diag);
  Q_ = Q_diag.asDiagonal();
  R_ = R_diag.asDiagonal();
  R_inv_ = R_.inverse();

  u_prev_ = u_ref_;
}


void LQR::computeControl(const vehicle::Stated& xhat, const Vector3d& vw, const Vector3d& wp_prev,
                         const Vector3d& wp, vehicle::Stated &xc, uVector& u)
{
  // Get commanded state
  computeCommandState(xhat, vw, wp_prev, wp, xc);

  // Calculate control error
  Vector3d p_err = Vector3d::Zero();
  Vector3d v_err = xc.v - xhat.v;
  Vector3d q_err = xc.q - xhat.q;
  Vector3d omega_err = xc.omega - xhat.omega;

  // Create error state
  Matrix<double,12,1> x_tilde;
  x_tilde.segment<3>(0) = common::saturateVector<double,3>(p_err_max_, p_err);
  x_tilde.segment<3>(3) = common::saturateVector<double,3>(v_err_max_, v_err);
  x_tilde.segment<3>(6) = common::saturateVector<double,3>(q_err_max_, q_err);
  x_tilde.segment<3>(9) = common::saturateVector<double,3>(omega_err_max_, omega_err);

  if (update_count_ % gain_matrix_update_iters_ == 0)
  {
    // Update Jacobians
//    analyticAB(xhat, vw);
//    std::cout << "\nAa = \n" << A_ << "\n\nBa = \n" << B_ << std::endl;
    numericalAB(xhat, xc, u_ref_, vw, wp_prev, wp);
//    std::cout << "\nAn = \n" << A_ << "\n\nBn = \n" << B_ << std::endl;

    // Update gain matrix
    care_solver.solve(P_, A_, B_, Q_, R_);
    K_ = R_inv_ * B_.transpose() * P_;

    // Reset update counter
    update_count_ = 0;
  }
  update_count_++;

  // Compute control vector
  u = -K_ * x_tilde;
//  uVector u_tilde = -K_ * x_tilde;
//  u = u_ref_ - u_tilde;
  u(AIL) = common::saturate(u(AIL), 1.0, -1.0);
  u(ELE) = common::saturate(u(ELE), 1.0, -1.0);
  u(THR) = common::saturate(u(THR), 1.0,  0.0);
  u(RUD) = common::saturate(u(RUD), 1.0, -1.0);

  u_prev_ = u;
}


void LQR::computeCommandState(const vehicle::Stated &x, const Vector3d& vw, const Vector3d& wp_prev, const Vector3d& wp, vehicle::Stated &xc) const
{
  // Compute line path parameters
  Vector3d line_dir = (wp - wp_prev).normalized(); // direction
  Vector3d line_err = (common::I_3x3 - line_dir * line_dir.transpose()) * (x.p - wp_prev);

  // Altitude
  xc.p = wp;
//  xc.p(2) = common::e3.dot(wp_prev + line_dir * line_dir.dot(x.p - wp_prev));

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

  double phi_ref = common::saturate(1.0 * common::wrapAngle(chi_ref - chi, M_PI), 1.0472, -1.0472);
  double theta_ref = common::saturate(1.0 * (gamma_ref - gamma), 0.7854, -0.7854);
  double psi_ref = x.q.yaw();

  xc.q = quat::Quatd(phi_ref, theta_ref, psi_ref);

  // Velocity
  Vector3d v_aI_b = x.v - x.q.rotp(vw); // velocity w.r.t. air in body frame
  double Va_ref = v_ref_.norm();
  double alpha = atan(v_aI_b(2) / v_aI_b(0));
  double gamma_a = x.q.pitch() - alpha;
  xc.v = x.q.rotp(Va_ref * Vector3d(cos(x.q.yaw()) * cos(gamma_a), sin(x.q.yaw()) * cos(gamma_a), -sin(gamma_a)) + vw);

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
  Vector3d v_r = x.v - x.q.rotp(vw); // velocity w.r.t. air in body frame
  double Va = v_r.norm();
  double alpha = atan2(v_r(2), v_r(0));
//  double beta = asin(v_r(1) / Va);

  A_.setZero();
  A_.block<3,3>(0,3) = x.q.inverse().R();
  A_.block<3,3>(0,6) = -x.q.inverse().R() * common::skew(x.v);
  A_.block<3,3>(3,3) = -common::skew(x.omega);
  A_.block<3,3>(3,6) = common::skew(x.q.rotp(common::gravity * common::e3));
  A_.block<3,3>(3,9) = 0.25 * rho_ * Va * wing_S_ / mass_ * C_F_omega(alpha) + common::skew(x.v);
  A_.block<3,3>(6,6) = -common::skew(x.omega);
  A_.block<3,3>(6,9) = common::I_3x3;
  A_.block<3,3>(9,9) = J_inv_ * (0.25 * rho_ * Va * wing_S_ * C_bc<double>() * C_tau_omega<double>() -
                       common::skew(x.omega) * J_ + common::skew(J_ * x.omega));

  B_.setZero();
  B_.block<3,4>(3,0) = 0.5 * rho_ * Va * Va * wing_S_ / mass_ * C_F_ul(alpha, Va);
  B_.block<3,4>(9,0) = 0.5 * rho_ * Va * Va * wing_S_ * J_inv_ * C_bc<double>() * C_tau_ul(Va);
}


void LQR::numericalAB(const vehicle::Stated &x, const vehicle::Stated &x_ref,
                        const uVector& u, const Vector3d &vw, const Vector3d& wp_prev, const Vector3d& wp)
{
  double eps = 1e-5;
  Matrix<double,12,12> I12 = Matrix<double,12,12>::Identity();
  Matrix4d I4 = Matrix4d::Identity();

  // Error state
  vehicle::dxVector x_tilde = x_ref - x;

  for (int i = 0; i < A_.cols(); ++i)
  {
    // Poke the error state
    vehicle::dxVector x_tildep = x_tilde + eps * I12.col(i);
    vehicle::dxVector x_tildem = x_tilde + -eps * I12.col(i);

    // Error state derivatives
    vehicle::dxVector x_tilde_dotp, x_tilde_dotm;
    f_tilde(x_ref, x_tildep, u, vw, wp_prev, wp, eps, x_tilde_dotp);
    f_tilde(x_ref, x_tildem, u, vw, wp_prev, wp, eps, x_tilde_dotm);

    // Derivative of x_tilde_dot w.r.t. x_tilde
    A_.col(i) = (x_tilde_dotp - x_tilde_dotm) / (2.0 * eps);
  }
  for (int i = 0; i < B_.cols(); ++i)
  {
    // Poke the command vector
    Vector4d up = u + eps * I4.col(i);
    Vector4d um = u + -eps * I4.col(i);

    // Error state derivatives
    vehicle::dxVector x_tilde_dotp, x_tilde_dotm;
    f_tilde(x_ref, x_tilde, up, vw, wp_prev, wp, eps, x_tilde_dotp);
    f_tilde(x_ref, x_tilde, um, vw, wp_prev, wp, eps, x_tilde_dotm);

    // Derivative of x_tilde_dot w.r.t. u_ref
    B_.col(i) = (x_tilde_dotp - x_tilde_dotm) / (2.0 * eps);
  }
}


void LQR::f_tilde(const vehicle::Stated &x_ref, const vehicle::dxVector &x_tilde,
                  const uVector& u, const Vector3d& vw, const Vector3d& wp_prev, const Vector3d &wp,
                  const double& dt, vehicle::dxVector &x_tilde_dot) const
{
  // 'True state'
  vehicle::Stated x = x_ref + -x_tilde;

  // Derivative at current time
  vehicle::dxVector dx;
  f(x, u, vw, dx);

  // Future and previous states
  vehicle::Stated xp = x + dx * dt;
  vehicle::Stated xm = x + -dx * dt;

  // Future and previous reference states
  vehicle::Stated x_refp = x_ref;
  vehicle::Stated x_refm = x_ref;
  computeCommandState(xp, vw, wp_prev, wp, x_refp);
  computeCommandState(xm, vw, wp_prev, wp, x_refm);

  // Future and previous error states
  vehicle::dxVector x_tildep = x_refp - xp;
  vehicle::dxVector x_tildem = x_refm - xm;

  // Error state derivative
  x_tilde_dot = (x_tildep - x_tildem) / (2.0 * dt);
}


} // namespace fixedwing
