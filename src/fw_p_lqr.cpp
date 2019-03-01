#include "fw_p_lqr.h"

namespace fixedwing
{


PLQR::PLQR()
{
  A_.setZero();
  B_.setZero();
}


void PLQR::init(const std::string& filename)
{
  load_base(filename);

  common::get_yaml_node("plqr_max_p_error", filename, p_err_max_);
  common::get_yaml_node("plqr_max_v_error", filename, v_err_max_);
  common::get_yaml_node("plqr_max_q_error", filename, q_err_max_);
  common::get_yaml_node("plqr_max_omega_error", filename, omega_err_max_);

  Vector4d q_ref;
  common::get_yaml_eigen("plqr_v_ref", filename, v_ref_);
  common::get_yaml_eigen("plqr_q_ref", filename, q_ref);
  common::get_yaml_eigen("plqr_omega_ref", filename, omega_ref_);
  common::get_yaml_eigen("plqr_u_ref", filename, u_ref_);
  q_ref_ = quat::Quatd(q_ref);

  Matrix<double,12,1> Q_diag;
  Matrix<double,4,1> R_diag;
  common::get_yaml_eigen("plqr_Q", filename, Q_diag);
  common::get_yaml_eigen("plqr_R", filename, R_diag);
  Q_ = Q_diag.asDiagonal();
  R_ = R_diag.asDiagonal();
  R_inv_ = R_.inverse();
}


void PLQR::computeControl(const vehicle::Stated& xhat, const Vector3d& vw, vehicle::Stated& xc, uVector& u)
{
  // Put reference velocity along direction of waypoint
  v_ref_ = xhat.q.rotp(xc.p - xhat.p);

  // Calculate control error
  Vector3d p_err = xc.p - xhat.p;
  Vector3d v_err = v_ref_ - xhat.v;
  Vector3d q_err = q_ref_ - xhat.q;
  Vector3d omega_err = omega_ref_ - xhat.omega;

  // Create error state
  Matrix<double,12,1> x_tilde;
  x_tilde.segment<3>(0) = common::saturateVector<double,3>(p_err_max_, p_err);
  x_tilde.segment<3>(3) = common::saturateVector<double,3>(v_err_max_, v_err);
  x_tilde.segment<3>(6) = common::saturateVector<double,3>(q_err_max_, q_err);
  x_tilde.segment<3>(9) = common::saturateVector<double,3>(omega_err_max_, omega_err);

  // Jacobians
  Vector3d v_r = xhat.v - xhat.q.rotp(vw); // velocity w.r.t. air in body frame
  double Va = v_r.norm();
  double alpha = atan2(v_r(2), v_r(0));
//  double beta = asin(v_r(1) / Va);

  A_.block<3,3>(0,3) = xhat.q.inverse().R();
  A_.block<3,3>(0,6) = -xhat.q.inverse().R() * common::skew(xhat.v);
  A_.block<3,3>(3,3) = -common::skew(xhat.omega);
  A_.block<3,3>(3,6) = common::skew(xhat.q.rotp(common::gravity * common::e3));
  A_.block<3,3>(3,9) = 0.25 / mass_ * rho_ * Va * wing_S_ * C_F_omega(alpha) + common::skew(xhat.v);
  A_.block<3,3>(6,6) = -common::skew(xhat.omega);
  A_.block<3,3>(6,9) = common::I_3x3;
  A_.block<3,3>(9,9) = J_inv_ * (0.25 * rho_ * Va * wing_S_ * C_bc<double>() * C_tau_omega<double>() +
                       common::skew(J_ * xhat.omega) - common::skew(xhat.omega) * J_);

  B_.block<3,4>(3,0) = 0.5 / mass_ * rho_ * Va * Va * wing_S_ * C_F_ul(alpha, Va);
  B_.block<3,4>(9,0) = 0.5 * rho_ * Va * Va * wing_S_ * J_inv_ * C_bc<double>() * C_tau_ul(Va);

  // Compute control
  care_solver.solve(P_, A_, B_, Q_, R_);
  K_ = R_inv_ * B_.transpose() * P_;
  uVector u_tilde = -K_ * x_tilde;
  u = u_ref_ - u_tilde;
  u(AIL) = common::saturate(u(AIL), 1.0, -1.0);
  u(ELE) = common::saturate(u(ELE), 1.0, -1.0);
  u(THR) = common::saturate(u(THR), 1.0,  0.0);
  u(RUD) = common::saturate(u(RUD), 1.0, -1.0);
}


} // namespace fixedwing
