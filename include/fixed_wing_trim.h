#pragma once

#include <fstream>
#include <ceres/ceres.h>

#include "common_cpp/common.h"
#include "vehicle.h"
#include "fixed_wing_base.h"

using namespace Eigen;



namespace fixedwing
{


enum
{
  PN, PE, H,
  U, V, W,
  PHI, THETA, PSI,
  P, Q, R,
  TRIM_SIZE
};
typedef Matrix<double,TRIM_SIZE,1> TrimState;


// Cost functor to minimize when computing trim
class DynamicsCost : public FixedWingBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DynamicsCost(const std::string& filename);
  void load(const std::string& filename);

  TrimState xdot_star_;
  double Va_star_, R_star_, gamma_star_;
  double mass_;
  double Jx_, Jy_, Jz_, Jxz_;
  double rho_;
  double wing_c_, wing_epsilon_;
  double k_motor_, k_T_p_, k_Omega_;
  double prop_e_, prop_S_, prop_C_;
  double C_L_beta_, C_L_p_, C_L_r_, C_L_delta_a_, C_L_delta_r_;
  double C_D_0_, C_D_alpha_, C_D_beta_, C_D_r_, C_D_delta_a_, C_D_delta_r_;
  double C_el_0_, C_el_alpha_, C_el_beta_, C_el_p_, C_el_q_, C_el_r_, C_el_delta_a_, C_el_delta_e_, C_el_delta_r_;
  double C_m_0_, C_m_alpha_, C_m_beta_, C_m_p_, C_m_q_, C_m_r_, C_m_delta_a_, C_m_delta_e_, C_m_delta_r_;
  double C_n_0_, C_n_alpha_, C_n_beta_, C_n_p_, C_n_q_, C_n_r_, C_n_delta_a_, C_n_delta_e_, C_n_delta_r_;
  double C_Y_0_, C_Y_alpha_, C_Y_beta_, C_Y_p_, C_Y_q_, C_Y_r_, C_Y_delta_a_, C_Y_delta_e_, C_Y_delta_r_;
  double Gamma_, Gamma_1_, Gamma_2_, Gamma_3_, Gamma_4_, Gamma_5_, Gamma_6_, Gamma_7_, Gamma_8_;
  double C_p_0_, C_p_beta_, C_p_p_, C_p_r_, C_p_delta_a_, C_p_delta_r_;
  double C_r_0_, C_r_beta_, C_r_p_, C_r_r_, C_r_delta_a_, C_r_delta_r_;
  double delta_a_max_, delta_e_max_, delta_r_max_;

  template<typename T>
  bool operator()(const T* const alpha, const T* const beta, const T* const phi, T* residuals) const
  {
    // Copy/map inputs and create variables
    Matrix<T,TRIM_SIZE,1> x_star, f_of_x_u_star;
    Map<Matrix<T,TRIM_SIZE,1>> r(residuals);
    Matrix<T,COMMAND_SIZE,1> u_star;

    // Compute trimmed states/commands, trimmed dynamics, and error residual
    computeTrimmedStateAndCommand(*alpha, *beta, *phi, x_star, u_star);
    f(x_star, u_star, f_of_x_u_star);
    r = xdot_star_ - f_of_x_u_star;

    return true;
  }

  template<typename T>
  void f(const Matrix<T,TRIM_SIZE,1>& x, const Matrix<T,COMMAND_SIZE,1>& cmd, Matrix<T,TRIM_SIZE,1>& dx) const
  {
    // Unpack states and commands for readability
    T u = x(U);
    T v = x(V);
    T w = x(W);
    T phi = x(PHI);
    T theta = x(THETA);
    T psi = x(PSI);
    T p = x(P);
    T q = x(Q);
    T r = x(R);

    T delta_a = cmd(AIL);
    T delta_e = cmd(ELE);
    T delta_t = cmd(THR);
    T delta_r = cmd(RUD);

    T g = (T)common::gravity;

    // Trim dynamics assume no wind
    T Va = x.template segment<3>(U).norm();
    T alpha = atan2(w, u);
    T beta = asin(v / Va);

    // Fill up derivatives
    dx(PN) = (cos(theta) * cos(psi)) * u + (sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi)) * v + (cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi)) * w;
    dx(PE) = (cos(theta) * sin(psi)) * u + (sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi)) * v + (cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi)) * w;
    dx(H) = u * sin(theta) - v * sin(phi) * cos(theta) - w * cos(phi) * cos(theta);
    dx(U) = r * v - q * w - g * sin(theta) + 0.5 * rho_ * Va * Va * wing_S_ / mass_ * (C_X(alpha) + 0.5 * C_X_q(alpha) * wing_c_ * q / Va + C_X_delta_e(alpha) * delta_e) + rho_ * prop_S_ * prop_C_ / mass_ * (Va + delta_t * (k_motor_ - Va)) * delta_t * (k_motor_ - Va);
    dx(V) = p * w - r * u + g * cos(theta) * sin(phi) + 0.5 * rho_ * Va * Va * wing_S_ / mass_ * (C_Y_0_ + C_Y_beta_ * beta + 0.5 * wing_b_ / Va *(C_Y_p_ * p + C_Y_r_ * r) + C_Y_delta_a_ * delta_a + C_Y_delta_r_ * delta_r);
    dx(W) = q * u - p * v + g * cos(theta) * cos(phi) + 0.5 * rho_ * Va * Va * wing_S_ / mass_ * (C_Z(alpha) + 0.5 * C_Z_q(alpha) * wing_c_ * q / Va + C_Z_delta_e(alpha) * delta_e);
    dx(PHI) = p + (q * sin(phi) + r * cos(phi)) * tan(theta);
    dx(THETA) = q * cos(phi) - r * sin(phi);
    dx(PSI) = (q * sin(phi) + r * cos(phi)) / cos(theta);
    dx(P) = Gamma_1_ * p * q - Gamma_2_ * q * r + 0.5 * rho_ * Va * Va * wing_S_ * wing_b_ * (C_p_0_ + C_p_beta_ * beta + 0.5 * wing_b_ / Va * (C_p_p_ * p + C_p_r_ * r) + C_p_delta_a_ * delta_a + C_p_delta_r_ * delta_r) - Gamma_3_ * k_T_p_ * k_Omega_ * k_Omega_ * delta_t * delta_t;
    dx(Q) = Gamma_5_ * p * r - Gamma_6_ * (p * p - r * r) + 0.5 * rho_ * Va * Va * wing_S_ * wing_c_ / Jy_ * (C_m_0_ + C_m_alpha_ * alpha + 0.5 * C_m_q_ * wing_c_ * q / Va + C_m_delta_e_ * delta_e);
    dx(R) = Gamma_7_ * p * q - Gamma_1_ * q * r + 0.5 * rho_ * Va * Va * wing_S_ * wing_b_ * (C_r_0_ + C_r_beta_ * beta + 0.5 * wing_b_ / Va * (C_r_p_ * p + C_r_r_ * r) + C_r_delta_a_ * delta_a + C_r_delta_r_ * delta_r);
  }

  template<typename T>
  void computeTrimmedStateAndCommand(const T& alpha, const T& beta, const T& phi, Matrix<T,TRIM_SIZE,1>& x, Matrix<T,COMMAND_SIZE,1>& u) const
  {
    // Trimmed states
    x.setZero();
    T theta = alpha + gamma_star_;
    x(U) = Va_star_ * cos(alpha) * cos(beta);
    x(V) = Va_star_ * sin(beta);
    x(W) = Va_star_ * sin(alpha) * cos(beta);
    x(PHI) = phi;
    x(THETA) = theta;
    x(P) = -Va_star_ / R_star_ * sin(theta);
    x(Q) = Va_star_ / R_star_ * sin(phi) * cos(theta);
    x(R) = Va_star_ / R_star_ * cos(phi) * cos(theta);

    // Trimmed commands
    u(ELE) = (2.0 * (Jxz_ * (x(P) * x(P) - x(R) * x(R)) + (Jx_ - Jz_) * x(P) * x(R)) / (rho_ * Va_star_ * Va_star_ * wing_c_ * wing_S_) - C_m_0_ - C_m_alpha_ * alpha - C_m_q_ * wing_c_ * x(Q) / (2.0 * Va_star_)) / C_m_delta_e_;
    u(THR) = 1.0 / (2.0 * (k_motor_ - Va_star_)) * (sqrt(Va_star_ * Va_star_ + (4.0 * mass_ * (x(Q) * x(W) - x(R) * x(V) + common::gravity * sin(theta)) - 2.0 * rho_ * Va_star_ * Va_star_ * wing_S_ * (C_X(alpha) + C_X_q(alpha) * wing_c_ * x(Q) / (2.0 * Va_star_) + C_X_delta_e(alpha) * u(ELE))) / (rho_ * prop_S_ * prop_C_)) - Va_star_);

    Matrix<T,2,2> C;
    C(0,0) = (T)C_p_delta_a_;
    C(0,1) = (T)C_p_delta_r_;
    C(1,0) = (T)C_r_delta_a_;
    C(1,1) = (T)C_r_delta_r_;
    Matrix<T,2,1> c;
    c(0) = (-Gamma_1_ * x(P) * x(Q) + Gamma_2_ * x(Q) * x(R)) / (0.5 * rho_ * Va_star_ * Va_star_ * wing_S_ * wing_b_) - C_p_0_ - C_p_beta_ * beta - C_p_p_ * wing_b_ * x(P) / (2.0 * Va_star_) - C_p_r_ * wing_b_ * x(R) / (2.0 * Va_star_);
    c(1) = (-Gamma_7_ * x(P) * x(Q) + Gamma_1_ * x(Q) * x(R)) / (0.5 * rho_ * Va_star_ * Va_star_ * wing_S_ * wing_b_) - C_r_0_ - C_r_beta_ * beta - C_r_p_ * wing_b_ * x(P) / (2.0 * Va_star_) - C_r_r_ * wing_b_ * x(R) / (2.0 * Va_star_);
    Matrix<T,2,1> delta = C.inverse() * c;
    u(AIL) = delta(0);
    u(RUD) = delta(1);
  }

};
typedef ceres::AutoDiffCostFunction<DynamicsCost, TRIM_SIZE, 1, 1, 1> DynamicsCostFactor;


} // namespace fixedwing
