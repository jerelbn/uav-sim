#pragma once

#include <Eigen/Dense>
#include "common_cpp/common.h"
#include "vehicle.h"

using namespace Eigen;


namespace fixedwing
{


class FixedWingBase
{
protected:

  void load_base(const std::string& filename);

  double origin_alt_; // altitude above sea level at flight location
  double origin_temp_; // temperature at flight location (degrees Fahrenheit)
  double rho_; // air density at flight location (computed from altitude and temperature)

  double mass_;
  Eigen::Matrix3d J_, J_inv_;
  double Jx_, Jy_, Jz_, Jxz_;
  double wing_S_, wing_b_, wing_c_, wing_M_, wing_epsilon_, wing_alpha0_;
  double k_motor_, k_T_p_, k_Omega_;
  double prop_e_, prop_S_, prop_C_;
  double C_L_0_, C_L_alpha_, C_L_beta_, C_L_p_, C_L_q_, C_L_r_, C_L_delta_a_, C_L_delta_e_, C_L_delta_r_;
  double C_D_0_, C_D_alpha_, C_D_beta_, C_D_p_, C_D_q_, C_D_r_, C_D_delta_a_, C_D_delta_e_, C_D_delta_r_;
  double C_el_0_, C_el_alpha_, C_el_beta_, C_el_p_, C_el_q_, C_el_r_, C_el_delta_a_, C_el_delta_e_, C_el_delta_r_;
  double C_m_0_, C_m_alpha_, C_m_beta_, C_m_p_, C_m_q_, C_m_r_, C_m_delta_a_, C_m_delta_e_, C_m_delta_r_;
  double C_n_0_, C_n_alpha_, C_n_beta_, C_n_p_, C_n_q_, C_n_r_, C_n_delta_a_, C_n_delta_e_, C_n_delta_r_;
  double C_Y_0_, C_Y_alpha_, C_Y_beta_, C_Y_p_, C_Y_q_, C_Y_r_, C_Y_delta_a_, C_Y_delta_e_, C_Y_delta_r_;
  double delta_a_max_, delta_e_max_, delta_r_max_;
  double Va_star_, R_star_, gamma_star_;
  double C_F_t_, C_tau_t_;

  template<typename T>
  void f(const vehicle::State<T>& x, const Matrix<T,COMMAND_SIZE,1>& u,
         const Matrix<T,3,1>& vw, Matrix<T,vehicle::NUM_DOF,1>& dx) const
  {
    Matrix<T,3,1> v_r = x.v - x.q.rotp(vw); // velocity w.r.t. air in body frame
    T Va = v_r.norm();
    T alpha = atan(v_r(2) / v_r(0));
    T beta = asin(v_r(1) / Va);

    Matrix<T,3,1> f_b = mass_ * common::gravity * x.q.rotp(common::e3) + 0.5 * rho_ * Va * Va * wing_S_ *
                        (C_F_alpha_beta(alpha,beta) + 1.0 / (2.0 * Va) * C_F_omega(alpha) * x.omega + C_F_u(alpha) * u) +
                        rho_ * prop_S_ * prop_C_ * (Va + u(THR) * (k_motor_ - Va)) * u(THR) * (k_motor_ - Va) * common::e1;
    Matrix<T,3,1> tau_b = 0.5 * rho_ * Va * Va * wing_S_ * C_bc<double>() *
                          (C_tau_alpha_beta(alpha,beta) + 1.0 / (2.0 * Va) * C_tau_omega<double>() * x.omega + C_tau_u<double>() * u) -
                          k_T_p_ * k_Omega_ * k_Omega_ * u(THR) * u(THR) * common::e1;

    dx.template segment<3>(vehicle::DP) = x.q.rota(x.v);
    dx.template segment<3>(vehicle::DV) = 1.0 / mass_ * f_b - x.omega.cross(x.v);
    dx.template segment<3>(vehicle::DQ) = x.omega;
    dx.template segment<3>(vehicle::DW) = J_inv_ * (tau_b - x.omega.cross(J_ * x.omega));
  }

  template<typename T>
  inline Matrix<T,3,4> C_F_ul(const T& alpha, const T& Va) const
  {
    Matrix<T,3,4> M = Matrix<T,3,4>::Zero();
    M(0,1) = C_X_delta_e(alpha) * delta_e_max_;
    M(0,2) = 2.0 * C_F_t_ / (rho_ * Va * Va * wing_S_);
    M(1,0) = C_Y_delta_a_ * delta_a_max_;
    M(1,3) = C_Y_delta_r_ * delta_r_max_;
    M(2,1) = C_Z_delta_e(alpha) * delta_e_max_;
    return M;
  }

  template<typename T>
  inline Matrix<T,3,4> C_tau_ul(const T& Va) const
  {
    Matrix<T,3,4> M = Matrix<T,3,4>::Zero();
    M(0,0) = C_el_delta_a_ * delta_a_max_;
    M(0,2) = -2.0 * C_tau_t_ / (rho_ * Va * Va * wing_S_ * wing_b_);
    M(0,3) = C_el_delta_r_ * delta_r_max_;
    M(1,1) = C_m_delta_e_ * delta_e_max_;
    M(2,0) = C_n_delta_a_ * delta_a_max_;
    M(2,3) = C_n_delta_r_ * delta_r_max_;
    return M;
  }

  template<typename T>
  inline Matrix<T,3,1> C_F_alpha_beta(const T& alpha, const T& beta) const
  {
    return Matrix<T,3,1>(C_X(alpha), C_Y_0_ + C_Y_beta_ * beta, C_Z(alpha));
  }

  template<typename T>
  inline Matrix<T,3,3> C_F_omega(const T& alpha) const
  {
    Matrix<T,3,3> M = Matrix<T,3,3>::Zero();
    M(0,1) = C_X_q(alpha) * wing_c_;
    M(1,0) = C_Y_p_ * wing_b_;
    M(1,2) = C_Y_r_ * wing_b_;
    M(2,1) = C_Z_q(alpha) * wing_c_;
    return M;
  }

  template<typename T>
  inline Matrix<T,3,4> C_F_u(const T& alpha) const
  {
    Matrix<T,3,4> M = Matrix<T,3,4>::Zero();
    M(0,1) = C_X_delta_e(alpha) * delta_e_max_;
    M(1,0) = C_Y_delta_a_ * delta_a_max_;
    M(1,3) = C_Y_delta_r_ * delta_r_max_;
    M(2,1) = C_Z_delta_e(alpha) * delta_e_max_;
    return M;
  }

  template<typename T>
  inline Matrix<T,3,3> C_bc() const
  {
    Matrix<T,3,3> M = Matrix<T,3,3>::Zero();
    M(0,0) = wing_b_;
    M(1,1) = wing_c_;
    M(2,2) = wing_b_;
    return M;
  }

  template<typename T>
  inline Matrix<T,3,1> C_tau_alpha_beta(const T& alpha, const T& beta) const
  {
    return Matrix<T,3,1>(C_el_0_ + C_el_beta_ * beta,
                         C_m_0_ + C_m_alpha_ * alpha,
                         C_n_0_ + C_n_beta_ * beta);
  }

  template<typename T>
  inline Matrix<T,3,3> C_tau_omega() const
  {
    Matrix<T,3,3> M = Matrix<T,3,3>::Zero();
    M(0,0) = C_el_p_ * wing_b_;
    M(0,2) = C_el_r_ * wing_b_;
    M(1,1) = C_m_q_ * wing_c_;
    M(2,0) = C_n_p_ * wing_b_;
    M(2,2) = C_n_r_ * wing_b_;
    return M;
  }

  template<typename T>
  inline Matrix<T,3,4> C_tau_u() const
  {
    Matrix<T,3,4> M = Matrix<T,3,4>::Zero();
    M(0,0) = C_el_delta_a_ * delta_a_max_;
    M(0,3) = C_el_delta_r_ * delta_r_max_;
    M(1,1) = C_m_delta_e_ * delta_e_max_;
    M(2,0) = C_n_delta_a_ * delta_a_max_;
    M(2,3) = C_n_delta_r_ * delta_r_max_;
    return M;
  }

  template<typename T>
  inline T sigma(const T& alpha) const
  {
    return (1.0 + exp(-wing_M_ * (alpha - wing_alpha0_)) + exp(wing_M_ * (alpha + wing_alpha0_))) /
           ((1.0 + exp(-wing_M_ * (alpha - wing_alpha0_))) * (1.0 + exp(wing_M_ * (alpha + wing_alpha0_))));
  }


  template<typename T>
  inline T C_L(const T& alpha) const
  {
    return (1.0 - sigma(alpha)) * (C_L_0_ + C_L_alpha_ * alpha) +
           sigma(alpha) * (2.0 * common::sign(alpha) * sin(alpha) * sin(alpha) * cos(alpha));
  }


  template<typename T>
  inline T C_D(const T& alpha) const
  {
    return C_D_p_ + wing_S_ * (C_L_0_ + C_L_alpha_ * alpha) * (C_L_0_ + C_L_alpha_ * alpha) /
           (M_PI * prop_e_ * wing_b_ * wing_b_);
  }


  template<typename T>
  inline T C_X(const T& alpha) const
  {
    return -C_D(alpha) * cos(alpha) + C_L(alpha) * sin(alpha);
  }


  template<typename T>
  inline T C_X_q(const T& alpha) const
  {
    return -C_D_q_ * cos(alpha) + C_L_q_ * sin(alpha);
  }


  template<typename T>
  inline T C_X_delta_e(const T& alpha) const
  {
    return -C_D_delta_e_ * cos(alpha) + C_L_delta_e_ * sin(alpha);
  }


  template<typename T>
  inline T C_Z(const T& alpha) const
  {
    return -C_D(alpha) * sin(alpha) - C_L(alpha) * cos(alpha);
  }


  template<typename T>
  inline T C_Z_q(const T& alpha) const
  {
    return -C_D_q_ * sin(alpha) - C_L_q_ * cos(alpha);
  }


  template<typename T>
  inline T C_Z_delta_e(const T& alpha) const
  {
    return -C_D_delta_e_ * sin(alpha) - C_L_delta_e_ * cos(alpha);
  }

};


} // namespace fixedwing
