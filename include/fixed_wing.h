#pragma once

#include <fstream>
#include <ceres/ceres.h>

#include "common_cpp/common.h"
#include "fixed_wing_control.h"
#include "sensors.h"
#include "environment.h"

using namespace Eigen;


namespace fixedwing
{


class FixedWing
{

public:

  FixedWing();
  FixedWing(const std::string &filename, const environment::Environment& env, const bool &use_random_seed, const int& id);
  ~FixedWing();

  void load(const std::string &filename, const environment::Environment &env, const bool &use_random_seed);
  void run(const double &t, const environment::Environment& env);
  void computeTrim(vehicle::State<double>& x_star, uVector& u_star);

  const vehicle::State<double>& getState() const { return x_; }

  int id_;
  std::string name_;

private:

  void rk4(std::function<void(const vehicle::State<double>&, const uVector&,
                        const Eigen::Vector3d&, vehicle::dxVector&)> func,
           const double& dt, const vehicle::State<double>& x, const uVector& u,
           const Eigen::Vector3d& vw, vehicle::dxVector& dx);
  void propagate(const double &dt, const uVector& u, const Eigen::Vector3d& vw);
  void updateAccels(const uVector& u, const Eigen::Vector3d& vw);
  void getOtherVehicles(const std::vector<Eigen::Vector3d,
                        Eigen::aligned_allocator<Eigen::Vector3d> >& all_vehicle_positions);
  void log(const double &t);

  Controller controller_;
  sensors::Sensors sensors_;

  vehicle::State<double> x_;
  vehicle::dxVector dx_;
  uVector u_;

  bool accurate_integration_, control_using_estimates_;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > other_vehicle_positions_;
  double mass_, t_prev_;
  double Va_star_, R_star_, gamma_star_;
  Eigen::Matrix3d J_, J_inv_;
  double Jx_, Jy_, Jz_, Jxz_;
  double rho_;
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

  std::ofstream state_log_;
  std::ofstream command_log_;

  // Functions of angle of attack and side slip angle
  template<typename T>
  T sigma(const T& alpha)
  {
    return (1.0 + exp(-wing_M_ * (alpha - wing_alpha0_)) + exp(wing_M_ * (alpha + wing_alpha0_))) /
           ((1.0 + exp(-wing_M_ * (alpha - wing_alpha0_))) * (1.0 + exp(wing_M_ * (alpha + wing_alpha0_))));
  }


  template<typename T>
  T C_L(const T& alpha)
  {
    return (1.0 - sigma(alpha)) * (C_L_0_ + C_L_alpha_ * alpha) +
           sigma(alpha) * (2.0 * common::sign(alpha) * sin(alpha) * sin(alpha) * cos(alpha));
  }


  template<typename T>
  T C_D(const T& alpha)
  {
    return C_D_p_ + wing_S_ * (C_L_0_ + C_L_alpha_ * alpha) * (C_L_0_ + C_L_alpha_ * alpha) /
           (M_PI * prop_e_ * wing_b_ * wing_b_);
  }


  template<typename T>
  T C_X(const T& alpha)
  {
    return -C_D(alpha) * cos(alpha) + C_L(alpha) * sin(alpha);
  }


  template<typename T>
  T C_X_q(const T& alpha)
  {
    return -C_D_q_ * cos(alpha) + C_L_q_ * sin(alpha);
  }


  template<typename T>
  T C_X_delta_e(const T& alpha)
  {
    return -C_D_delta_e_ * cos(alpha) + C_L_delta_e_ * sin(alpha);
  }


  template<typename T>
  T C_Z(const T& alpha)
  {
    return -C_D(alpha) * sin(alpha) - C_L(alpha) * cos(alpha);
  }


  template<typename T>
  T C_Z_q(const T& alpha)
  {
    return -C_D_q_ * sin(alpha) - C_L_q_ * cos(alpha);
  }


  template<typename T>
  T C_Z_delta_e(const T& alpha)
  {
    return -C_D_delta_e_ * sin(alpha) - C_L_delta_e_ * cos(alpha);
  }

public:

  template<typename T>
  void f(const vehicle::State<T>& x, const Matrix<T,COMMAND_SIZE,1>& u,
         const Matrix<T,3,1>& vw, Matrix<T,vehicle::NUM_DOF,1>& dx)
  {

    Matrix<T,3,1> v_r = x.v - x.q.rotp(vw); // velocity w.r.t. air in body frame
    T Va = v_r.norm();
    T alpha = atan2(v_r(2), v_r(0));
    T beta = asin(v_r(1) / Va);

    Matrix<T,3,1> C_F_alpha_beta(C_X(alpha), C_Y_0_ + C_Y_beta_ * beta, C_Z(alpha));
    Matrix<T,3,3> C_F_omega = Matrix<T,3,3>::Zero();
    C_F_omega(0,1) = C_X_q(alpha) * wing_c_;
    C_F_omega(1,0) = (T)C_Y_p_ * wing_b_;
    C_F_omega(1,2) = (T)C_Y_r_ * wing_b_;
    C_F_omega(2,1) = C_Z_q(alpha) * wing_c_;
    Matrix<T,3,4> C_F_u = Matrix<T,3,4>::Zero();
    C_F_u(0,1) = C_X_delta_e(alpha) * delta_e_max_;
    C_F_u(1,0) = (T)C_Y_delta_a_ * delta_a_max_;
    C_F_u(1,3) = (T)C_Y_delta_r_ * delta_r_max_;
    C_F_u(2,1) = C_Z_delta_e(alpha) * delta_e_max_;
    Matrix<T,3,3> C_bc = Matrix<T,3,1>((T)wing_b_, (T)wing_c_, (T)wing_b_).asDiagonal();
    Matrix<T,3,1> C_tau_alpha_beta(C_el_0_ + C_el_beta_ * beta,
                                   C_m_0_ + C_m_alpha_ * alpha,
                                   C_n_0_ + C_n_beta_ * beta);
    Matrix<T,3,3> C_tau_omega = Matrix<T,3,3>::Zero();
    C_tau_omega(0,0) = (T)C_el_p_ * wing_b_;
    C_tau_omega(0,2) = (T)C_el_r_ * wing_b_;
    C_tau_omega(1,1) = (T)C_m_q_ * wing_c_;
    C_tau_omega(2,0) = (T)C_n_p_ * wing_b_;
    C_tau_omega(2,2) = (T)C_n_r_ * wing_b_;
    Matrix<T,3,4> C_tau_u = Matrix<T,3,4>::Zero();
    C_tau_u(0,0) = (T)C_el_delta_a_ * delta_a_max_;
    C_tau_u(0,3) = (T)C_el_delta_r_ * delta_r_max_;
    C_tau_u(1,1) = (T)C_m_delta_e_ * delta_e_max_;
    C_tau_u(2,0) = (T)C_n_delta_a_ * delta_a_max_;
    C_tau_u(2,3) = (T)C_n_delta_r_ * delta_r_max_;

    Matrix<T,3,1> f_b = mass_ * common::gravity * x.q.rotp(common::e3) + 0.5 * rho_ * Va * Va * wing_S_ *
                   (C_F_alpha_beta + 1.0 / (2.0 * Va) * C_F_omega * x.omega + C_F_u * u) +
                   rho_ * prop_S_ * prop_C_ * (Va + u(THR) * (k_motor_ - Va)) * u(THR) * (k_motor_ - Va) * common::e1;
    Matrix<T,3,1> tau_b = 0.5 * rho_ * Va * Va * wing_S_ * C_bc *
                     (C_tau_alpha_beta + 1.0 / (2.0 * Va) * C_tau_omega * x.omega + C_tau_u * u) -
                     k_T_p_ * k_Omega_ * k_Omega_ * u(THR) * u(THR) * common::e1;

    dx.template segment<3>(vehicle::DP) = x.q.rota(x.v);
    dx.template segment<3>(vehicle::DV) = 1.0 / mass_ * f_b - x.omega.cross(x.v);
    dx.template segment<3>(vehicle::DQ) = x.omega;
    dx.template segment<3>(vehicle::DW) = J_inv_ * (tau_b - x.omega.cross(J_ * x.omega));
  }


  template<typename T>
  void computeTrimmedStateAndCommand(const T& alpha, const T& beta, const T& phi,
                                     const T& Va, const T& R, const T& gamma,
                                     vehicle::State<T>& x, Matrix<T,COMMAND_SIZE,1>& u)
  {
    // Trimmed state
    x.setZero();
    T theta = alpha + gamma;
    x.v(0) = Va * cos(alpha) * cos(beta);
    x.v(1) = Va * sin(beta);
    x.v(2) = Va * sin(alpha) * cos(beta);
    x.q = quat::Quat<T>(phi, theta, T(0));
    x.omega(0) = -Va / R * sin(theta);
    x.omega(1) = Va / R * sin(phi) * cos(theta);
    x.omega(2) = Va / R * cos(phi) * cos(theta);

    // Trimmed command
    u.setZero();
    u(ELE) = (2.0 * (Jxz_ * (x.omega(0) * x.omega(0) - x.omega(2) * x.omega(2)) + (Jx_ - Jz_) * x.omega(0) * x.omega(2)) /
             (rho_ * Va * Va * wing_c_ * wing_S_) - C_m_0_ - C_m_alpha_ * alpha -
             C_m_q_ * wing_c_ * x.omega(1) / (2.0 * Va)) / C_m_delta_e_;
    u(THR) = 1.0 / (2.0 * (k_motor_ - Va)) * (sqrt(Va * Va + (4.0 * mass_ * (x.omega(1) * x.v(2) -
             x.omega(2) * x.v(1) + common::gravity * sin(theta)) - 2.0 * rho_ * Va * Va * wing_S_ *
             (C_X(alpha) + C_X_q(alpha) * wing_c_ * x.omega(1) / (2.0 * Va) + C_X_delta_e(alpha) * u(ELE))) /
             (rho_ * prop_S_ * prop_C_)) - Va);

    T Gamma = (T)Jx_ * Jz_ - Jxz_ * Jxz_;
    T Gamma_1 = (Jxz_ * (Jx_ - Jy_ + Jz_)) / Gamma;
    T Gamma_2 = (Jz_ * (Jz_ - Jy_) + Jxz_ * Jxz_) / Gamma;
    T Gamma_3 = Jz_ / Gamma;
    T Gamma_4 = Jxz_ / Gamma;
//    T Gamma_5 = (Jz_ - Jx_) / Jy_;
//    T Gamma_6 = Jxz_ / Jy_;
    T Gamma_7 = ((Jx_ - Jy_) * Jx_ + Jxz_ * Jxz_) / Gamma;
    T Gamma_8 = Jx_ / Gamma;

    T C_p_0 = Gamma_3 * C_el_0_ + Gamma_4 * C_n_0_;
    T C_p_beta = Gamma_3 * C_el_beta_ + Gamma_4 * C_n_beta_;
    T C_p_p = Gamma_3 * C_el_p_ + Gamma_4 * C_n_p_;
    T C_p_r = Gamma_3 * C_el_r_ + Gamma_4 * C_n_r_;
    T C_p_delta_a = Gamma_3 * C_el_delta_a_ + Gamma_4 * C_n_delta_a_;
    T C_p_delta_r = Gamma_3 * C_el_delta_r_ + Gamma_4 * C_n_delta_r_;

    T C_r_0 = Gamma_4 * C_el_0_ + Gamma_8 * C_n_0_;
    T C_r_beta = Gamma_4 * C_el_beta_ + Gamma_8 * C_n_beta_;
    T C_r_p = Gamma_4 * C_el_p_ + Gamma_8 * C_n_p_;
    T C_r_r = Gamma_4 * C_el_r_ + Gamma_8 * C_n_r_;
    T C_r_delta_a = Gamma_4 * C_el_delta_a_ + Gamma_8 * C_n_delta_a_;
    T C_r_delta_r = Gamma_4 * C_el_delta_r_ + Gamma_8 * C_n_delta_r_;

    Matrix<T,2,2> C;
    C(0,0) = C_p_delta_a;
    C(0,1) = C_p_delta_r;
    C(1,0) = C_r_delta_a;
    C(1,1) = C_r_delta_r;
    Matrix<T,2,1> c;
    c(0) = (-Gamma_1 * x.omega(0) * x.omega(1) + Gamma_2 * x.omega(1) * x.omega(2)) / (0.5 * rho_ * Va * Va * wing_S_ * wing_b_) -
           C_p_0 - C_p_beta * beta - C_p_p * wing_b_ * x.omega(0) / (2.0 * Va) - C_p_r * wing_b_ * x.omega(2) / (2.0 * Va);
    c(1) = (-Gamma_7 * x.omega(0) * x.omega(1) + Gamma_1 * x.omega(1) * x.omega(2)) / (0.5 * rho_ * Va * Va * wing_S_ * wing_b_) -
        C_r_0 - C_r_beta * beta - C_r_p * wing_b_ * x.omega(0) / (2.0 * Va) - C_r_r * wing_b_ * x.omega(2) / (2.0 * Va);
    Matrix<T,2,1> delta = C.inverse() * c;
    u(AIL) = delta(0);
    u(RUD) = delta(1);
  }

};


// Cost functor to minimize when computing trim
class DynamicsCost
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DynamicsCost(const vehicle::dxVector& xdot_star, const double& Va_star,
               const double& R_star, const double& gamma_star, FixedWing& fw)
               : xdot_star_(xdot_star), Va_star_(Va_star), R_star_(R_star), gamma_star_(gamma_star), fw_(fw)
               {}

  template<typename T>
  bool operator()(const T* const alpha, const T* const beta, const T* const phi, T* residuals) const
  {
    // Copy/map states and residual error
    Map<Matrix<T,vehicle::NUM_DOF,1>> r(residuals);

    // Compute trimmed state and command
    vehicle::State<T> x_star;
    Matrix<T,COMMAND_SIZE,1> u_star;
    fw_.computeTrimmedStateAndCommand(*alpha, *beta, *phi, (T)Va_star_, (T)R_star_, (T)gamma_star_, x_star, u_star);

    // Compute dynamics
    static const Matrix<T,3,1> vw(T(0), T(0), T(0));
    Matrix<T,vehicle::NUM_DOF,1> f_star;
    fw_.f(x_star, u_star, vw, f_star);

    // Compute residual error
    r = xdot_star_ - f_star;

    return true;
  }

  const vehicle::dxVector xdot_star_;
  const double Va_star_, R_star_, gamma_star_;
  FixedWing& fw_;

};
typedef ceres::AutoDiffCostFunction<DynamicsCost, vehicle::NUM_DOF, 1, 1, 1> DynamicsCostFactor;


} // namespace fixedwing
