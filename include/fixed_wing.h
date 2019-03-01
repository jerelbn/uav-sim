#pragma once

#include <fstream>
#include <ceres/ceres.h>

#include "common_cpp/common.h"
#include "fixed_wing_control.h"
#include "fixed_wing_trim.h"
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
  void computeTrim(const std::string& filename) const;

  const vehicle::State<double>& getState() const { return x_; }

  int id_;
  std::string name_;

private:

  void propagate(const double &dt, const uVector& u, const Eigen::Vector3d& vw);
  void f(const vehicle::State<double>& x, const uVector& u, const Vector3d& vw, vehicle::dxVector& dx) const;
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
