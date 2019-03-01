#include "fixed_wing_trim.h"


namespace fixedwing
{


DynamicsCost::DynamicsCost(const std::string &filename)
{
  load(filename);
}


void DynamicsCost::load(const std::string &filename)
{
  common::get_yaml_node("Va_star", filename, Va_star_);
  common::get_yaml_node("R_star", filename, R_star_);
  common::get_yaml_node("gamma_star", filename, gamma_star_);

  common::get_yaml_node("mass", filename, mass_);
  common::get_yaml_node("Jx", filename, Jx_);
  common::get_yaml_node("Jy", filename, Jy_);
  common::get_yaml_node("Jz", filename, Jz_);
  common::get_yaml_node("Jxz", filename, Jxz_);

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

  Gamma_ = Jx_ * Jz_ - Jxz_ * Jxz_;
  Gamma_1_ = (Jxz_ * (Jx_ - Jy_ + Jz_)) / Gamma_;
  Gamma_2_ = (Jz_ * (Jz_ - Jy_) + Jxz_ * Jxz_) / Gamma_;
  Gamma_3_ = Jz_ / Gamma_;
  Gamma_4_ = Jxz_ / Gamma_;
  Gamma_5_ = (Jz_ - Jx_) / Jy_;
  Gamma_6_ = Jxz_ / Jy_;
  Gamma_7_ = ((Jx_ - Jy_) * Jx_ + Jxz_ * Jxz_) / Gamma_;
  Gamma_8_ = Jx_ / Gamma_;

  C_p_0_ = Gamma_3_ * C_el_0_ + Gamma_4_ * C_n_0_;
  C_p_beta_ = Gamma_3_ * C_el_beta_ + Gamma_4_ * C_n_beta_;
  C_p_p_ = Gamma_3_ * C_el_p_ + Gamma_4_ * C_n_p_;
  C_p_r_ = Gamma_3_ * C_el_r_ + Gamma_4_ * C_n_r_;
  C_p_delta_a_ = Gamma_3_ * C_el_delta_a_ + Gamma_4_ * C_n_delta_a_;
  C_p_delta_r_ = Gamma_3_ * C_el_delta_r_ + Gamma_4_ * C_n_delta_r_;
  
  C_r_0_ = Gamma_4_ * C_el_0_ + Gamma_8_ * C_n_0_;
  C_r_beta_ = Gamma_4_ * C_el_beta_ + Gamma_8_ * C_n_beta_;
  C_r_p_ = Gamma_4_ * C_el_p_ + Gamma_8_ * C_n_p_;
  C_r_r_ = Gamma_4_ * C_el_r_ + Gamma_8_ * C_n_r_;
  C_r_delta_a_ = Gamma_4_ * C_el_delta_a_ + Gamma_8_ * C_n_delta_a_;
  C_r_delta_r_ = Gamma_4_ * C_el_delta_r_ + Gamma_8_ * C_n_delta_r_;

  xdot_star_.setZero();
  xdot_star_(H) = Va_star_ * sin(gamma_star_);
  xdot_star_(PSI) = Va_star_ / R_star_ * cos(gamma_star_);
}


} // namespace fixedwing
