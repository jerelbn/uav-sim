#include "fixed_wing_trim.h"


namespace fixedwing
{


DynamicsCost::DynamicsCost(const std::string &filename)
{
  load_base(filename);

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
