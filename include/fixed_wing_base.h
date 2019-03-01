#pragma once

#include "common_cpp/common.h"


namespace fixedwing
{


class FixedWingBase
{
protected:

  void load_base(const std::string& filename);

  double wing_S_, wing_b_, wing_M_, wing_alpha0_;
  double prop_e_;
  double C_L_0_, C_L_alpha_, C_L_q_, C_L_delta_e_;
  double C_D_p_, C_D_q_, C_D_delta_e_;

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
