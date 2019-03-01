#include "fixed_wing_base.h"



namespace fixedwing
{


void FixedWingBase::load_base(const std::string &filename)
{
  common::get_yaml_node("wing_S", filename, wing_S_);
  common::get_yaml_node("wing_b", filename, wing_b_);
  common::get_yaml_node("wing_M", filename, wing_M_);
  common::get_yaml_node("wing_alpha0", filename, wing_alpha0_);

  common::get_yaml_node("prop_e", filename, prop_e_);

  common::get_yaml_node("C_L_0", filename, C_L_0_);
  common::get_yaml_node("C_L_alpha", filename, C_L_alpha_);
  common::get_yaml_node("C_L_q", filename, C_L_q_);
  common::get_yaml_node("C_L_delta_e", filename, C_L_delta_e_);

  common::get_yaml_node("C_D_p", filename, C_D_p_);
  common::get_yaml_node("C_D_q", filename, C_D_q_);
  common::get_yaml_node("C_D_delta_e", filename, C_D_delta_e_);
}


} // namespace fixedwing
