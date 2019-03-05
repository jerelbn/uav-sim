// LQR Control of a fixed wing for waypoint following
#pragma once

#include "common_cpp/common.h"
#include "vehicle.h"
#include "lin_alg_tools/care.h"
#include "geometry/quat.h"
#include "fixed_wing_base.h"

namespace fixedwing
{


class LQR : public FixedWingBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  LQR();
  void init(const std::string &filename);
  void computeControl(const vehicle::Stated& xhat, const Vector3d &vw, vehicle::Stated& xc, uVector &u);

private:

  CareSolver<12,4> care_solver;

  Matrix<double,12,12> A_, Q_, P_;
  Matrix<double,12,4> B_;
  Matrix<double,4,12> K_;
  Matrix<double,4,4> R_, R_inv_;

  double p_err_max_;
  double v_err_max_;
  double q_err_max_;
  double omega_err_max_;

  Vector3d v_ref_;
  quat::Quatd q_ref_;
  Vector3d omega_ref_;
  uVector u_ref_, u_prev_;
  int update_count_;
  int gain_matrix_update_iters_;

  void computeCommandState(const vehicle::Stated& x, vehicle::Stated& xc) const;
  void analyticAB(const vehicle::Stated& x, const Vector3d& vw);
  void numericalAB(const vehicle::Stated &x, const vehicle::Stated &x_ref,
                    const uVector &u, const Vector3d &vw);
  void f_tilde(const vehicle::Stated& x_ref, const vehicle::dxVector &x_tilde,
               const uVector &u, const Vector3d &vw, const double &dt, vehicle::dxVector& x_tilde_dot) const;
};


} // namespace fixedwing
