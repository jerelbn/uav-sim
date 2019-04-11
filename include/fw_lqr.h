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

  // State Indices
  enum
  {
    V = 0, // Velocity
    Q = 3, // Attitude
    W = 7, // Angular rate
    NUM_STATES = 10
  };

  // State Derivative Indices
  enum
  {
    DV = 0,
    DQ = 3,
    DW = 6,
    NUM_DOF = 9
  };

  typedef Matrix<double, NUM_STATES, 1> xVector;
  typedef Matrix<double, NUM_STATES, NUM_STATES> xMatrix;
  typedef Matrix<double, NUM_DOF, 1> dxVector;
  typedef Matrix<double, NUM_DOF, NUM_DOF> dxMatrix;
  typedef Matrix<double, NUM_DOF, COMMAND_SIZE> dxuMatrix;
  typedef Matrix<double, COMMAND_SIZE, NUM_DOF> udxMatrix;

  struct State
  {
    Vector3d v;
    quat::Quatd q;
    Vector3d omega;

    State()
    {
      v.setZero();
      omega.setZero();
    }

    State(const xVector &x0)
    {
      v = x0.segment<3>(V);
      q = quat::Quatd(x0.segment<4>(Q));
      omega = x0.segment<3>(W);
    }

    State operator+(const dxVector &delta) const
    {
      State x;
      x.v = v + delta.segment<3>(DV);
      x.q = q + delta.segment<3>(DQ);
      x.omega = omega + delta.segment<3>(DW);
      return x;
    }

    void operator+=(const dxVector &delta)
    {
      *this = *this + delta;
    }

    dxVector operator-(const State &x) const
    {
      dxVector dx;
      dx.segment<3>(DV) = v - x.v;
      dx.segment<3>(DQ) = q - x.q;
      dx.segment<3>(DW) = omega - x.omega;
      return dx;
    }

    void setZero()
    {
      v.setZero();
      q = quat::Quatd();
      omega.setZero();
    }

    xVector toEigen() const
    {
      xVector x;
      x << v, q.elements(), omega;
      return x;
    }

  };

  LQR();
  void init(const std::string &filename);
  void computeControl(const vehicle::Stated& xhat, const Vector3d &vw, const Vector3d &wp_prev,
                      const Vector3d &wp, vehicle::Stated &xc, uVector &u);

private:

  CareSolver<NUM_DOF,COMMAND_SIZE> care_solver;

  dxMatrix A_, Q_, P_;
  dxuMatrix B_;
  udxMatrix K_;
  uMatrix R_, R_inv_;
  State xc_;

  double p_err_max_;
  double v_err_max_;
  double q_err_max_;
  double omega_err_max_;

  double chi_inf_;
  double gamma_inf_;
  double k_chi_;
  double k_gamma_;

  double max_roll_;
  double max_pitch_;

  Vector3d v_ref_;
  quat::Quatd q_ref_;
  Vector3d omega_ref_;
  uVector u_ref_;
  int update_count_;
  int gain_matrix_update_iters_;

  void computeCommandState(const State &x, const Vector3d &pos, const Vector3d &vw,
                           const Vector3d &wp_prev, const Vector3d &wp, State& xc) const;
  void analyticAB(const vehicle::Stated& x, const Vector3d& vw);
  void numericalAB(const State &x, const State &x_ref, const uVector &u,
                   const Vector3d &pos, const Vector3d &vw, const Vector3d &wp_prev, const Vector3d &wp);
  void f_lqr(const State& x, const uVector& u, const Vector3d& vw, dxVector& dx) const;
  void f_tilde(const State &x_ref, const dxVector &x_tilde, const uVector& u,
               const Vector3d &pos, const Vector3d& vw, const Vector3d& wp_prev,
               const Vector3d &wp, const double& dt, dxVector &x_tilde_dot) const;
};


} // namespace fixedwing
