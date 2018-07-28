#include "ekf/ekf.h"


namespace ekf
{


State::State()
{
  p.setZero();
  q = common::Quaternion();
  v.setZero();
  bg.setZero();
  ba.setZero();
}


State::State(const xVector &x)
{
  p = x.segment<3>(PX);
  q = common::Quaternion(x.segment<4>(QW));
  v = x.segment<3>(VX);
  bg = x.segment<3>(GX);
  ba = x.segment<3>(AX);
}


State State::operator+(const dxVector &delta)
{
  State x;
  x.p = p + delta.segment<3>(DPX);
  x.q = q + delta.segment<3>(DQX);
  x.v = v + delta.segment<3>(DVX);
  x.bg = bg + delta.segment<3>(DGX);
  x.ba = ba + delta.segment<3>(DAX);
  return x;
}


State State::operator+=(const dxVector &delta)
{
  State x;
  return x + delta;
}


void EKF::load(const std::string filename)
{

}


void EKF::propagate(const double t, const uVector&u)
{
  // Time step
  double dt = t - t_prev_;
  t_prev_ = t;

  // Propagate the state
  f(xdot_, x_, u);
  x_ += xdot_ * dt;

  // Propagate the covariance
  getF(F_, x_, u);
  getG(G_, x_);
  P_ += (F_ * P_ + P_ * F_.transpose() + G_ * Qu_ * G_.transpose() + Qx_) * dt;
}


void EKF::imageUpdate(const Eigen::MatrixXd &pts, const Eigen::Matrix<double, 5, 5> &R)
{
  // Match current landmarks to keyframe landmarks


  // Estimate camera pose relative to keyframe via nonlinear optimization
}

void EKF::f(dxVector &xdot, const State &x, const uVector &u)
{
  xdot.segment<3>(DPX) = x.q.inv().rot(x.v);
  xdot.segment<3>(DQX) = u.segment<3>(UWX) - x.bg;
  xdot.segment<3>(DVX) = u.segment<3>(UAX) - x.ba + common::gravity * x.q.rot(common::e3) -
                       Eigen::Vector3d(u.segment<3>(UWX)-x.bg).cross(x.v);
  xdot.segment<3>(DGX).setZero();
  xdot.segment<3>(DAX).setZero();
}


void EKF::getF(dxMatrix &F, const State &x, const uVector &u)
{
  F.block<3,3>(DPX,DQX) = -x.q.R().transpose() * common::skew(x.v);
  F.block<3,3>(DPX,DVX) = x.q.R().transpose();
  F.block<3,3>(DQX,DGX) = -common::I_3x3;
  F.block<3,3>(DVX,DQX) = common::gravity * common::skew(x.q.rot(common::e3));
  F.block<3,3>(DVX,DVX) = -common::skew(Eigen::Vector3d(u.segment<3>(UWX)-x.bg));
  F.block<3,3>(DVX,DGX) = -common::skew(x.v);
  F.block<3,3>(DVX,DAX) = -common::I_3x3;
}


void EKF::getG(Eigen::Matrix<double, NUM_DOF, NUM_INPUTS> &G, const State &x)
{
  G.block<3,3>(DQX,UAX) = -common::I_3x3;
  G.block<3,3>(DVX,UAX) = -common::skew(x.v);
  G.block<3,3>(DVX,UWX) = -common::I_3x3;
}


void EKF::imageH(Eigen::Matrix<double, 5, NUM_DOF> &H, const State &x)
{
  Eigen::Vector3d pt = q_bc_.rot(x.q.rot(pk_ + qk_.inv().rot(p_bc_) - x.p) - p_bc_);
  Eigen::Vector3d t = pt / pt.norm();
  Eigen::Vector3d t_x_k = t.cross(common::e3);
  double tT_k = t.transpose() * common::e3;
  Eigen::Vector3d at = acos(tT_k) * t_x_k / t_x_k.norm();

  Eigen::Matrix3d Gamma_at = common::Quaternion::dexp(at);
  double txk_mag = t_x_k.norm();
  common::Quaternion expat = common::Quaternion::exp(at);
  Eigen::Matrix<double, 2, 3> dexpat_dat = common::I_2x3 * expat.R().transpose() * Gamma_at;
  Eigen::Matrix3d dat_dt = acos(tT_k) / txk_mag * ((t_x_k * t_x_k.transpose()) /
                           (txk_mag * txk_mag) - common::I_3x3) * common::skew(common::e3) -
                           (t_x_k * common::e3.transpose()) / (txk_mag * sqrt(1.0 - tT_k * tT_k));
  double ptmag = pt.norm();
  Eigen::Matrix3d dt_dpt = (1.0 / ptmag) * (common::I_3x3 - pt * pt.transpose() / (ptmag * ptmag));
  Eigen::Matrix3d dpt_dp = -q_bc_.R() * x.q.R();
  Eigen::Matrix3d dpt_dq = q_bc_.R() * common::skew(q_bc_.rot(pk_ + qk_.R().transpose() * p_bc_ - x.p));

  H.block<2,3>(0,DPX) = dexpat_dat * dat_dt * dt_dpt * dpt_dp;
  H.block<2,3>(0,DQX) = dexpat_dat * dat_dt * dt_dpt * dpt_dq;
  H.block<3,3>(2,DQX) = -q_bc_.R() * qk_.R() * x.q.R().transpose();
}


}
