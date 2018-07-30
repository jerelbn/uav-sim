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


EKF::EKF() {}


EKF::EKF(std::string filename)
{
  load(filename);

  // Pre-allocate memory for vector arrays
  pts_k_.reserve(10000);
  pts_match_.reserve(10000);
  pts_match_k_.reserve(10000);
  dv_.reserve(10000);
  dv_k_.reserve(10000);
}


EKF::~EKF()
{
  // close log files
}


void EKF::load(const std::string filename)
{
  xVector x0;
  dxVector P0_diag, Qx_diag;
  Eigen::Vector4d q_bc;
  Eigen::Matrix<double, NUM_INPUTS, 1> Qu_diag;
  common::get_yaml_eigen("x0", filename, x0);
  x_ = State(x0);
  common::get_yaml_eigen("P0_diag", filename, P0_diag);
  P_ = P0_diag.asDiagonal();
  common::get_yaml_eigen("Qx_diag", filename, Qx_diag);
  Qx_ = Qx_diag.asDiagonal();
  common::get_yaml_eigen("Qu_diag", filename, Qu_diag);
  Qu_ = Qu_diag.asDiagonal();
  common::get_yaml_eigen("lambda", filename, lambda_);
  Lambda_ = ones_vec_ * lambda_.transpose() + lambda_ * ones_vec_.transpose() - lambda_ * lambda_.transpose();
  common::get_yaml_eigen("camera_matrix", filename, K_);
  K_inv_ = K_.inverse();
  common::get_yaml_eigen("q_bc", filename, q_bc);
  q_bc_ = common::Quaternion(q_bc);
  q_bc_.normalize();
  common::get_yaml_eigen("p_bc", filename, p_bc_);
  common::get_yaml_node("pixel_disparity_threshold", filename, pixel_disparity_threshold_);
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


void EKF::imageUpdate(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &pts,
                      const Eigen::Matrix<double, 5, 5> &R)
{
  // Match current image points to keyframe image points
  pts_match_.clear();
  pts_match_k_.clear();
  for (int i = 0; i < pts_k_.size(); ++i)
  {
    for (int j = 0; j < pts.size(); ++j)
    {
      if (pts_k_[i](2) == pts[j](2))
      {
        pts_match_.push_back(pts[j].topRows(2));
        pts_match_k_.push_back(pts_k_[i].topRows(2));
      }
    }
  }

  // Create new keyframe if necessary
  if (pts_match_.size() < 30)
  {
    pk_ = x_.p;
    qk_ = x_.q;
    pts_k_ = pts;
    return;
  }

  // Remove camera rotation and compute mean image point disparity
  dv_.clear();
  dv_k_.clear();
  double mean_disparity = 0;
  common::Quaternion q_c2ck = q_bc_.inv() * x_.q.inv() * qk_ * q_bc_;
  for (int n = 0; n < pts_match_.size(); ++n)
  {
    // Image points without rotation
    Eigen::Vector3d pt1 = K_ * q_c2ck.rot(K_inv_ * Eigen::Vector3d(pts_match_[n](0), pts_match_[n](1), 1.0));
    Eigen::Vector3d pt2 = pt1 / pt1(2);

    // Recursive mean of disparity
    mean_disparity = (n * mean_disparity + (pt2.topRows(2) - pts_match_k_[n]).norm()) / (n + 1);

    // Convert point matches to direction vectors
    Eigen::Vector3d dv = K_inv_ * Eigen::Vector3d(pts_match_[n](0), pts_match_[n](1), 1.0);
    Eigen::Vector3d dv_k = K_inv_ * Eigen::Vector3d(pts_match_k_[n](0), pts_match_k_[n](1), 1.0);
    dv_.push_back(dv / dv.norm());
    dv_k_.push_back(dv_k / dv_k.norm());
  }

  // Estimate camera pose relative to keyframe via nonlinear optimization and apply update
  if (mean_disparity > pixel_disparity_threshold_)
  {
    // Optimize for VO measurement
    common::Quaternion zt, zq;
    optimizePose(zq, zt, dv_k_, dv_, 10);

    // Measurement model and jacobian
    common::Quaternion ht, hq;
    Eigen::Matrix<double, 5, NUM_DOF> H;
    imageH(ht, hq, H, x_, q_bc_, p_bc_, qk_, pk_);

    // Error in translation direction and rotation
    Eigen::Vector2d err_t = common::Quaternion::log_uvec(zt,ht);
    Eigen::Vector3d err_q = zq - hq;

    // Innovation error
    Eigen::Matrix<double, 5, 1> err_i;
    err_i << err_t, err_q;

    // Change in state and covariance
    Eigen::Matrix<double, NUM_DOF, 5> K = P_ * H.transpose() * (R + H * P_ * H.transpose()).inverse();
    dxVector delta_x = lambda_.cwiseProduct(K * err_i);
    dxMatrix delta_P = Lambda_.cwiseProduct((I_num_dof_ - K * H) * P_ * (I_num_dof_ - K * H).transpose() +
                       K * R * K.transpose() - P_);

    // Apply update
    x_ += delta_x;
    P_ += delta_P;
  }
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
  F.setZero();
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
  G.setZero();
  G.block<3,3>(DQX,UAX) = -common::I_3x3;
  G.block<3,3>(DVX,UAX) = -common::skew(x.v);
  G.block<3,3>(DVX,UWX) = -common::I_3x3;
}


void EKF::imageH(common::Quaternion &ht, common::Quaternion &hq, Eigen::Matrix<double, 5, NUM_DOF> &H, const State &x,
                 const common::Quaternion &q_bc, const Eigen::Vector3d &p_bc, const common::Quaternion &q_ik,
                 const Eigen::Vector3d &p_ik)
{
  Eigen::Vector3d pt = q_bc.rot(x.q.rot(p_ik + q_ik.inv().rot(p_bc) - x.p) - p_bc);
  Eigen::Vector3d t = pt / pt.norm();
  Eigen::Vector3d t_x_k = t.cross(common::e3);
  double tT_k = t.transpose() * common::e3;
  Eigen::Vector3d at = acos(tT_k) * t_x_k / t_x_k.norm();

  ht = common::Quaternion::exp(at);
  hq = q_bc.inv() * x.q.inv() * q_ik * q_bc;

  Eigen::Matrix3d Gamma_at = common::Quaternion::dexp(at);
  double txk_mag = t_x_k.norm();
  Eigen::Matrix<double, 2, 3> dexpat_dat = common::I_2x3 * ht.R().transpose() * Gamma_at;
  Eigen::Matrix3d dat_dt = acos(tT_k) / txk_mag * ((t_x_k * t_x_k.transpose()) /
                           (txk_mag * txk_mag) - common::I_3x3) * common::skew(common::e3) -
                           (t_x_k * common::e3.transpose()) / (txk_mag * sqrt(1.0 - tT_k * tT_k));
  double ptmag = pt.norm();
  Eigen::Matrix3d dt_dpt = (1.0 / ptmag) * (common::I_3x3 - pt * pt.transpose() / (ptmag * ptmag));
  Eigen::Matrix3d dpt_dp = -q_bc.R() * x.q.R();
  Eigen::Matrix3d dpt_dq = q_bc.R() * common::skew(x.q.rot(p_ik + q_ik.inv().rot(p_bc) - x.p));

  H.setZero();
  H.block<2,3>(0,DPX) = dexpat_dat * dat_dt * dt_dpt * dpt_dp;
  H.block<2,3>(0,DQX) = dexpat_dat * dat_dt * dt_dpt * dpt_dq;
  H.block<3,3>(2,DQX) = -q_bc.R() * q_ik.R() * x.q.R().transpose();
}


// Relative camera pose optimizer
void EKF::optimizePose(common::Quaternion& q, common::Quaternion& qt,
                       const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& e1,
                       const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& e2,
                       const unsigned iters)
{
  // Ensure number of directions vectors from each camera match
  if (e1.size() != e2.size())
  {
    std::cout << "\nError in optimizePose. Direction vector arrays must be the same size.\n\n";
    return;
  }

  // Constants
  const int N = e1.size();
  static Eigen::Matrix3d e1x = common::skew(common::e1);
  static Eigen::Matrix3d e2x = common::skew(common::e2);
  static Eigen::Matrix3d e3x = common::skew(common::e3);

  // Declare things that change each loop
  static Eigen::Matrix3d R, Rtx, e1x_Rtx, e2x_Rtx, e3x_Rtx, e1tx, e2tx, R_e1tx_tx, R_e2tx_tx;
  static Eigen::Vector3d t, e1tx_t, e2tx_t;
  static Eigen::Matrix<double,5,1> delta;
  static Eigen::VectorXd r(N);
  static Eigen::MatrixXd J(N,5);

  // Main optimization loop
  for (int i = 0; i < iters; ++i)
  {
    // Pre-compute a few things
    R = q.R();
    t = qt.uvec();
    Rtx = R*common::skew(t);
    e1x_Rtx = e1x * Rtx;
    e2x_Rtx = e2x * Rtx;
    e3x_Rtx = e3x * Rtx;
    e1tx = common::skew(qt.rot(common::e1));
    e2tx = common::skew(qt.rot(common::e2));
    e1tx_t = e1tx * t;
    e2tx_t = e2tx * t;
    R_e1tx_tx = R * common::skew(e1tx_t);
    R_e2tx_tx = R * common::skew(e2tx_t);

    // Vector of residual errors
    for (int j = 0; j < N; ++j)
      se(r(j), e1[j], e2[j], Rtx);

    // Jacobian of residual errors
    for (int j = 0; j < N; ++j)
    {
      dse(J(j,0), e1[j], e2[j], Rtx, -e1x_Rtx);
      dse(J(j,1), e1[j], e2[j], Rtx, -e2x_Rtx);
      dse(J(j,2), e1[j], e2[j], Rtx, -e3x_Rtx);
      dse(J(j,3), e1[j], e2[j], Rtx, -R_e1tx_tx);
      dse(J(j,4), e1[j], e2[j], Rtx, -R_e2tx_tx);
    }

    // Innovation
    delta = -(J.transpose() * J).inverse() * J.transpose() * r;

    // Update camera rotation and translation
    q = q * common::Quaternion::exp(delta.segment<3>(0));
    qt = qt * common::Quaternion::exp(qt.proj() * delta.segment<2>(3));
  }
}


// Sampson's error
void EKF::se(double& err, const Eigen::Vector3d& e1, const Eigen::Vector3d& e2, const Eigen::Matrix3d& E)
{
  static Eigen::Vector3d e1T_E, E_e2;
  double e1T_E_e2 = e1.transpose() * E * e2;
  e1T_E = (e1.transpose() * E).transpose();
  E_e2 = E * e2;
  err = e1T_E_e2 / sqrt(e1T_E(0) * e1T_E(0) + e1T_E(1) * e1T_E(1) + E_e2(0) * E_e2(0) + E_e2(1) * E_e2(1));
}


// Derivative of Sampson's error
void EKF::dse(double& derr, const Eigen::Vector3d& e1, const Eigen::Vector3d& e2, const Eigen::Matrix3d& E, const Eigen::Matrix3d& dE)
{
  static Eigen::Vector3d e1T_E, E_e2, e1T_dE, dE_e2;
  e1T_E = (e1.transpose() * E).transpose();
  E_e2 = E * e2;
  e1T_dE = (e1.transpose() * dE).transpose();
  dE_e2 = dE * e2;
  double val1 = sqrt(e1T_E(0) * e1T_E(0) + e1T_E(1) * e1T_E(1) + E_e2(0) * E_e2(0) + E_e2(1) * E_e2(1));
  double val2 = e1T_E(0) * e1T_dE(0) + e1T_E(1) * e1T_dE(1) + E_e2(0) * dE_e2(0) + E_e2(1) * dE_e2(1);
  double e1T_dE_e2 = e1.transpose() * dE * e2;
  double e1T_E_e2 = e1.transpose() * E * e2;
  derr = (e1T_dE_e2 * val1 - e1T_E_e2 * val2) / (val1 * val1);
}


}
