#pragma once

#include <Eigen/Dense>
#include <chrono>
#include "vi_ekf_state.h"
#include "common_cpp/common.h"
#include "geometry/quat.h"
#include "geometry/xform.h"
#include "geometry/support.h"
#include "sensors.h"
#include "vehicle.h"

using namespace std;
using namespace Eigen;


namespace qviekf
{


class EKF
{

public:

  EKF();
  EKF(string filename);
  ~EKF();

  void load(const string &filename, const string &name);
  void run(const double &t, const sensors::Sensors &sensors, const Vector3d &vw, const vehicle::Stated &x_true);
  vehicle::Stated getState() const;

private:

  void propagate(const double &t, const uVector &imu);
  void f(const Stated &x, const uVector& u, VectorXd &dx, const uVector& eta = uVector::Zero());
  void analyticalFG(const Stated &x, const uVector& u, MatrixXd& F, MatrixXd& G);
  void numericalFG(const Stated &x, const uVector& u, MatrixXd& F, MatrixXd& G);
  void gpsUpdate(const Vector6d& z);
  void mocapUpdate(const Vector7d& z);
  void cameraUpdate(const sensors::FeatVec &tracked_feats);
  void update(const VectorXd& err, const MatrixXd &R, const MatrixXd& H, MatrixXd &K);
  void logTruth(const double &t, const sensors::Sensors &sensors, const vehicle::Stated& x_true);
  void logEst(const double &t);
  void getPixMatches(const sensors::FeatVec& tracked_feats);
  void removeFeatFromState(const int& idx);
  void addFeatToState(const sensors::FeatVec &tracked_feats);
  void keyframeReset(const sensors::FeatVec &tracked_feats);
  void numericalN(const Stated &x, MatrixXd& N);

  Matrix<double,2,3> Omega(const Vector2d& nu);
  Matrix<double,2,3> V(const Vector2d& nu);
  RowVector3d M(const Vector2d& nu);

  // Primary variables
  bool use_drag_, use_partial_update_, use_keyframe_reset_;
  int nfm_; // maximum number of features in the state
  int nfa_; // active number of features in the state
  int nbs_, nbd_; // number of base states/degrees of freedom
  int num_states_, num_dof_;
  double t_prev_;
  double rho0_;
  bool init_imu_bias_;
  State<double> x_;
  VectorXd xdot_, xdot_prev_, dxp_, dxm_, lambda_, dx_ones_;
  MatrixXd P_, F_, A_, Qx_, G_, B_, Lambda_, N_;
  Matrix3d P0_feat_, Qx_feat_;
  uMatrix Qu_;
  MatrixXd I_DOF_;
  VectorXd P_diag_;
  vector<Vector2d, aligned_allocator<Vector2d>> matched_feats_;

  // Keyframe reset parameters
  bool initial_keyframe_;
  int kfr_min_matches_;
  double kfr_mean_pix_disparity_thresh_;
  sensors::FeatVec kf_feats_;
  Vector3d p_global_;
  quat::Quatd q_yaw_global_;

  // Sensor parameters
  Vector6d h_gps_;
  MatrixXd H_gps_, K_gps_;
  Matrix6d R_gps_;
  xform::Xformd h_mocap_;
  MatrixXd H_mocap_, K_mocap_;
  Matrix6d R_mocap_;
  VectorXd z_cam_, h_cam_;
  MatrixXd H_cam_, K_cam_;
  Matrix2d R_cam_;
  MatrixXd R_cam_big_;
  Vector3d p_ub_, p_um_, p_uc_;
  quat::Quatd q_u2b_, q_u2m_, q_u2c_;
  Matrix3d cam_matrix_;
  double fx_, fy_, u0_, v0_;
  Vector2d image_center_;
  sensors::FeatVec feats_true_;

  // Logging
  ofstream true_state_log_;
  ofstream ekf_state_log_;
  ofstream cov_log_;

};


} // namespace qviekf
