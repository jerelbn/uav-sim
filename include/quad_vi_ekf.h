#pragma once

#include <Eigen/Dense>
#include <chrono>
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


// State Indices
enum
{
  P = 0,
  V = 3,
  Q = 6,
  BA = 10,
  BG = 13,
  NBS = 16 // number of base states (neglecting features)
};

// Derivative indices
enum
{
  DP = 0,
  DV = 3,
  DQ = 6,
  DBA = 9,
  DBG = 12,
  NBD = 15 // number of base degrees of freedom (neglecting features)
};

// Input indices
enum
{
  UA = 0,
  UG = 3,
  NI = 6 // number of propagation inputs (imu)
};

typedef Matrix<double, NBS, 1> baseXVector;
typedef Matrix<double, NBD, 1> baseDxVector;
typedef Matrix<double, NI, 1> uVector;
typedef Matrix<double, NI, NI> uMatrix;


template<typename T>
struct State
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  State() {}

  State(const int& num_feat)
  {
    nf = num_feat;
    p.setZero();
    v.setZero();
    ba.setZero();
    bg.setZero();
    for (int i = 0; i < nf; ++i)
      feats.push_back(sensors::Feat());
  }

  State(const State<T>& x)
  {
    nf = x.nf;
    p = x.p;
    v = x.v;
    q = x.q;
    ba = x.ba;
    bg = x.bg;
    feats = x.feats;
  }

  State(const baseXVector &x, const int& num_feat)
  {
    nf = num_feat;
    p = x.template segment<3>(P);
    v = x.template segment<3>(V);
    q = quat::Quat<T>(x.template segment<4>(Q));
    ba = x.template segment<3>(BA);
    bg = x.template segment<3>(BG);
    for (int i = 0; i < nf; ++i)
      feats.push_back(sensors::Feat());
  }

  State<T> operator+(const VectorXd &delta) const
  {
    State<T> x(nf);
    x.p = p + delta.template segment<3>(DP);
    x.v = v + delta.template segment<3>(DV);
    x.q = q + delta.template segment<3>(DQ);
    x.ba = ba + delta.template segment<3>(DBA);
    x.bg = bg + delta.template segment<3>(DBG);
    for (int i = 0; i < nf; ++i)
    {
      sensors::Feat f;
      f.pix = feats[i].pix + delta.template segment<2>(NBD+3*i);
      f.rho = feats[i].rho + delta(NBD+3*i+2);
      f.id = feats[i].id;
      x.feats[i] = f;
    }
    return x;
  }

  VectorXd operator-(const State<T> &x2) const
  {
    VectorXd dx(NBD+3*nf);
    dx.template segment<3>(DP) = p - x2.p;
    dx.template segment<3>(DV) = v - x2.v;
    dx.template segment<3>(DQ) = q - x2.q;
    dx.template segment<3>(DBA) = ba - x2.ba;
    dx.template segment<3>(DBG) = bg - x2.bg;
    for (int i = 0; i < nf; ++i)
    {
      dx.template segment<2>(NBD+3*i) = feats[i].pix - x2.feats[i].pix;
      dx(NBD+3*i+2) = feats[i].rho - x2.feats[i].rho;
    }
    return dx;
  }

  void operator+=(const VectorXd &delta)
  {
    *this = *this + delta;
  }


  VectorXd vec() const
  {
    VectorXd x(NBS+3*nf);
    x.template segment<3>(P) = p;
    x.template segment<3>(V) = v;
    x.template segment<4>(Q) = q.elements();
    x.template segment<3>(BA) = ba;
    x.template segment<3>(BG) = bg;
    for (int i = 0; i < nf; ++i)
    {
      x.template segment<2>(NBS+3*i) = feats[i].pix;
      x(NBS+3*i+2) = feats[i].rho;
    }
    return x;
  }

  Matrix<T,3,1> p;
  Matrix<T,3,1> v;
  quat::Quat<T> q;
  Matrix<T,3,1> ba;
  Matrix<T,3,1> bg;
  sensors::FeatVec feats;
  int nf;

};
typedef State<double> Stated;


class EKF
{

public:

  EKF();
  EKF(string filename);
  ~EKF();

  void load(const string &filename, const string &name);
  void run(const double &t, const sensors::Sensors &sensors, const Vector3d &vw, const vehicle::Stated &x_true);
  void f(const Stated &x, const uVector& u, VectorXd &dx);
  void f2(const Stated &x, const uVector& u, const uVector& eta, VectorXd& dx);
  void analyticalFG(const Stated &x, const uVector& u, MatrixXd& F, MatrixXd& G);
  void numericalFG(const Stated &x, const uVector& u, MatrixXd& F, MatrixXd& G);
  vehicle::Stated getState() const;

private:

  void propagate(const double &t, const uVector &imu);
  void gpsUpdate(const Vector6d& z);
  void mocapUpdate(const Vector7d& z);
  void cameraUpdate(const sensors::FeatVec &tracked_feats);
  void update(const VectorXd& err, const MatrixXd &R, const MatrixXd& H, MatrixXd &K);
  void logTruth(const double &t, const sensors::Sensors &sensors, const vehicle::Stated& x_true);
  void logEst(const double &t);
  void getPixMatches(const sensors::FeatVec& tracked_feats);

  Matrix<double,2,3> Omega(const Vector2d& nu);
  Matrix<double,2,3> V(const Vector2d& nu);
  RowVector3d M(const Vector2d& nu);

  // Primary variables
  int nfm_; // maximum number of features in the state
  int nfa_; // active number of features in the state
  int num_states_, num_dof_;
  double t_prev_;
  double rho0_;
  bool init_imu_bias_;
  State<double> x_;
  VectorXd xdot_, xdot_prev_, dxp_, dxm_;
  MatrixXd P_, F_, A_, Qx_, G_, B_;
  Matrix3d P0_feat_, Qx_feat_;
  uMatrix Qu_;
  MatrixXd I_DOF_;
  VectorXd P_diag_;
  vector<Vector2d, aligned_allocator<Vector2d>> matched_feats_;

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
  sensors::FeatVec feats_true_;

  // Logging
  ofstream true_state_log_;
  ofstream ekf_state_log_;
  ofstream cov_log_;

};


} // namespace qviekf
