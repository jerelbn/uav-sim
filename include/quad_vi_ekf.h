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
  NUM_BASE_STATES = 16
};

// Derivative indices
enum
{
  DP = 0,
  DV = 3,
  DQ = 6,
  DBA = 9,
  DBG = 12,
  NUM_BASE_DOF = 15
};

// Input indices
enum
{
  UA = 0,
  UG = 3,
  NUM_INPUTS = 6
};

typedef Matrix<double, NUM_BASE_STATES, 1> baseXVector;
typedef Matrix<double, NUM_BASE_DOF, 1> baseDxVector;
typedef Matrix<double, NUM_INPUTS, 1> uVector;

struct Feat
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Feat()
  {
    pix.setZero();
    rho = 0.001;
    id = -1;
  }

  Feat(const Vector2d& _pix, const double& _rho, const int& _id)
  {
    pix = _pix;
    rho = _rho;
    id = _id;
  }

  Vector2d pix;
  double rho;
  int id;
};
typedef vector<Feat, aligned_allocator<Feat>> FeatVec;


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
    feats.reserve(nf);
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
    feats.reserve(nf);
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
      Feat f;
      f.pix = feats[i].pix + delta.template segment<2>(NUM_BASE_DOF+3*i);
      f.rho = feats[i].rho + delta(NUM_BASE_DOF+3*i+2);
      f.id = feats[i].id;
      x.feats.push_back(f);
    }
    return x;
  }

  VectorXd operator-(const State<T> &x2) const
  {
    VectorXd dx(NUM_BASE_DOF+3*nf);
    dx.template segment<3>(DP) = p - x2.p;
    dx.template segment<3>(DV) = v - x2.v;
    dx.template segment<3>(DQ) = q - x2.q;
    dx.template segment<3>(DBA) = ba - x2.ba;
    dx.template segment<3>(DBG) = bg - x2.bg;
    for (int i = 0; i < nf; ++i)
    {
      dx.template segment<2>(NUM_BASE_DOF+3*i) = feats[i].pix - x2.feats[i].pix;
      dx(NUM_BASE_DOF+3*i+2) = feats[i].rho - x2.feats[i].rho;
    }
    return dx;
  }

  void operator+=(const VectorXd &delta)
  {
    *this = *this + delta;
  }


  VectorXd toEigen() const
  {
    VectorXd x(NUM_BASE_STATES+3*nf);
    x.template segment<3>(P) = p;
    x.template segment<3>(V) = v;
    x.template segment<4>(Q) = q.elements();
    x.template segment<3>(BA) = ba;
    x.template segment<3>(BG) = bg;
    for (int i = 0; i < nf; ++i)
    {
      x.template segment<2>(NUM_BASE_STATES+3*i) = feats[i].pix;
      x(NUM_BASE_STATES+3*i+2) = feats[i].rho;
    }
    return x;
  }

  Matrix<T,3,1> p;
  Matrix<T,3,1> v;
  quat::Quat<T> q;
  Matrix<T,3,1> ba;
  Matrix<T,3,1> bg;
  FeatVec feats;
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
  void cameraUpdate(const VectorXd &z);
  void update(const VectorXd& err, const MatrixXd &R, const MatrixXd& H, MatrixXd &K);
  void logTruth(const double &t, const sensors::Sensors &sensors, const vehicle::Stated& x_true);
  void logEst(const double &t);
  void proj(const vehicle::Stated& x_true, const Vector3d& lm, Vector2d& pix, double& rho);

  Matrix<double,2,3> Omega(const Vector2d& nu);
  Matrix<double,2,3> V(const Vector2d& nu);
  RowVector3d M(const Vector2d& nu);

  // Temporary variables for testing pixel propagation
  vector<Vector3d, aligned_allocator<Vector3d>> lms_; // landmarks in inertial frame

  // Primary variables
  int num_feat_, num_states_, num_dof_;
  double t_prev_;
  double rho0_;
  State<double> x_;
  VectorXd xdot_, xdot_prev_, dxp_, dxm_;
  MatrixXd P_, F_, A_, Qx_, G_, B_;
  Matrix3d P0_feat_, Qx_feat_;
  Matrix<double,NUM_INPUTS,NUM_INPUTS> Qu_;
  MatrixXd I_NUM_DOF_;
  VectorXd P_diag_;

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

  // Logging
  ofstream true_state_log_;
  ofstream ekf_state_log_;
  ofstream cov_log_;

};


} // namespace qviekf
