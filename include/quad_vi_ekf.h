#pragma once

#include <Eigen/Dense>
#include <chrono>
#include "common_cpp/common.h"
#include "geometry/quat.h"
#include "geometry/xform.h"
#include "sensors.h"
#include "vehicle.h"

#define NUM_FEATURES 4


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
  NUM_BASE_STATES = 16,
  NUM_STATES = NUM_BASE_STATES + 3 * NUM_FEATURES
};

// Derivative indices
enum
{
  DP = 0,
  DV = 3,
  DQ = 6,
  DBA = 9,
  DBG = 12,
  NUM_BASE_DOF = 15,
  NUM_DOF = NUM_BASE_DOF + 3 * NUM_FEATURES
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
typedef Matrix<double, NUM_STATES, 1> xVector;
typedef Matrix<double, NUM_DOF, 1> dxVector;
typedef Matrix<double, NUM_DOF, NUM_DOF> dxMatrix;
typedef Matrix<double, NUM_INPUTS, 1> uVector;
typedef Matrix<double, NUM_INPUTS, NUM_INPUTS> uMatrix;
typedef Matrix<double, NUM_DOF, NUM_INPUTS> nuMatrix;

typedef vector<Vector2d, aligned_allocator<Vector2d>> FeatVec;


template<typename T>
struct State
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  State()
  {
    p.setZero();
    v.setZero();
    ba.setZero();
    bg.setZero();
    for (int i = 0; i < NUM_FEATURES; ++i)
    {
      pixs.push_back(Vector2d::Zero());
      rhos.push_back(1.0);
    }
  }

  State(const State<T>& x)
  {
    p = x.p;
    v = x.v;
    q = x.q;
    ba = x.ba;
    bg = x.bg;
    pixs = x.pixs;
    rhos = x.rhos;
  }

  State(const baseXVector &x)
  {
    p = x.template segment<3>(P);
    v = x.template segment<3>(V);
    q = quat::Quat<T>(x.template segment<4>(Q));
    ba = x.template segment<3>(BA);
    bg = x.template segment<3>(BG);
    for (int i = 0; i < NUM_FEATURES; ++i)
    {
      pixs.push_back(Vector2d::Zero());
      rhos.push_back(1.0);
    }
  }

  State(const T* ptr)
  {
    p = Matrix<T,3,1>(ptr[P], ptr[P+1], ptr[P+2]);
    v = Matrix<T,3,1>(ptr[V], ptr[V+1], ptr[V+2]);
    q = quat::Quat<T>(ptr+Q);
    ba = Matrix<T,3,1>(ptr[BA], ptr[BA+1], ptr[BA+2]);
    bg = Matrix<T,3,1>(ptr[BG], ptr[BG+1], ptr[BG+2]);
    for (int i = 0; i < NUM_FEATURES; ++i)
    {
      pixs.push_back(Vector2d::Zero());
      rhos.push_back(1.0);
    }
  }

  State<T> operator+(const Matrix<T,NUM_DOF,1> &delta) const
  {
    State<T> x;
    x.p = p + delta.template segment<3>(DP);
    x.v = v + delta.template segment<3>(DV);
    x.q = q + delta.template segment<3>(DQ);
    x.ba = ba + delta.template segment<3>(DBA);
    x.bg = bg + delta.template segment<3>(DBG);
    for (int i = 0; i < NUM_FEATURES; ++i)
    {
      x.pixs[i] = pixs[i] + delta.template segment<2>(NUM_BASE_DOF+3*i);
      x.rhos[i] = rhos[i] + delta(NUM_BASE_DOF+3*i+2);
    }
    return x;
  }

  Matrix<T,NUM_DOF,1> operator-(const State<T> &x2) const
  {
    Matrix<T,NUM_BASE_DOF,1> dx;
    dx.template segment<3>(DP) = p - x2.p;
    dx.template segment<3>(DV) = v - x2.v;
    dx.template segment<3>(DQ) = q - x2.q;
    dx.template segment<3>(DBA) = ba - x2.ba;
    dx.template segment<3>(DBG) = bg - x2.bg;
    for (int i = 0; i < NUM_FEATURES; ++i)
    {
      dx.template segment<2>(NUM_BASE_DOF+3*i) = pixs[i] - x2.pixs[i];
      dx(NUM_BASE_DOF+3*i+2) = rhos[i] - x2.rhos[i];
    }
    return dx;
  }

  void operator+=(const Matrix<T,NUM_DOF,1> &delta)
  {
    *this = *this + delta;
  }


  Matrix<T,NUM_STATES,1> toEigen() const
  {
    Matrix<T,NUM_STATES,1> x;
    x.template segment<3>(P) = p;
    x.template segment<3>(V) = v;
    x.template segment<4>(Q) = q.elements();
    x.template segment<3>(BA) = ba;
    x.template segment<3>(BG) = bg;
    for (int i = 0; i < NUM_FEATURES; ++i)
    {
      x.template segment<2>(NUM_BASE_STATES+3*i) = pixs[i];
      x(NUM_BASE_STATES+3*i+2) = rhos[i];
    }
    return x;
  }

  Matrix<T,3,1> p;
  Matrix<T,3,1> v;
  quat::Quat<T> q;
  Matrix<T,3,1> ba;
  Matrix<T,3,1> bg;
  FeatVec pixs;
  vector<double> rhos;

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
  void f(const Stated &x, const uVector& u, dxVector& dx);
  void f2(const Stated &x, const uVector& u, const uVector& eta, dxVector& dx);
  void analyticalFG(const Stated &x, const uVector& u, dxMatrix& F, nuMatrix& G);
  void numericalFG(const Stated &x, const uVector& u, dxMatrix& F, nuMatrix& G);
  vehicle::Stated getState() const;

private:

  void propagate(const double &t, const uVector &imu);
  void updateGPS(const Vector6d& z);
  void updateMocap(const Vector7d& z);
  void updateCamera(const Matrix<double, 2*NUM_FEATURES, 1> &z);
  void logTruth(const double &t, const sensors::Sensors &sensors, const vehicle::Stated& x_true);
  void logEst(const double &t);
  void proj(const vehicle::Stated& x_true, const Vector3d& lm, Vector2d& pix, double& rho);

  Matrix<double,2,3> Omega(const Vector2d& nu);
  Matrix<double,2,3> V(const Vector2d& nu);
  RowVector3d M(const Vector2d& nu);

  // Temporary variables for testing pixel propagation
  vector<Vector3d, aligned_allocator<Vector3d>> lms_; // landmarks in inertial frame

  // Primary variables
  double t_prev_;
  State<double> x_;
  dxVector xdot_, xdot_prev_;
  dxMatrix P_, F_, A_;
  dxMatrix Qx_;
  nuMatrix G_, B_;
  Matrix<double,NUM_INPUTS,NUM_INPUTS> Qu_;
  dxMatrix I_NUM_DOF_;

  // Sensor parameters
  Matrix6d R_gps_;
  Matrix6d R_mocap_;
  Matrix<double,2*NUM_FEATURES,2*NUM_FEATURES> R_pix_;
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
