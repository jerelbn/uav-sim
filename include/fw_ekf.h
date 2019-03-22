#pragma once

#include <Eigen/Dense>
//#include <ceres/ceres.h>
#include <chrono>
#include "common_cpp/common.h"
#include "geometry/quat.h"
#include "geometry/support.h"
#include "sensors.h"
#include "vehicle.h"


using namespace std;
using namespace Eigen;


namespace ekf
{


// State Indices
enum
{
  P = 0,
  V = 3,
  Q = 6,
  BA = 10,
  BG = 13,
  VW  = 16,
  BB = 19,
  NUM_STATES = 20
};

// Derivative indices
enum
{
  DP = 0,
  DV = 3,
  DQ = 6,
  DBA = 9,
  DBG = 12,
  DVW = 15,
  DBB = 18,
  NUM_DOF = 19
};

// Input indices
enum
{
  UA = 0,
  UG = 3,
  NUM_INPUTS = 6
};

typedef Matrix<double, NUM_STATES, 1> xVector;
typedef Matrix<double, NUM_DOF, 1> dxVector;
typedef Matrix<double, NUM_DOF, NUM_DOF> dxMatrix;
typedef Matrix<double, NUM_INPUTS, 1> uVector;
typedef Matrix<double, NUM_INPUTS, NUM_INPUTS> uMatrix;
typedef Matrix<double, NUM_DOF, NUM_INPUTS> nuMatrix;


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
    vw.setZero();
    bb = 0;
  }

  State(const State<T>& x)
  {
    p = x.p;
    v = x.v;
    q = x.q;
    ba = x.ba;
    bg = x.bg;
    vw = x.vw;
    bb = x.bb;
  }

  State(const Matrix<T,NUM_STATES,1> &x)
  {
    p = x.template segment<3>(P);
    v = x.template segment<3>(V);
    q = quat::Quat<T>(x.template segment<4>(Q));
    ba = x.template segment<3>(BA);
    bg = x.template segment<3>(BG);
    vw = x.template segment<3>(VW);
    bb = x(BB);
  }

  State(const T* ptr)
  {
    p = Matrix<T,3,1>(ptr[P], ptr[P+1], ptr[P+2]);
    v = Matrix<T,3,1>(ptr[V], ptr[V+1], ptr[V+2]);
    q = quat::Quat<T>(ptr+Q);
    ba = Matrix<T,3,1>(ptr[BA], ptr[BA+1], ptr[BA+2]);
    bg = Matrix<T,3,1>(ptr[BG], ptr[BG+1], ptr[BG+2]);
    vw = Matrix<T,3,1>(ptr[VW], ptr[VW+1], ptr[VW+2]);
    bb = ptr[BB];
  }

  State<T> operator+(const Matrix<T,NUM_DOF,1> &delta) const
  {
    State<T> x;
    x.p = p + delta.template segment<3>(DP);
    x.v = v + delta.template segment<3>(DV);
    x.q = q + delta.template segment<3>(DQ);
    x.ba = ba + delta.template segment<3>(DBA);
    x.bg = bg + delta.template segment<3>(DBG);
    x.vw = vw + delta.template segment<3>(DVW);
    x.bb = bb + delta(DBB);
    return x;
  }

  Matrix<T,NUM_DOF,1> operator-(const State<T> &x2) const
  {
    Matrix<T,NUM_DOF,1> dx;
    dx.template segment<3>(DP) = p - x2.p;
    dx.template segment<3>(DV) = v - x2.v;
    dx.template segment<3>(DQ) = q - x2.q;
    dx.template segment<3>(DBA) = ba - x2.ba;
    dx.template segment<3>(DBG) = bg - x2.bg;
    dx.template segment<3>(DVW) = vw - x2.vw;
    dx(DBB) = bb - x2.bb;
    return dx;
  }

  void operator+=(const Matrix<T,NUM_DOF,1> &delta)
  {
    *this = *this + delta;
  }


  Matrix<T,NUM_STATES,1> toEigen() const
  {
    Matrix<T,NUM_STATES,1> x;
    x << p, v, q.elements(), ba, bg, vw, bb;
    return x;
  }

  Matrix<T,3,1> p;
  Matrix<T,3,1> v;
  quat::Quat<T> q;
  Matrix<T,3,1> ba;
  Matrix<T,3,1> bg;
  Matrix<T,3,1> vw;
  T bb;

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
  static void f(const Stated &x, const uVector& u, dxVector& dx);
  static void getFG(const Stated &x, const uVector& u, dxMatrix& F, nuMatrix& G);
  vehicle::Stated getState() const;
  Vector3d getWind() const { return x_.vw; }

private:

  void propagate(const double &t, const uVector &imu);
  void updateGPS(const Matrix<double,6,1>& z);
  void updateBaro(const double& z);
  void updatePitot(const double& z);
  void updateWVane(const double& z);
  void logTruth(const double &t, const sensors::Sensors &sensors, const Vector3d& vw, const vehicle::Stated& x_true);
  void logEst(const double &t);

  // Primary variables
  double t_prev_;
  State<double> x_;
  dxVector xdot_, xdot_prev_;
  dxMatrix P_, F_, A_;
  dxMatrix Qx_;
  nuMatrix G_, B_;
  uMatrix Qu_;
  dxMatrix I_NUM_DOF_;

  // Sensor parameters
  double rho_;
  Matrix6d R_gps_;
  double R_baro_, R_pitot_, R_wv_;
  Vector3d p_ub_;
  quat::Quatd q_u2b_, q_u2pt_, q_u2wv_;

  // Logging
  ofstream true_state_log_;
  ofstream ekf_state_log_;
  ofstream cov_log_;

};


} // namespace ekf
