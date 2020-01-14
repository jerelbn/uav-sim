#pragma once

#include <Eigen/Dense>
#include <chrono>
#include "common_cpp/common.h"
#include "common_cpp/measurement.h"
#include "common_cpp/logger.h"
#include "geometry/quat.h"
#include "geometry/support.h"
#include "sensors.h"
#include "vehicle.h"
#include "wgs84.h"


using namespace std;
using namespace Eigen;


namespace gmbl_ekf
{


// State Indices
enum
{
  V = 0,
  Q = 3,
  SA = 7,
  BG = 8,
  BM = 11,
  NUM_STATES = 12
};

// Derivative indices
enum
{
  DV = 0,
  DQ = 3,
  DSA = 6,
  DBG = 7,
  DBM = 10,
  NUM_DOF = 11
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
    v.setZero();
    sa = 0;
    bg.setZero();
    bm = 0;
  }

  State(const State<T>& x)
  {
    v = x.v;
    q = x.q;
    sa = x.sa;
    bg = x.bg;
    bm = x.bm;
  }

  State(const Matrix<T,NUM_STATES,1> &x)
  {
    v = x.template segment<3>(V);
    q = quat::Quat<T>(x.template segment<4>(Q));
    sa = x(SA);
    bg = x.template segment<3>(BG);
    bm = x(BM);
  }

  State(const T* ptr)
  {
    v = Matrix<T,3,1>(ptr[V], ptr[V+1], ptr[V+2]);
    q = quat::Quat<T>(ptr+Q);
    sa = ptr[SA];
    bg = Matrix<T,3,1>(ptr[BG], ptr[BG+1], ptr[BG+2]);
    bm = ptr[BM];
  }

  State<T> operator+(const Matrix<T,NUM_DOF,1> &delta) const
  {
    State<T> x;
    x.v = v + delta.template segment<3>(DV);
    x.q = q + delta.template segment<3>(DQ);
    x.sa = sa + delta(DSA);
    x.bg = bg + delta.template segment<3>(DBG);
    x.bm = bm + delta(DBM);
    return x;
  }

  Matrix<T,NUM_DOF,1> operator-(const State<T> &x2) const
  {
    Matrix<T,NUM_DOF,1> dx;
    dx.template segment<3>(DV) = v - x2.v;
    dx.template segment<3>(DQ) = q - x2.q;
    dx(DSA) = sa - x2.sa;
    dx.template segment<3>(DBG) = bg - x2.bg;
    dx(DBM) = bm - x2.bm;
    return dx;
  }

  void operator+=(const Matrix<T,NUM_DOF,1> &delta)
  {
    *this = *this + delta;
  }


  Matrix<T,NUM_STATES,1> toEigen() const
  {
    Matrix<T,NUM_STATES,1> x;
    x << v, q.elements(), sa, bg, bm;
    return x;
  }

  Matrix<T,3,1> v, bg;
  quat::Quat<T> q;
  T sa, bm;

};
typedef State<double> Stated;


class EKF
{

public:

  EKF();
  EKF(const string &filename, const string &name);
  ~EKF();

  void load(const string &filename, const string &name);
  void run(const double &t, const sensors::Sensors &gimbal_sensors, const sensors::Sensors &aircraft_sensors,
           const quat::Quatd& q_bg, const vehicle::Stated& xg_true, const vehicle::Stated& xac_true);
  static void f(const Stated &x, const uVector& u, dxVector& dx);
  static void h_gps(const Stated &x, const quat::Quatd &q_ecef2ned, Vector3d& h);
  static void h_mag(const Stated &x, const Vector3d &pos_ecef, const quat::Quatd &q_ecef2ned, const Vector3d &mnp_ecef, const quat::Quatd &q_ecef2mnp, double& h);
  static void h_cam(const Stated &x, quat::Quatd& h);
  static void getF(const Stated &x, const uVector& u, dxMatrix& F);
  static void getG(const Stated &x, const uVector& u, nuMatrix& G);
  static void getH_gps(const Stated &x, const Matrix3d &R_ecef2ned, Matrix<double,3,NUM_DOF> &H);
  static void getH_mag(const Stated &x, const Vector3d &pos_ecef, const quat::Quatd &q_ecef2ned, const Vector3d &mnp_ecef, const quat::Quatd &q_ecef2mnp, Matrix<double,1,NUM_DOF> &H);
  static void getH_cam(const Stated &x, Matrix<double,3,NUM_DOF> &H);
  static Vector3d magFieldNED(const Vector3d &pos_ecef, const quat::Quatd &q_ecef2ned, const Vector3d &mnp_ecef, const quat::Quatd &q_ecef2mnp);
  vehicle::Stated stateRelToBody(const vehicle::Stated& x_Ib) const;

private:

  void propagate(const double &t, const uVector &imu);
  void updateGPS(const Matrix<double,6,1>& z);
  void updateMag(const Matrix<double,3,1>& z, const quat::Quatd &q_bg);
  void updateCam(const vehicle::Stated& xac_true, const vehicle::Stated& xg_true);
  void logTruth(const double &t, const sensors::Sensors &gimbal_sensors, const sensors::Sensors &aircraft_sensors,
                const vehicle::Stated& xg_true, const vehicle::Stated& xac_true);
  void logEst(const double &t);

  // Primary variables
  double t_prev_;
  bool mag_initialized_;
  Stated x_;
  dxVector xdot_;
  dxMatrix P_, F_, A_;
  dxMatrix Qx_;
  nuMatrix G_, B_;
  uMatrix Qu_;
  dxMatrix I_NUM_DOF_;

  Vector6d imu_prev_;

  // Sensor parameters
  double rho_;
  Matrix3d R_gps_, R_cam_;
  double R_mag_;
  Vector3d p_bu_, p_gcb_;
  quat::Quatd q_bu_, q_gcb_, q_ecef_to_mnp_;
  quat::Quatd qk_, qkt_; // keyframe attitude and true keyframe attitude
  xform::Xformd X_ecef2ned_;
  Vector3d mnp_ecef_, last_gps_pos_;

  // Logging
  common::Logger true_state_log_;
  common::Logger ekf_state_log_;
  common::Logger cov_log_;

};


} // namespace gmbl_ekf
