#pragma once

#include <eigen3/Eigen/Eigen>
#include <ceres/ceres.h>
#include <chrono>
#include "common_cpp/common.h"
#include "sensors.h"
#include "vehicle_common.h"


using namespace std;
using namespace Eigen;


namespace ekf
{


// State Indices
enum
{
  PX,
  PY,
  PZ,
  QW,
  QX,
  QY,
  QZ,
  VX,
  VY,
  VZ,
  AX,
  AY,
  AZ,
  GX,
  GY,
  GZ,
  NUM_STATES
};

// Derivative indices
enum
{
  DPX,
  DPY,
  DPZ,
  DQX,
  DQY,
  DQZ,
  DVX,
  DVY,
  DVZ,
  DAX,
  DAY,
  DAZ,
  DGX,
  DGY,
  DGZ,
  NUM_DOF
};

// Input indices
enum
{
  UAX,
  UAY,
  UAZ,
  UGX,
  UGY,
  UGZ,
  NUM_INPUTS
};

typedef Matrix<double, NUM_STATES, 1> xVector;
typedef Matrix<double, NUM_DOF, 1> dxVector;
typedef Matrix<double, NUM_DOF, NUM_DOF> dxMatrix;
typedef Matrix<double, NUM_INPUTS, 1> uVector;


template<typename T>
struct State
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  State()
  {
    t = common::Transform<T>();
    v.setZero();
    ba.setZero();
    bg.setZero();
  }

  State(const State<T>& x)
  {
    t = x.t;
    v = x.v;
    ba = x.ba;
    bg = x.bg;
  }

  State(const Matrix<T,NUM_STATES,1> &x)
  {
    t = common::Transform<T>(x.template segment<3>(PX), x.template segment<4>(QW));
    v = x.template segment<3>(VX);
    ba = x.template segment<3>(AX);
    bg = x.template segment<3>(GX);
  }

  State(const T* ptr)
  {
    t = common::Transform<T>(ptr);
    v = Matrix<T,3,1>(ptr[VX], ptr[VY], ptr[VZ]);
    ba = Matrix<T,3,1>(ptr[AX], ptr[AY], ptr[AZ]);
    bg = Matrix<T,3,1>(ptr[GX], ptr[GY], ptr[GZ]);
  }

  State<T> operator+(const Matrix<T,NUM_DOF,1> &delta) const
  {
    State<T> x;
    x.t = t + delta.template segment<6>(DPX);
    x.v = v + delta.template segment<3>(DVX);
    x.ba = ba + delta.template segment<3>(DAX);
    x.bg = bg + delta.template segment<3>(DGX);
    return x;
  }


  void operator+=(const Matrix<T,NUM_DOF,1> &delta)
  {
    *this = *this + delta;
  }


  Matrix<T, NUM_STATES, 1> toEigen() const
  {
    Matrix<T, NUM_STATES, 1> x;
    x << t.p(), t.q().toEigen(), v, ba, bg;
    return x;
  }

  common::Transform<T> t;
  Matrix<T,3,1> v;
  Matrix<T,3,1> ba;
  Matrix<T,3,1> bg;

};


template<typename T>
void dynamics(const State<T> &x, const Matrix<T,NUM_INPUTS,1> &imu, Matrix<T,NUM_DOF,1> &xdot)
{
  xdot.template segment<3>(DPX) = x.v;
  xdot.template segment<3>(DQX) = imu.template segment<3>(UGX) - x.bg;
  xdot.template segment<3>(DVX) = imu.template segment<3>(UAX) - x.ba + T(common::gravity) * x.t.q().rot(common::e3.cast<T>()) -
                                 (imu.template segment<3>(UGX) - x.bg).cross(x.v);
  xdot.template segment<3>(DGX).setZero();
  xdot.template segment<3>(DAX).setZero();
}


// Dynamics with noise for input noise Jacobian calculation
template<typename T>
void dynamics(const State<T> &x, const Matrix<T,NUM_INPUTS,1> &imu, const Matrix<T,NUM_INPUTS,1> &noise, Matrix<T,NUM_DOF,1> &xdot)
{
  xdot.template segment<3>(DPX) = x.v;
  xdot.template segment<3>(DQX) = imu.template segment<3>(UGX) - x.bg - noise.template segment<3>(UGX);
  xdot.template segment<3>(DVX) = imu.template segment<3>(UAX) - x.ba - noise.template segment<3>(UAX) + T(common::gravity) * x.t.q().rot(common::e3.cast<T>()) -
                                 (imu.template segment<3>(UGX) - x.bg - noise.template segment<3>(UGX)).cross(x.v);
  xdot.template segment<3>(DGX).setZero();
  xdot.template segment<3>(DAX).setZero();
}


// Model for relative translation direction and rotation
template<typename T>
void rel_pose_model(const State<T> &x, const Matrix<T,3,1>& omega_sum, const T& dt,
                    const common::Quaternion<T>& q_bc, const Matrix<T,3,1>& p_bc,
                    common::Quaternion<T>& ht, common::Quaternion<T>& hq)
{
  // Compute translation direction model
  static Matrix<T,3,1> pt, t, t_x_e3, at;
  pt = -q_bc.rot(x.t.q().rot(x.t.p()) + p_bc);
  t = pt / pt.norm();
  t_x_e3 = t.cross(common::e3);
  T tT_e3 = common::saturate<T>(t.dot(common::e3.cast<T>()), T(1.0), T(-1.0));
  at = acos(tT_e3) * t_x_e3 / t_x_e3.norm();
  ht = common::Quaternion<T>::exp(at);

  // Compute rotation model
  hq = q_bc.inv() * x.t.q().inv() * (x.t.q() + Matrix<T,3,1>(-omega_sum + x.bg * dt)) * q_bc;
}


// Derivative of q + delta w.r.t. delta
template<typename T>
Matrix<T,4,3> dqpd_dd(const Matrix<T,4,1>& q)
{
  Matrix<T,4,3> m;
  m << -q(1), -q(2), -q(3),
        q(0), -q(3),  q(2),
        q(3),  q(0), -q(1),
       -q(2),  q(1),  q(0);
  m *= 0.5;
  return m;
}


// Derivative of transform + delta w.r.t. delta
template<typename T>
Matrix<T,7,6> dTpd_dd(const Matrix<T,7,1>& t)
{
  common::Transform<T> T_(t);
  Matrix<T,7,6> m;
  m.setZero();
  m.template block<3,3>(0,0) = T_.q().inv().R();
  m.template block<4,3>(3,3) = dqpd_dd(Matrix<T,4,1>(t.template segment<4>(3)));
  return m;
}


// Derivative of qz + delta w.r.t. delta
template<typename T>
Matrix<T,4,2> dqzpd_dd(const Matrix<T,4,1>& q)
{
  // Unpacking and precalcs
  T w = q(0);
  T x = q(1);
  T y = q(2);
  T z = q(3);
  T ww = w * w;
  T xx = x * x;
  T yy = y * y;
  T zz = z * z;
  T wxyz = ww + xx + yy + zz;

  Matrix<T,4,2> m;
  m << x * (0.5 - wxyz), y * (0.5 - wxyz),
       w * (wxyz - 0.5),         -0.5 * z,
                0.5 * z, w * (wxyz - 0.5),
               -0.5 * y,          0.5 * x;
  return m;
}


// Derivative of state + state_delta w.r.t. state_delta
template<typename T>
Matrix<T,NUM_STATES,NUM_DOF> dxpd_dd(const Matrix<T,NUM_STATES,1>& x)
{
  Matrix<T,NUM_STATES,NUM_DOF> dx;
  dx.setZero();
  dx.template block<7,6>(PX,DPX) = dTpd_dd<T>(x.template segment<7>(PX));
  dx.template block<3,3>(VX,DVX).setIdentity();
  dx.template block<3,3>(AX,DAX).setIdentity();
  dx.template block<3,3>(GX,DGX).setIdentity();
  return dx;
}


// Struct for calculation of Jacobian of dynamics w.r.t. the state
struct StateDot
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  StateDot(const Matrix<double,NUM_INPUTS,1>& _imu)
    : imu(_imu) {}

  template <typename T>
  bool operator()(const T* const _x, const T* const _n, T* residual) const
  {
    // Map state and noise
    const State<T> x(_x);
    const Matrix<T,NUM_INPUTS,1> n(_n);

    // Compute output
    Map<Matrix<T,NUM_DOF,1>> dx(residual);
    Matrix<T,NUM_DOF,1> dx_;
    dynamics<T>(x, imu.cast<T>(), n, dx_);
    dx = dx_;
    return true;
  }

private:

  const Matrix<double,NUM_INPUTS,1> imu;

};


// Struct for calculation of Jacobian of the camera translation model w.r.t. the state
struct RelPoseModel
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RelPoseModel(const Vector3d& _omega_sum, const double& _dt,
               const common::Quaterniond& _q_bc, const Vector3d& _p_bc)
    : omega_sum(_omega_sum), dt(_dt), q_bc(_q_bc), p_bc(_p_bc) {}

  template <typename T>
  bool operator()(const T* const _x, T* residual) const
  {
    // Map state
    const State<T> x(_x);

    // Compute output
    Map<Matrix<T,2,1>> h(residual);
    common::Quaternion<T> ht, hq;
    rel_pose_model(x, omega_sum.cast<T>(), T(dt), q_bc.cast<T>(), p_bc.cast<T>(), ht, hq);
    h.template segment<4>(0) = ht;
    h.template segment<4>(4) = hq;
    return true;
  }

private:

  const double dt;
  const Vector3d omega_sum, p_bc;
  const common::Quaterniond q_bc;

};


// Use automatic differentiation to calculate the Jacobian of state dynamics w.r.t. minimal state and input noise
template<typename T>
void getFG(const Matrix<T,NUM_STATES,1>& x, const Matrix<T,6,1>& imu,
                 Matrix<T,NUM_DOF,NUM_DOF>& F, Matrix<T,NUM_DOF,6>& G)
{
  // Calculate autodiff Jacobian
  static Matrix<T,NUM_INPUTS,1> n = Matrix<T,6,1>::Zero();
  T const* parameters[2];
  parameters[0] = x.data();
  parameters[1] = n.data();
  Matrix<T,NUM_DOF,1> r;
  Matrix<T,NUM_DOF,NUM_STATES,RowMajor> J_autodiff_wrt_x;
  Matrix<T,NUM_DOF,6,RowMajor> J_autodiff_wrt_noise;
  T* J_autodiff_ptr_ptr[2];
  J_autodiff_ptr_ptr[0] = J_autodiff_wrt_x.data();
  J_autodiff_ptr_ptr[1] = J_autodiff_wrt_noise.data();
  ceres::AutoDiffCostFunction<StateDot, NUM_DOF, NUM_STATES, NUM_INPUTS> cost_function(new StateDot(imu));
  cost_function.Evaluate(parameters, r.data(), J_autodiff_ptr_ptr);

  // Convert autodiff Jacobian to minimal Jacobian
  F = J_autodiff_wrt_x * dxpd_dd<T>(x);
  G = J_autodiff_wrt_noise;
}


// Use automatic differentiation to calculate the Jacobian of relative pose model w.r.t. minimal state
template<typename T>
void getH_vo(const Matrix<T,NUM_STATES,1>& x, const Matrix<T,6,1>& imu,
                 Matrix<T,5,NUM_DOF>& H)
{
  // Calculate autodiff Jacobian
  T const* parameters[1];
  parameters[0] = x.data();
  Matrix<T,8,1> r;
  Matrix<T,8,NUM_STATES,RowMajor> J;
  T* J_autodiff_ptr_ptr[2];
  J_autodiff_ptr_ptr[0] = J_autodiff_wrt_x.data();
  J_autodiff_ptr_ptr[1] = J_autodiff_wrt_noise.data();
  ceres::AutoDiffCostFunction<StateDot, NUM_DOF, NUM_STATES, NUM_INPUTS> cost_function(new StateDot(imu));
  cost_function.Evaluate(parameters, r.data(), J_autodiff_ptr_ptr);

  // Convert autodiff Jacobian to minimal Jacobian
  H = J_autodiff_wrt_x * dxpd_dd<T>(x);
}


// Constants
static const dxMatrix I_num_dof_ = [] {
dxMatrix tmp;
tmp.setIdentity();
return tmp;
}();

static const dxVector ones_vec_ = [] {
dxVector tmp;
tmp.setOnes();
return tmp;
}();


class EKF
{

public:

  EKF();
  EKF(string filename);
  ~EKF();

  void load(const string &filename);
  void run(const double &t, const sensors::Sensors &sensors);
  static void imageH(common::Quaternion<double> &ht, common::Quaternion<double> &hq, Matrix<double, 5, NUM_DOF> &H, const State<double> &x,
                     const common::Quaternion<double> &q_bc, const Vector3d &p_bc, const common::Quaternion<double> &q_ik,
                     const Vector3d &p_ik);
  const xVector getState() const { return x_.toEigen(); }
  vehicle::State getVehicleState() const;

private:

  void log(const double &t);
  void propagate(const double &t, const Vector3d &gyro, const Vector3d &acc);
  void imageUpdate();
  bool trackFeatures(const vector<Vector3d, aligned_allocator<Vector3d> > &pts);
  void optimizePose(common::Quaternion<double>& q, common::Quaternion<double>& qt,
                    const vector<Vector3d, aligned_allocator<Vector3d> >& e1,
                    const vector<Vector3d, aligned_allocator<Vector3d> >& e2,
                    const unsigned &iters);
  void optimizePose2(Matrix3d& R, Matrix3d& Rt,
                     const vector<Vector3d, aligned_allocator<Vector3d> >& e1,
                     const vector<Vector3d, aligned_allocator<Vector3d> >& e2,
                     const unsigned &iters);
  void se(double& err, const Vector3d& e1, const Vector3d& e2, const Matrix3d& E);
  void dse(double& derr, const Vector3d& e1, const Vector3d& e2, const Matrix3d& E, const Matrix3d& dE);

  // Primary EKF variables
  double t_prev_;
  State<double> x_;
  dxVector xdot_;
  dxMatrix P_, F_, A_;
  dxMatrix Qx_;
  Matrix<double, NUM_DOF, NUM_INPUTS> G_, B_;
  Matrix<double, NUM_INPUTS, NUM_INPUTS> Qu_;
  Matrix<double, 5, NUM_DOF> H_vo_;
  Matrix<double, 5, 5> R_vo_;
  dxVector lambda_;
  dxMatrix Lambda_;
  double vo_meas_gate_upper_, vo_meas_gate_lower_;
  double q_time_avg_, R_time_avg_, ceres_time_avg_, nn_;

  // Storage for current IMU measurement
  uVector imu_;

  // Keyframe and image update data
  double pixel_disparity_threshold_; // Threshold to allow relative pose optimization
  Vector3d pk_; // Keyframe inertial position
  common::Quaternion<double> qk_; // Keyframe body attitude
  vector<Vector3d, aligned_allocator<Vector3d> > pts_k_; // Keyframe image points
  vector<Vector2d, aligned_allocator<Vector2d> > pts_match_;
  vector<Vector2d, aligned_allocator<Vector2d> > pts_match_k_;
  vector<Vector3d, aligned_allocator<Vector3d> > dv_, dv_k_; // Landmark direction vectors

  // Camera parameters
  common::Quaternion<double> q_bc_, q_bu_;
  Vector3d p_bc_, p_bu_;
  Matrix3d K_, K_inv_;

  // Feature tracking parameters
  int max_tracked_features_, min_kf_feature_matches_;
  vector<Vector3d, aligned_allocator<Vector3d> > tracked_pts_, new_tracked_pts_;

  // Logging
  string directory_;
  ofstream state_log_;
  ofstream cov_log_;

  // Ceres
  struct SampsonError
  {
    SampsonError(const Vector3d _ek, const Vector3d _ec)
        : ek(_ek), ec(_ec) {}

    template <typename T>
    bool operator()(const T* const _R, const T* const _Rt, T* residual) const
    {
      // Map inputs to Eigen matrices
      Matrix<T,3,3> R = Map<const Matrix<T,3,3>>(_R);
      Matrix<T,3,3> Rt = Map<const Matrix<T,3,3>>(_Rt);

      // Construct residual
      const Matrix<T,3,1> t = Rt * common::e3.cast<T>();
      const Matrix<T,3,3> E = R * common::skew(t);
      const Matrix<T,1,3> ekT_E = ek.cast<T>().transpose() * E;
      const Matrix<T,3,1> E_ec = E * ec.cast<T>();
      const T ekT_E_ec = ek.cast<T>().transpose() * E_ec;
      residual[0] = ekT_E_ec / sqrt(ekT_E(0) * ekT_E(0) + ekT_E(1) * ekT_E(1) + E_ec(0) * E_ec(0) + E_ec(1) * E_ec(1));
      return true;
    }

  private:

    const Vector3d ek, ec;

  };

  struct SO3Plus
  {
    template<typename T>
    bool operator()(const T *x, const T *delta, T *x_plus_delta) const
    {
      Map<const Matrix<T,3,3>> _R(x);
      Map<const Matrix<T,3,1>> dR(delta);

      Map<Matrix<T,3,3>> R(x_plus_delta);
      R = common::expR(common::skew(Matrix<T,3,1>(-dR))) * _R;
      return true;
    }
  };

  struct S2Plus
  {
    template<typename T>
    bool operator()(const T *x, const T *delta, T *x_plus_delta) const
    {
      Map<const Matrix<T,3,3>> _R(x);
      Map<const Matrix<T,2,1>> dR(delta);

      Map<Matrix<T,3,3>> R(x_plus_delta);
      R = common::expR(common::skew(Matrix<T,3,1>(-_R * common::I_2x3.cast<T>().transpose() * dR))) * _R;
      return true;
    }
  };

};


}
