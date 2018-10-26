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
  PX, PY, PZ,
  QW, QX, QY, QZ,
  VX, VY, VZ,
  AX, AY, AZ,
  GX, GY, GZ,
  KPX, KPY, KPZ,
  KQW, KQX, KQY, KQZ,
  NUM_STATES
};

// Derivative indices
enum
{

  DPX, DPY, DPZ,
  DQX, DQY, DQZ,
  DVX, DVY, DVZ,
  DAX, DAY, DAZ,
  DGX, DGY, DGZ,
  DKPX, DKPY, DKPZ,
  DKQX, DKQY, DKQZ,
  NUM_DOF
};

// Input indices
enum
{
  UAX, UAY, UAZ,
  UGX, UGY, UGZ,
  NUM_INPUTS
};

typedef Matrix<double, NUM_STATES, 1> xVector;
typedef Matrix<double, NUM_DOF, 1> dxVector;
typedef Matrix<double, NUM_DOF, NUM_DOF> dxMatrix;
typedef Matrix<double, NUM_INPUTS, 1> uVector;
typedef Matrix<double, NUM_DOF, NUM_INPUTS> uMatrix;


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
    tk = common::Transform<T>();
  }

  State(const State<T>& x)
  {
    t = x.t;
    v = x.v;
    ba = x.ba;
    bg = x.bg;
    tk = x.tk;
  }

  State(const Matrix<T,NUM_STATES,1> &x)
  {
    t = common::Transform<T>(x.template segment<3>(PX), x.template segment<4>(QW));
    v = x.template segment<3>(VX);
    ba = x.template segment<3>(AX);
    bg = x.template segment<3>(GX);
    tk = common::Transform<T>(x.template segment<3>(KPX), x.template segment<4>(KQW));
  }

  State(const T* ptr)
  {
    t = common::Transform<T>(ptr);
    v = Matrix<T,3,1>(ptr[VX], ptr[VY], ptr[VZ]);
    ba = Matrix<T,3,1>(ptr[AX], ptr[AY], ptr[AZ]);
    bg = Matrix<T,3,1>(ptr[GX], ptr[GY], ptr[GZ]);
    tk = common::Transform<T>(ptr+KPX);
  }

  State<T> operator+(const Matrix<T,NUM_DOF,1> &delta) const
  {
    State<T> x;
    x.t = t + delta.template segment<6>(DPX);
    x.v = v + delta.template segment<3>(DVX);
    x.ba = ba + delta.template segment<3>(DAX);
    x.bg = bg + delta.template segment<3>(DGX);
    x.tk = tk + delta.template segment<6>(DKPX);
    return x;
  }

  Matrix<T,NUM_DOF,1> operator-(const State<T> &x2) const
  {
    Matrix<T,NUM_DOF,1> dx;
    dx.template segment<6>(DPX) = t - x2.t;
    dx.template segment<3>(DVX) = v - x2.v;
    dx.template segment<3>(DAX) = ba - x2.ba;
    dx.template segment<3>(DGX) = bg - x2.bg;
    dx.template segment<6>(DKPX) = tk - x2.tk;
    return dx;
  }

  void operator+=(const Matrix<T,NUM_DOF,1> &delta)
  {
    *this = *this + delta;
  }


  Matrix<T, NUM_STATES, 1> toEigen() const
  {
    Matrix<T, NUM_STATES, 1> x;
    x << t.p(), t.q().toEigen(), v, ba, bg, tk.p(), tk.q().toEigen();
    return x;
  }

  common::Transform<T> t;
  Matrix<T,3,1> v;
  Matrix<T,3,1> ba;
  Matrix<T,3,1> bg;
  common::Transform<T> tk;

};


template<typename T>
void dynamics(const State<T> &x, const Matrix<T,NUM_INPUTS,1> &imu, Matrix<T,NUM_DOF,1> &xdot)
{
  xdot.setZero();
  xdot.template segment<3>(DPX) = x.v;
  xdot.template segment<3>(DQX) = imu.template segment<3>(UGX) - x.bg;
  xdot.template segment<3>(DVX) = imu.template segment<3>(UAX) - x.ba + T(common::gravity) * x.t.q().rot(common::e3.cast<T>()) -
                                 (imu.template segment<3>(UGX) - x.bg).cross(x.v);
}


// Dynamics with noise for input noise Jacobian calculation
template<typename T>
void dynamics(const State<T> &x, const Matrix<T,NUM_INPUTS,1> &imu, const Matrix<T,NUM_INPUTS,1> &noise, Matrix<T,NUM_DOF,1> &xdot)
{
  xdot.setZero();
  xdot.template segment<3>(DPX) = x.v;
  xdot.template segment<3>(DQX) = imu.template segment<3>(UGX) - x.bg - noise.template segment<3>(UGX);
  xdot.template segment<3>(DVX) = imu.template segment<3>(UAX) - x.ba - noise.template segment<3>(UAX) + T(common::gravity) * x.t.q().rot(common::e3.cast<T>()) -
                                 (imu.template segment<3>(UGX) - x.bg - noise.template segment<3>(UGX)).cross(x.v);
}


// Model for relative translation direction and rotation
template<typename T>
void rel_pose_model(const State<T> &x, const common::Quaternion<T>& q_bc, const Matrix<T,3,1>& p_bc,
                    common::Quaternion<T>& ht, common::Quaternion<T>& hq)
{
  // Compute translation direction model
  Matrix<T,3,1> pt = q_bc.rot(x.t.q().rot(x.tk.p() - x.t.p()) + (x.t.q().R() * x.tk.q().inv().R() - common::I_3x3) * p_bc);;
  ht = common::Quaternion<T>(pt);

  // Compute rotation model
  hq = q_bc.inv() * x.t.q().inv() * x.tk.q() * q_bc;
}


// Model for state reset
template<typename T>
void state_reset_model(State<T>& x)
{
  x.t.setP(Matrix<T,3,1>::Zero());
  x.t.setQ(common::Quaternion<T>(T(0),T(0),x.t.q().yaw()).inv() * x.t.q());
  x.tk.setP(Matrix<T,3,1>::Zero());
  x.tk.setQ(x.t.q());
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


// Derivative of dq w.r.t. q
template<typename T>
Matrix<T,3,4> ddq_dq(const Matrix<T,4,1>& q)
{
  Matrix<T,3,4> m;
  m << -q(1),  q(0),  q(3), -q(2),
       -q(2), -q(3),  q(0),  q(1),
       -q(3),  q(2), -q(1),  q(0);
  m *= 2.0;
  return m;
}


// Derivative of qz + delta w.r.t. delta
template<typename T>
Matrix<T,4,2> dqtpd_dd(const Matrix<T,4,1>& q)
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


// Derivative of dqt w.r.t. q
template<typename T>
Matrix<T,2,4> ddqt_dq(const Matrix<T,4,1>& q)
{
  return T(4.0) * dqtpd_dd(q).transpose();
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


// Derivative of dT w.r.t. T
template<typename T>
Matrix<T,6,7> ddT_dT(const Matrix<T,7,1>& t)
{
  common::Transform<T> T_(t);
  Matrix<T,6,7> m;
  m.setZero();
  m.template block<3,3>(0,0) = -T_.q().R();
  m.template block<3,4>(3,3) = ddq_dq(Matrix<T,4,1>(t.template segment<4>(3)));
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
  dx.template block<7,6>(KPX,DKPX) = dTpd_dd<T>(x.template segment<7>(KPX));
  return dx;
}


// Derivative of dstate w.r.t. state
template<typename T>
Matrix<T,NUM_DOF,NUM_STATES> ddx_dx(const Matrix<T,NUM_STATES,1>& x)
{
  Matrix<T,NUM_DOF,NUM_STATES> dx;
  dx.setZero();
  dx.template block<6,7>(DPX,PX) = ddT_dT<T>(x.template segment<7>(PX));
  dx.template block<3,3>(DVX,VX).setIdentity();
  dx.template block<3,3>(DAX,AX).setIdentity();
  dx.template block<3,3>(DGX,GX).setIdentity();
  dx.template block<6,7>(DKPX,KPX) = ddT_dT<T>(x.template segment<7>(KPX));
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


// Struct for calculation of the error measurement Jacobian w.r.t. the state
struct RelPoseModel
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RelPoseModel(const common::Quaterniond& _q_bc, const Vector3d& _p_bc)
    : q_bc(_q_bc), p_bc(_p_bc) {}

  template <typename T>
  bool operator()(const T* const _x, T* _residual) const
  {
    // Map state
    const State<T> x(_x);

    // Compute models
    common::Quaternion<T> ht, hq;
    rel_pose_model<T>(x, q_bc.cast<T>(), p_bc.cast<T>(), ht, hq);

    // Output
    Map<Matrix<T,8,1>> r(_residual);
    r.template segment<4>(0) = ht.toEigen();
    r.template segment<4>(4) = hq.toEigen();

    return true;
  }

private:

  const Vector3d p_bc;
  const common::Quaterniond q_bc;

};


// Struct for calculation of the error measurement Jacobian w.r.t. the state
struct KeyframeResetModel
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  KeyframeResetModel() {}

  template <typename T>
  bool operator()(const T* const _x, T* _residual) const
  {
    // Copy and reset the state
    State<T> x(_x);
    state_reset_model(x);

    Map<Matrix<T,NUM_STATES,1>> r(_residual);
    r = x.toEigen();

    return true;
  }
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
void getH(const Matrix<T,NUM_STATES,1>& x, const common::Quaternion<T>& ht,
          const common::Quaternion<T>& hq, const common::Quaternion<T>& q_bc,
          const Matrix<T,3,1>& p_bc, Matrix<T,5,NUM_DOF>& H)
{
  // Calculate autodiff Jacobian
  T const* parameters[1];
  parameters[0] = x.data();
  Matrix<T,8,1> r;
  Matrix<T,8,NUM_STATES,RowMajor> J_autodiff_wrt_x;
  T* J_autodiff_ptr_ptr[1];
  J_autodiff_ptr_ptr[0] = J_autodiff_wrt_x.data();
  ceres::AutoDiffCostFunction<RelPoseModel, 8, NUM_STATES> cost_function(new RelPoseModel(q_bc, p_bc));
  cost_function.Evaluate(parameters, r.data(), J_autodiff_ptr_ptr);

  // Convert autodiff Jacobian to minimal Jacobian
  Matrix<T,5,8> ddqt;
  ddqt.setZero();
  ddqt.template block<2,4>(0,0) = ddqt_dq<T>(ht.toEigen());
  ddqt.template block<3,4>(2,4) = ddq_dq<T>(hq.toEigen());
  H = ddqt * J_autodiff_wrt_x * dxpd_dd<T>(x);
}


// Use automatic differentiation to calculate the Jacobian of the reset state w.r.t. minimal state
template<typename T>
void getN(const Matrix<T,NUM_STATES,1>& x, Matrix<T,NUM_DOF,NUM_DOF>& N)
{
  // Calculate autodiff Jacobian
  T const* parameters[1];
  parameters[0] = x.data();
  Matrix<T,NUM_STATES,1> r;
  Matrix<T,NUM_STATES,NUM_STATES,RowMajor> J_autodiff_wrt_x;
  T* J_autodiff_ptr_ptr[1];
  J_autodiff_ptr_ptr[0] = J_autodiff_wrt_x.data();
  ceres::AutoDiffCostFunction<KeyframeResetModel, NUM_STATES, NUM_STATES> cost_function(new KeyframeResetModel());
  cost_function.Evaluate(parameters, r.data(), J_autodiff_ptr_ptr);

  // Convert autodiff Jacobian to minimal Jacobian
  N = ddx_dx<T>(x) * J_autodiff_wrt_x * dxpd_dd<T>(x);
}


void getN_analytical(State<double> &x, dxMatrix &N);


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
  void run(const double &t, const sensors::Sensors &sensors, const vehicle::State &x_true);
  const xVector getState() const { return x_.toEigen(); }
  vehicle::State getVehicleState() const;

private:

  void log(const double &t);
  void propagate(const double &t, const Vector3d &gyro, const Vector3d &acc);
  void imageUpdate();
  void keyframeReset(State<double> &x, dxMatrix& P);
  bool trackFeatures(const vector<Vector3d, aligned_allocator<Vector3d> > &pts);
  void optimizePose(common::Quaterniond& q, common::Quaterniond& qt,
                    const vector<Vector3d, aligned_allocator<Vector3d> >& e1,
                    const vector<Vector3d, aligned_allocator<Vector3d> >& e2,
                    const unsigned &iters);

  vehicle::State x_true_;
  Vector3d pk_true_;
  common::Quaterniond qk_true_;
  Vector3d global_node_position_;
  double global_node_heading_;
  common::Quaterniond q_global_heading_;

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
  ofstream ekf_global_pos_euler_log_;
  ofstream true_global_euler_log_;
  ofstream true_pose_b2u_log_;


  // Ceres
  struct S3Plus
  {
    template<typename T>
    bool operator()(const T* _q1, const T* _delta, T* _q2) const
    {
      common::Quaternion<T> q1(_q1);
      Map<const Matrix<T,3,1>> delta(_delta);
      Map<Matrix<T,4,1>> q2(_q2);
      q2 = (q1 + delta).toEigen();
      return true;
    }
  };


  struct S2Plus
  {
    template<typename T>
    bool operator()(const T* _q1, const T* _delta, T* _q2) const
    {
      common::Quaternion<T> q1(_q1);
      Map<const Matrix<T,2,1>> delta(_delta);
      Map<Matrix<T,4,1>> q2(_q2);
      q2 = (common::Quaternion<T>::exp(q1.proj() * delta) * q1).toEigen();
      return true;
    }
  };


  struct SampsonError
  {
    SampsonError(const Vector3d& _e1, const Vector3d& _e2)
        : e1(_e1), e2(_e2) {}

    template <typename T>
    bool operator()(const T* const _q, const T* _qt, T* residuals) const
    {
      // Map data
      common::Quaternion<T> q(_q);
      common::Quaternion<T> qt(_qt);

      // Construct residual
      Matrix<T,3,3> E = q.R() * common::skew(qt.uvec());
      Matrix<T,1,3> e1T_E = e1.cast<T>().transpose() * E;
      Matrix<T,3,1> E_e2 = E * e2.cast<T>();
      T e1T_E_e2 = e1.cast<T>().transpose() * E_e2;
      residuals[0] = e1T_E_e2 / sqrt(e1T_E(0) * e1T_E(0) + e1T_E(1) * e1T_E(1) + E_e2(0) * E_e2(0) + E_e2(1) * E_e2(1));
      return true;
    }

  private:

    const Vector3d e1, e2;

  };


};


}
