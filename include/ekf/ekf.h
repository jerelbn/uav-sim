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
  GX, GY, GZ,
  AX, AY, AZ,
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
  DGX, DGY, DGZ,
  DAX, DAY, DAZ,
  DKPX, DKPY, DKPZ,
  DKQX, DKQY, DKQZ,
  NUM_DOF
};

// Input indices
enum
{
  UWX, UWY, UWZ,
  UAX, UAY, UAZ,
  NUM_INPUTS
};

typedef Matrix<double, NUM_STATES, 1> xVector;
typedef Matrix<double, NUM_DOF, 1> dxVector;
typedef Matrix<double, NUM_DOF, NUM_DOF> dxMatrix;
typedef Matrix<double, NUM_DOF, NUM_INPUTS> gMatrix;
typedef Matrix<double, NUM_INPUTS, 1> uVector;


struct State
{

  State();
  State(const xVector &x0);
  Vector3d p;
  common::Quaterniond q;
  Vector3d v;
  Vector3d bg;
  Vector3d ba;
  Vector3d pk;
  common::Quaterniond qk;

  State operator+(const dxVector &delta) const;
  dxVector operator-(const State &x2) const;
  void operator+=(const dxVector &delta);
  Matrix<double, NUM_STATES, 1> toEigen() const;
  Matrix<double, NUM_DOF, 1> minimal() const;

};


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
  static void f(const State &x, const Vector3d &gyro, const Vector3d &acc, dxVector &xdot);
  static void getFG(const State &x, const Vector3d &gyro, dxMatrix &F, gMatrix &G);
  static void getH(const State &x, const common::Quaterniond &q_bc, const Vector3d &p_bc,
                   common::Quaterniond &ht, common::Quaterniond &hq, Matrix<double, 5, NUM_DOF> &H);
  static void keyframeReset(State &x, dxMatrix &P);
  static void getN(const State &x, dxMatrix &N);
  const xVector getState() const { return x_.toEigen(); }
  vehicle::State getVehicleState() const;

private:

  void log(const double &t);
  void propagate(const double &t, const Vector3d &gyro, const Vector3d &acc);
  void imageUpdate();
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
  State x_;
  dxVector xdot_;
  dxMatrix P_, F_, A_;
  dxMatrix Qx_;
  gMatrix G_, B_;
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
  common::Quaterniond qk_; // Keyframe body attitude
  vector<Vector3d, aligned_allocator<Vector3d> > pts_k_; // Keyframe image points
  vector<Vector2d, aligned_allocator<Vector2d> > pts_match_;
  vector<Vector2d, aligned_allocator<Vector2d> > pts_match_k_;
  vector<Vector3d, aligned_allocator<Vector3d> > dv_, dv_k_; // Landmark direction vectors

  // Camera parameters
  common::Quaterniond q_bc_, q_bu_;
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

}; // EKF


}
