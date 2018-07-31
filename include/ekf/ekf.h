#include <eigen3/Eigen/Eigen>
#include <chrono>
#include "common_cpp/common.h"
#include "sensors.h"


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
  GX,
  GY,
  GZ,
  AX,
  AY,
  AZ,
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
  DGX,
  DGY,
  DGZ,
  DAX,
  DAY,
  DAZ,
  NUM_DOF
};

// Input indices
enum
{
  UAX,
  UAY,
  UAZ,
  UWX,
  UWY,
  UWZ,
  NUM_INPUTS
};

typedef Eigen::Matrix<double, NUM_STATES, 1> xVector;
typedef Eigen::Matrix<double, NUM_DOF, 1> dxVector;
typedef Eigen::Matrix<double, NUM_DOF, NUM_DOF> dxMatrix;
typedef Eigen::Matrix<double, NUM_INPUTS, 1> uVector;


struct State
{

  State();
  State(const xVector &x0);
  Eigen::Vector3d p;
  common::Quaternion q;
  Eigen::Vector3d v;
  Eigen::Vector3d bg;
  Eigen::Vector3d ba;

  State operator+(const dxVector &delta) const;
  void operator+=(const dxVector &delta);
  Eigen::Matrix<double, NUM_STATES, 1> toEigen() const;

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
  EKF(std::string filename);
  ~EKF();

  void load(const std::string &filename);
  void run(const double &t, const sensors::Sensors &sensors);
  static void f(dxVector &xdot, const State &x, const Eigen::Vector3d &gyro, const Eigen::Vector3d &acc);
  static void getF(dxMatrix &F, const State &x, const Eigen::Vector3d &gyro);
  static void getG(Eigen::Matrix<double, NUM_DOF, NUM_INPUTS> &G, const State &x);
  static void imageH(common::Quaternion &ht, common::Quaternion &hq, Eigen::Matrix<double, 5, NUM_DOF> &H, const State &x,
                     const common::Quaternion &q_bc, const Eigen::Vector3d &p_bc, const common::Quaternion &q_ik,
                     const Eigen::Vector3d &p_ik);

private:

  void log(const double &t);
  void propagate(const double &t, const Eigen::Vector3d &gyro, const Eigen::Vector3d &acc);
  void imageUpdate();
  bool trackFeatures(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &pts);
  void optimizePose(common::Quaternion& q, common::Quaternion& qt,
                    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& e1,
                    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& e2,
                    const unsigned &iters);
  void se(double& err, const Eigen::Vector3d& e1, const Eigen::Vector3d& e2, const Eigen::Matrix3d& E);
  void dse(double& derr, const Eigen::Vector3d& e1, const Eigen::Vector3d& e2, const Eigen::Matrix3d& E, const Eigen::Matrix3d& dE);

  // Primary EKF variables
  double t_prev_;
  State x_;
  dxVector xdot_;
  dxMatrix P_, F_;
  dxMatrix Qx_;
  Eigen::Matrix<double, NUM_DOF, NUM_INPUTS> G_;
  Eigen::Matrix<double, NUM_INPUTS, NUM_INPUTS> Qu_;
  Eigen::Matrix<double, 5, NUM_DOF> H_vo_;
  Eigen::Matrix<double, 5, 5> R_vo_;
  dxVector lambda_;
  dxMatrix Lambda_;

  // Keyframe and image update data
  double pixel_disparity_threshold_; // Threshold to allow relative pose optimization
  Eigen::Vector3d pk_; // Keyframe inertial position
  common::Quaternion qk_; // Keyframe body attitude
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > pts_k_; // Keyframe image points
  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > pts_match_;
  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > pts_match_k_;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > dv_, dv_k_; // Landmark direction vectors

  // Camera parameters
  common::Quaternion q_bc_;
  Eigen::Vector3d p_bc_;
  Eigen::Matrix3d K_, K_inv_;

  // Feature tracking parameters
  int max_tracked_features_, min_kf_feature_matches_;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > tracked_pts_, new_tracked_pts_;

  // Logging
  std::string directory_;
  std::ofstream state_log_;
  std::ofstream cov_log_;

};


}
