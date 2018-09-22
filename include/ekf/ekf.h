#include <eigen3/Eigen/Eigen>
#include <ceres/ceres.h>
#include <chrono>
#include "common_cpp/common.h"
#include "sensors.h"
#include "vehicle_common.h"


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
  Eigen::Matrix<double, NUM_DOF, 1> minimal() const;

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
  const xVector getState() const { return x_.toEigen(); }
  vehicle::State getVehicleState() const;

private:

  void log(const double &t);
  void propagate(const double &t, const Eigen::Vector3d &gyro, const Eigen::Vector3d &acc);
  void imageUpdate();
  bool trackFeatures(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &pts);
  void optimizePose(common::Quaternion& q, common::Quaternion& qt,
                    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& e1,
                    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& e2,
                    const unsigned &iters);
  void optimizePose2(Eigen::Matrix3d& R, Eigen::Matrix3d& Rt,
                     const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& e1,
                     const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& e2,
                     const unsigned &iters);
  void optimizePose3(Eigen::Matrix3d& R, Eigen::Matrix3d& Rt,
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
  double vo_meas_gate_upper_, vo_meas_gate_lower_;
  double q_time_avg_, R_time_avg_, ceres_time_avg_, nn_;

  // Storage for current IMU measurement
  uVector imu_;

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

  // Ceres
  struct SampsonError
  {
    SampsonError(const double _ek1, const double _ek2, const double _ek3,
                 const double _ec1, const double _ec2, const double _ec3)
        : ek1(_ek1), ek2(_ek2), ek3(_ek3), ec1(_ec1), ec2(_ec2), ec3(_ec3) {}

    template <typename T>
    bool operator()(const T* const _R, const T* const _Rt, T* residual) const
    {
      const T R11 = _R[0];
      const T R21 = _R[1];
      const T R31 = _R[2];
      const T R12 = _R[3];
      const T R22 = _R[4];
      const T R32 = _R[5];
      const T R13 = _R[6];
      const T R23 = _R[7];
      const T R33 = _R[8];

      const T Rt13 = _Rt[6];
      const T Rt23 = _Rt[7];
      const T Rt33 = _Rt[8];

      residual[0] = -(ec3*(ek1*(R12*Rt13 - R11*Rt23) + ek2*(R22*Rt13 - R21*Rt23) + ek3*(R32*Rt13 - R31*Rt23)) -
                      ec2*(ek1*(R13*Rt13 - R11*Rt33) + ek2*(R23*Rt13 - R21*Rt33) + ek3*(R33*Rt13 - R31*Rt33)) +
                      ec1*(ek1*(R13*Rt23 - R12*Rt33) + ek2*(R23*Rt23 - R22*Rt33) + ek3*(R33*Rt23 - R32*Rt33)))/
                      pow(pow(ec3*(R12*Rt13 - R11*Rt23) - ec2*(R13*Rt13 - R11*Rt33) + ec1*(R13*Rt23 - R12*Rt33), 2.0) +
                          pow(ec3*(R22*Rt13 - R21*Rt23) - ec2*(R23*Rt13 - R21*Rt33) + ec1*(R23*Rt23 - R22*Rt33), 2.0) +
                          pow(ek1*(R13*Rt13 - R11*Rt33) + ek2*(R23*Rt13 - R21*Rt33) + ek3*(R33*Rt13 - R31*Rt33), 2.0) +
                          pow(ek1*(R13*Rt23 - R12*Rt33) + ek2*(R23*Rt23 - R22*Rt33) + ek3*(R33*Rt23 - R32*Rt33), 2.0), 0.5);
      return true;
    }

  private:

    const double ek1, ek2, ek3, ec1, ec2, ec3;

  };

  class ReprojectionError {
   public:
    ReprojectionError(
        const Eigen::Matrix<double, 3, 4>& projection_matrix,
        const Eigen::Vector2d& feature)
        : projection_matrix_(projection_matrix), feature_(feature) {}

    template <typename T>
    bool operator()(const T* input_point, T* reprojection_error) const {
      Eigen::Map<const Eigen::Matrix<T, 4, 1> > point(input_point);

      // Multiply the point with the projection matrix, then perform homogeneous
      // normalization to obtain the 2D pixel location of the reprojection.
      const Eigen::Matrix<T, 2, 1> reprojected_pixel =  (projection_matrix_ * input_point).hnormalized();

      // Reprojection error is the distance from the reprojection to the observed
      // feature location.
      reprojection_error[0] = feature_[0] - reprojected_pixel[0];
      reprojection_error[1] = feature_[1] - reprojected_pixel[1];
      return true;
    }

   private:
    const Eigen::Matrix<double, 3, 4>& projection_matrix_;
    const Eigen::Vector2d& feature_;
  };

  class SO3LocalParameterization : public ceres::LocalParameterization
  {
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
    virtual bool ComputeJacobian(const double *x, double *jacobian) const;
    virtual int GlobalSize() const { return 9; }
    virtual int LocalSize() const { return 3; }
  };

  class S2LocalParameterization : public ceres::LocalParameterization
  {
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
    virtual bool ComputeJacobian(const double *x, double *jacobian) const;
    virtual int GlobalSize() const { return 9; }
    virtual int LocalSize() const { return 2; }
  };

};


}
