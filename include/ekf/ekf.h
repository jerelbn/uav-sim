#include <eigen3/Eigen/Eigen>
#include "common_cpp/common.h"


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

  State operator+(const dxVector &delta);
  State operator+=(const dxVector &delta);

};


class EKF
{

public:

  EKF();
  EKF(std::string filename);
  ~EKF();

  void load(const std::string filename);
  void propagate(const double t, const uVector&u);
  void imageUpdate(const Eigen::MatrixXd &pts, const Eigen::Matrix<double, 5, 5> &R);

private:

  void f(dxVector &xdot, const State &x, const uVector &u);
  void getF(dxMatrix &F, const State &x, const uVector &u);
  void getG(Eigen::Matrix<double, NUM_DOF, NUM_INPUTS> &G, const State &x);
  void imageH(Eigen::Matrix<double, 5, NUM_DOF> &H, const State &x);

  double t_prev_;
  State x_;
  dxVector xdot_;
  dxMatrix P_, F_;
  dxMatrix Qx_;
  Eigen::Matrix<double, NUM_DOF, NUM_INPUTS> G_;
  Eigen::Matrix<double, NUM_INPUTS, NUM_INPUTS> Qu_;
  Eigen::Matrix<double, 5, NUM_DOF> H_vo_;
  Eigen::Vector3d pk_; // Keyframe inertial position
  common::Quaternion qk_; // Keyframe body attitude
  common::Quaternion q_bc_;
  Eigen::Vector3d p_bc_;
  Eigen::MatrixXd pts_k_;

};


}
