#include "common_cpp/common.h"
#include <fstream>
#include <iostream>
#include <ceres/ceres.h>

using namespace std;
using namespace Eigen;

enum {PX, PY, PZ, VX, VY, VZ, QW, QX, QY, QZ, AX, AY, AZ, GX, GY, GZ, STATE_SIZE};
enum {DPX, DPY, DPZ, DVX, DVY, DVZ, DQX, DQY, DQZ, DAX, DAY, DAZ, DGX, DGY, DGZ, DELTA_STATE_SIZE};
typedef Matrix<double, STATE_SIZE, 1> State;
typedef Matrix<double, DELTA_STATE_SIZE, 1> DeltaState;
typedef vector<State,aligned_allocator<State>> state_vector;
typedef vector<Vector4d, aligned_allocator<Vector4d>> cam_pts_vector;


vector<cam_pts_vector> load_camera_data(const string& filename)
{
  // Load and organize image points
  MatrixXd camera = common::load_binary_to_matrix<double>(filename, 15001);

  // Put into vector with variable size per time step
  vector<cam_pts_vector> cam;
  for (int i = 0; i < camera.cols(); ++i)
  {
    cam_pts_vector pts;
    for (int j = 1; j < camera.rows(); j += 3)
    {
      if (camera(j,i) >= 0)
      {
        Vector4d pt;
        pt.segment<3>(0) = camera.block<3,1>(j,i);
        pt(3) = camera(0,i);
        pts.push_back(pt);
      }
    }
    cam.push_back(pts);
  }
  return cam;
}


// Logger to save initial, final, and true states for plotting in MATLAB
void log_data(const string& filename, const Matrix<double, 1, Dynamic>& t,
              const state_vector& states, const double& cd,
              const Matrix<double,7,1>& T_bm, const Matrix<double,4,1>& q_bu,
              const double& tm)
{
  ofstream logger;
  logger.open(filename);
  for (int i = 0; i < states.size(); ++i)
  {
    logger.write((char*)&t(i), sizeof(double));
    logger.write((char*)states[i].data(), states[i].rows() * sizeof(double));
    logger.write((char*)&cd, sizeof(double));
    logger.write((char*)T_bm.data(), T_bm.rows() * sizeof(double));
    logger.write((char*)q_bu.data(), q_bu.rows() * sizeof(double));
    logger.write((char*)&tm, sizeof(double));
  }
  logger.close();
  cout << "\nLogged " << filename << "\n\n";
}


// State dynamics including input noise for Jacobian calculations
template<typename T>
void dynamics(const Matrix<T,3,1>& v, const common::Quaternion<T>& q,
              const Matrix<T,3,1>& ba, const Matrix<T,3,1>& bg, const T& cd,
              const common::Quaternion<T>& q_bu,
              const Matrix<T,3,1>& acc, const Matrix<T,3,1>& omega,
              const Matrix<T,3,1>& na, const Matrix<T,3,1>& ng,
              Matrix<T,DELTA_STATE_SIZE,1>& dx)
{
  // Constants
  static Matrix<T,3,3> E3(common::e3.cast<T>() * common::e3.transpose().cast<T>());
  static Matrix<T,3,3> IE3(common::I_3x3.cast<T>() - common::e3.cast<T>() * common::e3.transpose().cast<T>());
  static Matrix<T,3,1> e3(common::e3.cast<T>());
  static T g = T(common::gravity);

  // Compute body acceleration and angular rate
  Matrix<T,3,1> omega_u = omega - bg - ng;
  Matrix<T,3,1> omega_b = q_bu.inv().rot(omega_u);
  Matrix<T,3,1> acc_b = q_bu.inv().rot(acc - ba - na);

  // Pack output
  dx.setZero();
  dx.template segment<3>(DPX) = q.inv().rot(v);
  dx.template segment<3>(DVX) = E3 * acc_b + g * q.rot(e3) - cd * IE3 * v.cwiseProduct(v) - omega_b.cross(v);
  dx.template segment<3>(DQX) = omega_b;
}


// Local parameterization for the state
struct StatePlus
{
  template<typename T>
  bool operator()(const T* x1, const T* delta, T* x2) const
  {
    Map<const Matrix<T,3,1>> p(x1+PX);
    Map<const Matrix<T,3,1>> v(x1+VX);
    const common::Quaternion<T> q(x1+QW);
    Map<const Matrix<T,3,1>> ba(x1+AX);
    Map<const Matrix<T,3,1>> bg(x1+GX);
    Map<const Matrix<T,DELTA_STATE_SIZE,1>> delta_(delta);
    Map<Matrix<T,STATE_SIZE,1>> x2_(x2);
    x2_.template segment<3>(PX) = p + delta_.template segment<3>(DPX);
    x2_.template segment<3>(VX) = v + delta_.template segment<3>(DVX);
    x2_.template segment<4>(QW) = (q + Matrix<T,3,1>(delta_.template segment<3>(DQX))).toEigen();
    x2_.template segment<3>(AX) = ba + delta_.template segment<3>(DAX);
    x2_.template segment<3>(GX) = bg + delta_.template segment<3>(DGX);
    return true;
  }
};


// Local parameterization for poses
struct PosePlus
{
  template<typename T>
  bool operator()(const T* _x1, const T* _delta, T* _x2) const
  {
    const common::Transform<T> T_(_x1);
    Map<const Matrix<T,6,1>> delta(_delta);
    Map<Matrix<T,7,1>> x2(_x2);
    common::Transform<T> T_2 = T_ + delta;
    x2.template segment<3>(0) = T_2.p();
    x2.template segment<4>(3) = T_2.q().toEigen();
    return true;
  }
};


// Local parameterization for quaternions
struct QuaternionPlus
{
  template<typename T>
  bool operator()(const T* _q1, const T* _delta, T* _q2) const
  {
    const common::Quaternion<T> q1(_q1);
    Map<const Matrix<T,3,1>> delta(_delta);
    Map<Matrix<T,4,1>> q2(_q2);
    q2 = (q1 + delta).toEigen();
    return true;
  }
};


struct PropagationFactor
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PropagationFactor(const vector<double>& _dts, const vector<Vector3d>& _accs, const vector<Vector3d>& _gyros)
      : dts(_dts), accs(_accs), gyros(_gyros) {}

  template <typename T>
  bool operator()(const T* const x1, const T* const x2, const T* const cd, const T* const _q_bu, T* residuals) const
  {
    // Constants
    static const Matrix<T,3,1> z3(T(0), T(0), T(0));

    // Copy current state
    Matrix<T,3,1> p2hat(x1+PX);
    Matrix<T,3,1> v2hat(x1+VX);
    common::Quaternion<T> q2hat(x1+QW);
    Matrix<T,3,1> ba2hat(x1+AX);
    Matrix<T,3,1> bg2hat(x1+GX);

    // Map next state
    Map<const Matrix<T,3,1>> p2(x2+PX);
    Map<const Matrix<T,3,1>> v2(x2+VX);
    const common::Quaternion<T> q2(x2+QW);
    Map<const Matrix<T,3,1>> ba2(x2+AX);
    Map<const Matrix<T,3,1>> bg2(x2+GX);

    // Map body to IMU rotation
    const common::Quaternion<T> q_bu(_q_bu);

    // Predict current state with IMU measurements
    Matrix<T,DELTA_STATE_SIZE,1> dx;
    for (int i = 0; i < dts.size(); ++i)
    {
      dynamics<T>(v2hat, q2hat, ba2hat, bg2hat, cd[0], q_bu, accs[i].cast<T>(), gyros[i].cast<T>(), z3, z3, dx);
      p2hat += dx.template segment<3>(DPX) * T(dts[i]);
      v2hat += dx.template segment<3>(DVX) * T(dts[i]);
      q2hat += dx.template segment<3>(DQX) * T(dts[i]);
    }

    // Map output and fill it with new values
    Map<Matrix<T,DELTA_STATE_SIZE,1>> residuals_(residuals);
    residuals_.template segment<3>(DPX) = p2 - p2hat;
    residuals_.template segment<3>(DVX) = v2 - v2hat;
    residuals_.template segment<3>(DQX) = q2 - q2hat;
    residuals_.template segment<3>(DAX) = ba2 - ba2hat;
    residuals_.template segment<3>(DGX) = bg2 - bg2hat;

    return true;
  }

private:

  const vector<double> dts;
  const vector<Vector3d> accs, gyros;

};


struct MocapFactor
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MocapFactor(const Vector3d& p_meas, const Vector4d& q_meas, const Vector3d& _V, const Vector3d& _Omega)
      : T_meas_(p_meas, q_meas), V(_V), Omega(_Omega) {}

  template <typename T>
  bool operator()(const T* const x, const T* const _T_bm, const T* const tm, T* residuals) const
  {
    Map<const Matrix<T,3,1>> p(x+PX);
    const common::Quaternion<T> q(x+QW);
    const common::Transform<T> T_bm(_T_bm);
    common::Transform<T> T_hat(p + q.inv().rot(T_bm.p()), q * T_bm.q());
    Map<Matrix<T,6,1>> residuals_(residuals);

    Matrix<T,6,1> delta;
    delta.template segment<3>(0) = T_bm.q().rot(V.cast<T>()) * tm[0];
    delta.template segment<3>(3) = Omega.cast<T>() * tm[0];
    common::Transform<T> T_meas_tm = T_meas_.cast<T>() + delta;

    residuals_ = Matrix<T,6,1>(T_meas_tm - T_hat);
    return true;
  }

private:

  const common::Transformd T_meas_;
  const Vector3d V, Omega;

};


//struct DragFactor
//{
//  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//  DragFactor(const Vector3d& _acc_meas, const Vector3d& _omega_meas)
//      : acc_meas(_acc_meas), omega_meas(_omega_meas) {}

//  template <typename T>
//  bool operator()(const T* const x, const T* const cd, const T* const _T_bu, T* residuals) const
//  {
//    static Matrix<T,3,3> IE3(common::I_3x3.cast<T>() -
//                             common::e3.cast<T>() * common::e3.transpose().cast<T>());
//    Map<const Matrix<T,3,1>> v(x+VX);
//    Map<const Matrix<T,3,1>> ba(x+AX);
//    Map<const Matrix<T,3,1>> bg(x+GX);
//    const common::Transform<T> T_bu(_T_bu);
//    Map<Matrix<T,2,1>> residuals_(residuals);
//    Matrix<T,3,1> omega_b = T_bu.q().inv().rot(omega_meas - bg);
//    residuals_ = common::I_2x3.cast<T>() * (T_bu.q().inv().rot(acc_meas.cast<T>() - ba) -
//                 (omega_b.cross(omega_b.cross(T_bu.p())) - cd[0] * IE3 * v.cwiseProduct(v)));
//    return true;
//  }

//private:

//  const Vector3d acc_meas, omega_meas;

//};


int main()
{
  // Load Mocap and IMU data
  // acc: [t,acc,bias,noise]
  // gyro: [t,gyro,bias,noise]
  // mocap: [t,pose,transform,noise]
  // truth: [t,pos,vel,acc,att,avel,acc]
  // cam: vector<vector[pix_x;pix_y;label;time]>
  MatrixXd acc = common::load_binary_to_matrix<double>("../logs/accel.bin", 10);
  MatrixXd gyro = common::load_binary_to_matrix<double>("../logs/gyro.bin", 10);
  MatrixXd mocap = common::load_binary_to_matrix<double>("../logs/mocap.bin", 21);
  MatrixXd truth = common::load_binary_to_matrix<double>("../logs/true_state.bin", 20);
  vector<cam_pts_vector> cam = load_camera_data("../logs/camera.bin");

  // Compute truth
  int j = 0;
  state_vector xt(mocap.cols());
  for (int i = 0; i < truth.cols(); ++i)
  {
    if (truth(0,i) == mocap(0,j)) // times happen to line up for now
    {
      xt[j].segment<3>(PX) = truth.block<3,1>(1,i);
      xt[j].segment<3>(VX) = truth.block<3,1>(4,i);
      xt[j].segment<4>(QW) = truth.block<4,1>(10,i);
      xt[j].segment<3>(AX) = acc.block<3,1>(4,i);
      xt[j].segment<3>(GX) = gyro.block<3,1>(4,i);
      ++j;
    }
    if (j > mocap.cols()-1) break;
  }
  double cd_t = 0.1;
  common::Transformd T_bm_t(Vector3d(0.1, 0.1, 0.1),Vector4d(0.9928, 0.0447, 0.0547, 0.0971));
  common::Quaterniond q_bu_t(Vector4d(0.5, 0.5, 0.5, 0.5));
  double tm_t = 0.01; // Motion capture time offset from IMU
  log_data("../logs/mocap_opt_truth.bin", mocap.row(0), xt, cd_t, T_bm_t.toEigen(), q_bu_t.toEigen(), tm_t);

  // Parameters
  const int N = mocap.cols();

  // Initialize the states with mocap
  state_vector x(N);
  for (int i = 0; i < N; ++i)
  {
    // Mocap for position/attitude
    x[i].segment<3>(PX) = mocap.block<3,1>(1,i);
    x[i].segment<4>(QW) = mocap.block<4,1>(4,i);
    x[i].segment<3>(AX) = Vector3d(0,0,0);//acc.block<3,1>(4,0); // TODO: Initialize biases assuming constant between mocap measurements
    x[i].segment<3>(GX) = Vector3d(0,0,0);//gyro.block<3,1>(4,0);

    // Compute velocity by numerical differentiation
    if (i == 0) // forward difference
      x[i].segment<3>(VX) = (mocap.block<3,1>(1,i+1) - mocap.block<3,1>(1,i)) /
                            (mocap(0,i+1) - mocap(0,i));
    else if (i == N-1) // backward difference
      x[i].segment<3>(VX) = (mocap.block<3,1>(1,i) - mocap.block<3,1>(1,i-1)) /
                            (mocap(0,i) - mocap(0,i-1));
    else // central difference
      x[i].segment<3>(VX) = (mocap.block<3,1>(1,i+1) - mocap.block<3,1>(1,i-1)) /
                            (mocap(0,i+1) - mocap(0,i-1));

    // Rotate velocity into body frame
    common::Quaterniond q_i2b(mocap.block<4,1>(4,i));
    x[i].segment<3>(VX) = q_i2b.rot(x[i].segment<3>(VX));
  }
  double cd = 0.2;
  common::Transformd T_bm(Vector3d(0.0, 0.0, 0.0),Vector4d(1.0, 0.0, 0.0, 0.0));
  common::Quaterniond q_bu;
  double tm = 0.0;
  log_data("../logs/mocap_opt_initial.bin", mocap.row(0), x, cd, T_bm.toEigen(), q_bu.toEigen(), tm);

  // Build optimization problem with Ceres-Solver
  ceres::Problem problem;

  // Add parameter blocks
  ceres::LocalParameterization *state_local_parameterization =
      new ceres::AutoDiffLocalParameterization<StatePlus,STATE_SIZE,DELTA_STATE_SIZE>;
  ceres::LocalParameterization *transform_local_parameterization =
      new ceres::AutoDiffLocalParameterization<PosePlus,7,6>;
  ceres::LocalParameterization *quaternion_local_parameterization =
      new ceres::AutoDiffLocalParameterization<QuaternionPlus,4,3>;
  for (int i = 0; i < N; ++i)
    problem.AddParameterBlock(x[i].data(), STATE_SIZE, state_local_parameterization);
  problem.AddParameterBlock(&cd, 1);
  problem.AddParameterBlock(T_bm.data(), 7, transform_local_parameterization);
  problem.AddParameterBlock(q_bu.data(), 4, quaternion_local_parameterization);

  // Add IMU factors
  for (int i = 0; i < N-1; ++i)
  {
    // create vector of delta times and measurements from current to next node
    vector<double> dts;
    vector<Vector3d> accs, gyros;
    for (int j = 0; j < acc.cols(); ++j)
    {
      // Accelerometer and gyro usually run on same time steps in an IMU
      if (acc(0,j) >= mocap(0,i) && acc(0,j) < mocap(0,i+1))
      {
        dts.push_back(acc(0,j+1) - acc(0,j));
        accs.push_back(acc.block<3,1>(1,j));
        gyros.push_back(gyro.block<3,1>(1,j));
      }
    }
    ceres::CostFunction* cost_function =
      new ceres::AutoDiffCostFunction<PropagationFactor, DELTA_STATE_SIZE, STATE_SIZE, STATE_SIZE, 1, 4>(
      new PropagationFactor(dts, accs, gyros));
    problem.AddResidualBlock(cost_function, NULL, x[i].data(), x[i+1].data(), &cd, q_bu.data());
  }

  // Add Mocap factors
  for (int i = 0; i < N; ++i)
  {
    // Compute body linear/angular velocity from mocap measurements
    Vector3d V, Omega;
    if (i == 0)
    {
      double dt = mocap(0,i+1) - mocap(0,i);
      V = (mocap.block<3,1>(1,i+1) - mocap.block<3,1>(1,i)) / dt;
      Omega = (common::Quaterniond(mocap.block<4,1>(4,i+1)) - common::Quaterniond(mocap.block<4,1>(4,i))) / dt;
    }
    else if (i == N-1)
    {
      double dt = mocap(0,i) - mocap(0,i-1);
      V = (mocap.block<3,1>(1,i) - mocap.block<3,1>(1,i-1)) / dt;
      Omega = (common::Quaterniond(mocap.block<4,1>(4,i)) - common::Quaterniond(mocap.block<4,1>(4,i-1))) / dt;
    }
    else
    {
      double dt = mocap(0,i+1) - mocap(0,i-1);
      V = (mocap.block<3,1>(1,i+1) - mocap.block<3,1>(1,i-1)) / dt;
      Omega = (common::Quaterniond(mocap.block<4,1>(4,i+1)) - common::Quaterniond(mocap.block<4,1>(4,i-1))) / dt;
    }

    // Add factor block
    ceres::CostFunction* cost_function =
      new ceres::AutoDiffCostFunction<MocapFactor, 6, STATE_SIZE, 7, 1>(
      new MocapFactor(mocap.block<3,1>(1,i), mocap.block<4,1>(4,i), V, Omega));
    problem.AddResidualBlock(cost_function, NULL, x[i].data(), T_bm.data(), &tm);
  }

//  // Add drag factors
//  for (int i = 0; i < N; ++i)
//  {
//    for (int j = 0; j < acc.cols(); ++j)
//    {
//      if (mocap(0,i) == acc(0,j))
//      {
//        Vector3d acc_mean, gyro_mean;
//        if (j < 4)
//        {
//          acc_mean = acc.block<3,5>(1,j).rowwise().mean();
//          gyro_mean = gyro.block<3,5>(1,j).rowwise().mean();
//        }
//        else if (j > acc.cols()-4)
//        {
//          acc_mean = acc.block<3,5>(1,j-4).rowwise().mean();
//          gyro_mean = gyro.block<3,5>(1,j-4).rowwise().mean();
//        }
//        else
//        {
//          acc_mean = acc.block<3,9>(1,j-4).rowwise().mean();
//          gyro_mean = gyro.block<3,9>(1,j-4).rowwise().mean();
//        }
//        ceres::CostFunction* cost_function =
//          new ceres::AutoDiffCostFunction<DragFactor, 2, STATE_SIZE, 1, 7>(
//          new DragFactor(acc_mean, gyro_mean));
//        problem.AddResidualBlock(cost_function, NULL, x[i].data(), &cd, T_bu.data());
//        break;
//      }
//    }
//  }

  // Solve for the optimal rotation and translation direction
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  options.max_num_iterations = 50;
  options.num_threads = 8;
  options.num_linear_solver_threads = 8;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  cout << summary.BriefReport() << "\n\n";

  log_data("../logs/mocap_opt_final.bin", mocap.row(0), x, cd, T_bm.toEigen(), q_bu.toEigen(), tm);

  return 0;
}
