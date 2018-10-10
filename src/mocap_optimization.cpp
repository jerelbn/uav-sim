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

// Print vector of eigen vectors
void print_state(const string& name, const state_vector& v,
                 const int& start, const int& end)
{
  cout << name << " = \n";
  MatrixXd x_print(v[0].rows(),end-start);
  for (int i = 0; i < end-start; ++i)
  {
    x_print.col(i) = v[i+start];
  }
  cout << x_print << endl;
}


// Local parameterization for Quaternions
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


template<typename T>
void state_dynamics(const Matrix<T,3,1>& p, const Matrix<T,3,1>& v, const common::Quaternion<T>& q,
                    const Matrix<T,3,1>& ba, const Matrix<T,3,1>& bg, const T& cd,
                    const Matrix<T,3,1>& acc, const Matrix<T,3,1>& omega,
                    Matrix<T,DELTA_STATE_SIZE,1>& dx)
{
  // Constants
  static Matrix<T,3,3> E3(common::e3.cast<T>() * common::e3.transpose().cast<T>());
  static Matrix<T,3,3> IE3(common::I_3x3.cast<T>() - common::e3.cast<T>() * common::e3.transpose().cast<T>());
  static Matrix<T,3,1> e3(common::e3.cast<T>());
  static T g = T(common::gravity);

  // Pack output
  dx.setZero();
  dx.template segment<3>(DPX) = v;
  dx.template segment<3>(DVX) = q.inv().rot(E3 * (acc - ba) - cd * IE3 * q.rot(v)) + g * e3;
  dx.template segment<3>(DQX) = omega - bg;
}


struct MocapFactor
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MocapFactor(const Vector3d& p_meas, const Vector4d& q_meas)
      : p_meas_(p_meas), q_meas_(q_meas) {}

  template <typename T>
  bool operator()(const T* const x, T* residuals) const
  {
    Map<const Matrix<T,3,1>> p(x+PX);
    const common::Quaternion<T> q(x+QW);
    Map<Matrix<T,6,1>> residuals_(residuals);
    residuals_.template segment<3>(0) = p_meas_.cast<T>() - p;
    residuals_.template segment<3>(3) = q_meas_.cast<T>() - q;
    return true;
  }

private:

  const Vector3d p_meas_;
  const common::Quaterniond q_meas_;

};


struct PropagationFactor
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PropagationFactor(const vector<double>& _dts, const vector<Vector3d>& _accs, const vector<Vector3d>& _gyros)
      : dts(_dts), accs(_accs), gyros(_gyros), E3(common::e3 * common::e3.transpose()),
        IE3(common::I_3x3 - common::e3 * common::e3.transpose()) {}

  template <typename T>
  bool operator()(const T* const x1, const T* const x2, const T* const cd, T* residuals) const
  {
    // Copy previous state
    Matrix<T,3,1> p2hat(x1+PX);
    Matrix<T,3,1> v2hat(x1+VX);
    common::Quaternion<T> q2hat(x1+QW);
    Matrix<T,3,1> ba2hat(x1+AX);
    Matrix<T,3,1> bg2hat(x1+GX);

    // Map current state
    Map<const Matrix<T,3,1>> p2(x2+PX);
    Map<const Matrix<T,3,1>> v2(x2+VX);
    const common::Quaternion<T> q2(x2+QW);
    Map<const Matrix<T,3,1>> ba2(x2+AX);
    Map<const Matrix<T,3,1>> bg2(x2+GX);

    // Predict current state with IMU measurements
    Matrix<T,DELTA_STATE_SIZE,1> dx;
    for (int i = 0; i < dts.size(); ++i)
    {
      state_dynamics<T>(p2hat, v2hat, q2hat, ba2hat, bg2hat, cd[0], accs[i].cast<T>(), gyros[i].cast<T>(), dx);
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
  const Matrix3d E3, IE3;

};


struct DragFactor
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DragFactor(const Vector3d& acc_meas)
      : acc_meas_(acc_meas), IE3(common::I_3x3 - common::e3 * common::e3.transpose()) {}

  template <typename T>
  bool operator()(const T* const x, const T* const cd, T* residuals) const
  {
    const common::Quaternion<T> q(x+QW);
    Map<const Matrix<T,3,1>> v(x+VX);
    Map<const Matrix<T,3,1>> ba(x+AX);
    Map<Matrix<T,2,1>> residuals_(residuals);
    residuals_ = common::I_2x3.cast<T>() * (acc_meas_.cast<T>() - (ba - cd[0] * IE3.cast<T>() * q.rot(v)));
    return true;
  }

private:

  const Vector3d acc_meas_;
  const Matrix3d IE3;

};


int main()
{
  // Load Mocap and IMU data
  // acc: [t,acc,bias,noise]
  // gyro: [t,gyro,bias,noise]
  // mocap: [t,pose,transform,noise]
  // truth: [t,pos,vel,acc,att,avel,aacc]
  MatrixXd acc = common::load_binary_to_matrix<double>("../logs/accel.bin", 10);
  MatrixXd gyro = common::load_binary_to_matrix<double>("../logs/gyro.bin", 10);
  MatrixXd mocap = common::load_binary_to_matrix<double>("../logs/mocap.bin", 21);
  MatrixXd truth = common::load_binary_to_matrix<double>("../logs/true_state.bin", 20);

  // Compute truth
  int j = 0;
  state_vector xt(mocap.cols());
  for (int i = 0; i < truth.cols(); ++i)
  {
    if (truth(0,i) == mocap(0,j)) // times happen to line up for now
    {
      common::Quaterniond q_i2b(truth.block<4,1>(10,i));
      xt[j].segment<3>(PX) = truth.block<3,1>(1,i);
      xt[j].segment<3>(VX) = q_i2b.inv().rot(truth.block<3,1>(4,i));
      xt[j].segment<4>(QW) = truth.block<4,1>(10,i);
      xt[j].segment<3>(AX) = acc.block<3,1>(4,i);
      xt[j].segment<3>(GX) = gyro.block<3,1>(4,i);
      ++j;
    }
    if (j > mocap.cols()-1) break;
  }

  // Parameters
  const int N = mocap.cols();
  const int print_start = mocap.cols()-5;
  const int print_end = mocap.cols();

  // Initialize the states with mocap
  state_vector x(N);
  for (int i = 0; i < N; ++i)
  {
    // Mocap for position/attitude
    x[i].segment<3>(PX) = mocap.block<3,1>(1,i);
    x[i].segment<4>(QW) = mocap.block<4,1>(4,i);
    x[i].segment<3>(AX).setZero(); // TODO: Initialize biases assuming constant between mocap measurements
    x[i].segment<3>(GX).setZero();

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
  }
  double cd = 0.1; // drag coefficient

  // Print initial state
  print_state("x0", x, print_start, print_end);
  cout << "cd0 = " << cd << endl;

  // Build optimization problem with Ceres-Solver
  ceres::Problem problem;

  // Add parameter blocks
  ceres::LocalParameterization *state_local_parameterization =
      new ceres::AutoDiffLocalParameterization<StatePlus,STATE_SIZE,DELTA_STATE_SIZE>;
  for (int i = 0; i < N; ++i)
    problem.AddParameterBlock(x[i].data(), STATE_SIZE, state_local_parameterization);
  problem.AddParameterBlock(&cd, 1);
  problem.SetParameterBlockConstant(&cd);

  // Add Mocap residuals
  for (int i = 0; i < N; ++i)
  {
    ceres::CostFunction* cost_function =
      new ceres::AutoDiffCostFunction<MocapFactor, 6, STATE_SIZE>(
      new MocapFactor(mocap.block<3,1>(1,i), mocap.block<4,1>(4,i)));
    problem.AddResidualBlock(cost_function, NULL, x[i].data());
  }

  // Add IMU residuals
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
      new ceres::AutoDiffCostFunction<PropagationFactor, DELTA_STATE_SIZE, STATE_SIZE, STATE_SIZE, 1>(
      new PropagationFactor(dts, accs, gyros));
    problem.AddResidualBlock(cost_function, NULL, x[i].data(), x[i+1].data(), &cd);
  }

  // Add drag residuals
  for (int i = 0; i < N; ++i)
  {
    for (int j = 0; j < acc.cols(); ++j)
    {
      ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<DragFactor, 2, STATE_SIZE, 1>(
        new DragFactor(acc.block<3,1>(1,j)));
      problem.AddResidualBlock(cost_function, NULL, x[i].data(), &cd);
      break;
    }
  }

  // Solve for the optimal rotation and translation direciton
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  options.max_num_iterations = 100;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  cout << summary.BriefReport() << "\n\n";

  // Print solution and truth
  print_state("xf", x, print_start, print_end);
  cout << "cdf = " << cd << endl;
  print_state("xt", xt, print_start, print_end);
  cout << "cdt = " << 0.1 << endl;

  return 0;
}
