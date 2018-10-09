#include "common_cpp/common.h"
#include <fstream>
#include <iostream>
#include <ceres/ceres.h>

using namespace std;
using namespace Eigen;

enum {PX, PY, PZ, VX, VY, VZ, QW, QX, QY, QZ, STATE_SIZE};
typedef Matrix<double, STATE_SIZE, 1> state;


// Local parameterization for Quaternions
struct StatePlus
{
  template<typename T>
  bool operator()(const T* x1, const T* delta, T* x2) const
  {
    Map<const Matrix<T,3,1>> p(x1+PX);
    Map<const Matrix<T,3,1>> v(x1+VX);
    const common::Quaternion<T> q(x1+QW);
    Map<const Matrix<T,9,1>> delta_(delta);
    Map<Matrix<T,STATE_SIZE,1>> x2_(x2);
    x2_.template segment<3>(PX) = p + delta_.template segment<3>(PX);
    x2_.template segment<3>(VX) = v + delta_.template segment<3>(VX);
    x2_.template segment<4>(QW) = (q + Matrix<T,3,1>(delta_.template segment<3>(QW))).toEigen();
    return true;
  }
};


struct MocapError
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MocapError(const Vector3d& p_meas, const Vector4d& q_meas)
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


struct ImuError
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ImuError(const double& dt, const Vector3d& acc1, const Vector3d& gyro1)
      : dt_(dt), acc1_(acc1), gyro1_(gyro1) {}

  template <typename T>
  bool operator()(const T* const x1, const T* const x2, T* residuals) const
  {
    // Map previous state
    Map<const Matrix<T,3,1>> p1(x1+PX);
    Map<const Matrix<T,3,1>> v1(x1+VX);
    const common::Quaternion<T> q1(x1+QW);

    // Map current state
    Map<const Matrix<T,3,1>> p2(x2+PX);
    Map<const Matrix<T,3,1>> v2(x2+VX);
    const common::Quaternion<T> q2(x2+QW);

    // Predict current state with IMU
    Matrix<T,3,1> p2hat = p1 + v1 * T(dt_);
    Matrix<T,3,1> v2hat = v1 + (q1.inv().rot(acc1_.cast<T>()) + T(common::gravity) * common::e3.cast<T>()) * T(dt_);
    common::Quaternion<T> q2hat = q1 + Matrix<T,3,1>(gyro1_.cast<T>() * T(dt_));

    // Map output and fill it with new values
    Map<Matrix<T,STATE_SIZE-1,1>> residuals_(residuals);
    residuals_.template segment<3>(PX) = p2 - p2hat;
    residuals_.template segment<3>(VX) = v2 - v2hat;
    residuals_.template segment<3>(QW) = q2 - q2hat;
    return true;
  }

private:

  const double dt_;
  const Vector3d acc1_, gyro1_;

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

  const int N = 50;
  const int print_size = 10;
  const int print_start = 40;

  // Define the states
  vector<Matrix<double, STATE_SIZE, 1>> x(N);
  for (int i = 0; i < N; ++i)
  {
    if (i == 0)
    {
      x[i].setZero();
      x[i](QW) = 1;
    }
    else
    {
      double dt = acc(0,i) - acc(0,i-1);
      Vector3d p = x[i-1].segment<3>(PX);
      Vector3d v = x[i-1].segment<3>(VX);
      common::Quaterniond q(x[i-1].segment<4>(QW));
      x[i].segment<3>(PX) = p + v * dt;
      x[i].segment<3>(VX) = v + (q.rot(acc.block<3,1>(1,i-1)) + common::gravity * common::e3) * dt;
      x[i].segment<4>(QW) = (q + Vector3d(gyro.block<3,1>(1,i-1) * dt)).toEigen();
    }
  }

  // Print initial state
  cout << "x0 = \n";
  Matrix<double,STATE_SIZE,print_size> x_print;
  for (int i = 0; i < print_size; ++i)
  {
    x_print.block<3,1>(PX,i) = x[i+print_start].segment<3>(PX);
    x_print.block<3,1>(VX,i) = x[i+print_start].segment<3>(VX);
    x_print.block<4,1>(QW,i) = x[i+print_start].segment<4>(QW);
  }
  cout << x_print << "\n\n";

  // Build optimization problem with Ceres-Solver
  ceres::Problem problem;

  // Add parameter blocks
  ceres::LocalParameterization *state_local_parameterization =
      new ceres::AutoDiffLocalParameterization<StatePlus,STATE_SIZE,STATE_SIZE-1>;
  for (int i = 0; i < N; ++i)
    problem.AddParameterBlock(x[i].data(), STATE_SIZE, state_local_parameterization);

  // Add Mocap residuals
  for (int i = 0; i < N; ++i)
  {
    ceres::CostFunction* cost_function =
      new ceres::AutoDiffCostFunction<MocapError, 6, STATE_SIZE>(
      new MocapError(truth.block<3,1>(1,i), truth.block<4,1>(10,i)));
    problem.AddResidualBlock(cost_function, NULL, x[i].data());
  }

  // Add IMU residuals
  for (int i = 0; i < N-1; ++i)
  {
    double dt = acc(0,i+1)-acc(0,i);
    ceres::CostFunction* cost_function =
      new ceres::AutoDiffCostFunction<ImuError, STATE_SIZE-1, STATE_SIZE, STATE_SIZE>(
      new ImuError(dt, acc.block<3,1>(1,i), gyro.block<3,1>(1,i)));
    problem.AddResidualBlock(cost_function, NULL, x[i].data(), x[i+1].data());
  }

  // Solve for the optimal rotation and translation direciton
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  options.max_num_iterations = 100;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  cout << summary.BriefReport() << "\n\n";

  // Print solution and truth
  cout << "xf = \n";
  for (int i = 0; i < print_size; ++i)
  {
    x_print.block<3,1>(PX,i) = x[i+print_start].segment<3>(PX);
    x_print.block<3,1>(VX,i) = x[i+print_start].segment<3>(VX);
    x_print.block<4,1>(QW,i) = x[i+print_start].segment<4>(QW);
  }
  cout << x_print << "\n\n";

  cout << "xt = \n";
  for (int i = 0; i < print_size; ++i)
  {
    common::Quaterniond q_i2b(truth.block<4,1>(10,i+print_start));
    x_print.block<3,1>(PX,i) = truth.block<3,1>(1,i+print_start);
    x_print.block<3,1>(VX,i) = q_i2b.inv().rot(truth.block<3,1>(4,i+print_start));
    x_print.block<4,1>(QW,i) = truth.block<4,1>(10,i+print_start);
  }
  cout << x_print << endl;

  return 0;
}
