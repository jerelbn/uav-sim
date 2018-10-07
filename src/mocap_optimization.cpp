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
    Map<const Matrix<T,9,1>> delta_(delta);
    Matrix<T,3,1> p(x1[PX], x1[PY], x1[PZ]);
    Matrix<T,3,1> v(x1[VX], x1[VY], x1[VZ]);
    common::Quaternion<T> q(x1[QW], x1[QX], x1[QY], x1[QZ]);
    p += delta_.template segment<3>(PX);
    v += delta_.template segment<3>(VX);
    q += Matrix<T,3,1>(delta_.template segment<3>(QW));
    x2[PX] = p.x();
    x2[PY] = p.y();
    x2[PZ] = p.z();
    x2[VX] = v.x();
    x2[VY] = v.y();
    x2[VZ] = v.z();
    x2[QW] = q.w();
    x2[QX] = q.x();
    x2[QY] = q.y();
    x2[QZ] = q.z();
    return true;
  }
};


struct MocapError
{
  MocapError(const double pose[7])
      : pose_{pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6]} {}

  template <typename T>
  bool operator()(const T* const x, T* residuals) const
  {
    Matrix<T,3,1> p(x[PX], x[PY], x[PZ]);
    Vector3d p_meas(pose_[0], pose_[1], pose_[2]);
    common::Quaternion<T> q(x[QW], x[QX], x[QY], x[QZ]);
    common::Quaterniond q_meas(pose_[3], pose_[4], pose_[5], pose_[6]);

    Matrix<T,3,1> p_err = p_meas.cast<T>() - p;
    Matrix<T,3,1> q_err = q_meas.cast<T>() - q;

    residuals[0] = p_err(0);
    residuals[1] = p_err(1);
    residuals[2] = p_err(2);
    residuals[3] = q_err(0);
    residuals[4] = q_err(1);
    residuals[5] = q_err(2);
    return true;
  }

private:

  const double pose_[7];

};


struct ImuError
{
  ImuError(const double& dt, const double acc1[3], const double gyro1[3])
      : dt_(dt), acc1_{acc1[0], acc1[1], acc1[2]}, gyro1_{gyro1[0], gyro1[1], gyro1[2]} {}

  template <typename T>
  bool operator()(const T* const x1, const T* const x2, T* residuals) const
  {
    // Unpack current state
    Matrix<T,3,1> p1(x1[0], x1[1], x1[2]);
    Matrix<T,3,1> v1(x1[3], x1[4], x1[5]);
    common::Quaternion<T> q1(x1[6], x1[7], x1[8], x1[9]);

    // Unpack previous state
    Matrix<T,3,1> p2(x2[0], x2[1], x2[2]);
    Matrix<T,3,1> v2(x2[3], x2[4], x2[5]);
    common::Quaternion<T> q2(x2[6], x2[7], x2[8], x2[9]);

    // Unpack measurements
    Vector3d acc1(acc1_[0], acc1_[1], acc1_[2]);
    Vector3d gyro1(gyro1_[0], gyro1_[1], gyro1_[2]);

    // Predict current state with IMU
    Matrix<T,3,1> p2hat = p1 + v1 * T(dt_);
    Matrix<T,3,1> v2hat = v1 + (q1.inv().rot(acc1.cast<T>()) + T(common::gravity) * common::e3.cast<T>()) * T(dt_);
    common::Quaternion<T> q2hat = q1 + Matrix<T,3,1>(gyro1.cast<T>() * T(dt_));

    Matrix<T,3,1> p_err = p2 - p2hat;
    Matrix<T,3,1> v_err = v2 - v2hat;
    Matrix<T,3,1> q_err = q2 - q2hat;

    residuals[0] = p_err(0);
    residuals[1] = p_err(1);
    residuals[2] = p_err(2);
    residuals[3] = v_err(0);
    residuals[4] = v_err(1);
    residuals[5] = v_err(2);
    residuals[6] = q_err(0);
    residuals[7] = q_err(1);
    residuals[8] = q_err(2);
    return true;
  }

private:

  const double dt_;
  const double acc1_[3], gyro1_[3];

};


int main()
{
  // Load Mocap and IMU data
  long acc_size, gyro_size, mocap_size, truth_size;
  double* acc_ptr = common::load_binary<double>("../logs/accel.bin", acc_size);
  double* gyro_ptr = common::load_binary<double>("../logs/gyro.bin", gyro_size);
  double* mocap_ptr = common::load_binary<double>("../logs/mocap.bin", mocap_size);
  double* truth_ptr = common::load_binary<double>("../logs/true_state.bin", truth_size);

  // Map into Eigen arrays with column for each time step
  Map<MatrixXd> acc(acc_ptr, 10, acc_size/10);
  Map<MatrixXd> gyro(gyro_ptr, 10, gyro_size/10);
  Map<MatrixXd> mocap(mocap_ptr, 21, mocap_size/21);
  Map<MatrixXd> truth(truth_ptr, 20, truth_size/20);

  // Define the states
  // First just use single IMU measurements
  // Second feed multiple measurements into the cost function to link the states
  double x[10][STATE_SIZE];
  for (int i = 0; i < 10; ++i)
  {
    if (i == 0)
    {
      x[i][PX] = 0;
      x[i][PY] = 0;
      x[i][PZ] = 0;
      x[i][VX] = 0;
      x[i][VY] = 0;
      x[i][VZ] = 0;
      x[i][QW] = 1;
      x[i][QX] = 0;
      x[i][QY] = 0;
      x[i][QZ] = 0;
    }
    else
    {
      double dt = acc(0, i) - acc(0, i-1);
      Vector3d p(x[i-1][PX], x[i-1][PY], x[i-1][PZ]);
      Vector3d v(x[i-1][VX], x[i-1][VY], x[i-1][VZ]);
      common::Quaterniond q(x[i-1][QW], x[i-1][QX], x[i-1][QY], x[i-1][QZ]);
      Vector3d pnew = p + v * dt;
      Vector3d vnew = v + (q.rot(acc.block<3,1>(0, i-1)) + common::gravity * common::e3) * dt;
      Vector4d qnew = (q + Vector3d(gyro.block<3,1>(1, i-1) * dt)).toEigen();

      x[i][PX] = pnew(0);
      x[i][PY] = pnew(1);
      x[i][PZ] = pnew(2);
      x[i][VX] = vnew(0);
      x[i][VY] = vnew(1);
      x[i][VZ] = vnew(2);
      x[i][QW] = qnew(0);
      x[i][QX] = qnew(1);
      x[i][QY] = qnew(2);
      x[i][QZ] = qnew(3);
    }
  }

  // Build optimization problem with Ceres-Solver
  ceres::Problem problem;

  // Add parameter blocks
  ceres::LocalParameterization *state_local_parameterization =
      new ceres::AutoDiffLocalParameterization<StatePlus,10,9>;
  for (int i = 0; i < 10; ++i)
    problem.AddParameterBlock(x[i], STATE_SIZE, state_local_parameterization);

  // Add Mocap residuals
  for (int i = 0; i < 10; ++i)
  {
    double pose[7]{truth(1,i), truth(2,i), truth(3,i), truth(10,i), truth(11,i), truth(12,i), truth(13,i)};
    ceres::CostFunction* cost_function =
      new ceres::AutoDiffCostFunction<MocapError, 6, 10>(new MocapError(pose));
    problem.AddResidualBlock(cost_function, NULL, x[i]);
  }

  // Add IMU residuals
  for (int i = 1; i < 10; ++i)
  {
    double dt = acc(0,i)-acc(0,i-1);
    double acc_meas[3]{acc(1,i), acc(2,i), acc(3,i)};
    double gyro_meas[3]{gyro(1,i), gyro(2,i), gyro(3,i)};
    ceres::CostFunction* cost_function =
      new ceres::AutoDiffCostFunction<ImuError, 9, STATE_SIZE, STATE_SIZE>(
      new ImuError(dt, acc_meas, gyro_meas));
    problem.AddResidualBlock(cost_function, NULL, x[i-1], x[i]);
  }

  cout << "x0 = \n";
  Matrix<double,10,10> x_print;
  for (int i = 0; i < 10; ++i)
  {
    x_print(PX,i) = x[i][PX];
    x_print(PY,i) = x[i][PY];
    x_print(PZ,i) = x[i][PZ];
    x_print(VX,i) = x[i][VX];
    x_print(VY,i) = x[i][VY];
    x_print(VZ,i) = x[i][VZ];
    x_print(QW,i) = x[i][QW];
    x_print(QX,i) = x[i][QX];
    x_print(QY,i) = x[i][QY];
    x_print(QZ,i) = x[i][QZ];
  }
  cout << x_print << "\n\n";

  // Solve for the optimal rotation and translation direciton
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  options.max_num_iterations = 100;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  cout << summary.BriefReport() << "\n\n";

  cout << "xf = \n";
  for (int i = 0; i < 10; ++i)
  {
    x_print(PX,i) = x[i][PX];
    x_print(PY,i) = x[i][PY];
    x_print(PZ,i) = x[i][PZ];
    x_print(VX,i) = x[i][VX];
    x_print(VY,i) = x[i][VY];
    x_print(VZ,i) = x[i][VZ];
    x_print(QW,i) = x[i][QW];
    x_print(QX,i) = x[i][QX];
    x_print(QY,i) = x[i][QY];
    x_print(QZ,i) = x[i][QZ];
  }
  cout << x_print << "\n\n";

  cout << "xt = \n";
  for (int i = 0; i < 10; ++i)
  {
    common::Quaterniond q_i2b(truth.block<4,1>(10,i));
    x_print.block<3,1>(PX,i) = truth.block<3,1>(1,i);
    x_print.block<3,1>(VX,i) = q_i2b.inv().rot(truth.block<3,1>(4,i));
    x_print.block<4,1>(QW,i) = truth.block<4,1>(10,i);
  }
  cout << x_print << endl;

  // Clean up
  delete[] acc_ptr, gyro_ptr, mocap_ptr, truth_ptr;

  return 0;
}
