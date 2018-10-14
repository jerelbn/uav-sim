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
typedef Matrix<double, DELTA_STATE_SIZE, DELTA_STATE_SIZE> CovMatrix;
typedef vector<State,aligned_allocator<State>> state_vector;
typedef vector<CovMatrix,aligned_allocator<CovMatrix>> cov_vector;

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


// Logger to save initial, final, and true states for plotting in MATLAB
void log_data(const string& filename, const Matrix<double, 1, Dynamic>& t,
              const state_vector& states, const double& cd,
              const Matrix<double,7,1>& T_bm)
{
  ofstream logger;
  logger.open(filename);
  for (int i = 0; i < states.size(); ++i)
  {
    logger.write((char*)&t(i), sizeof(double));
    logger.write((char*)states[i].data(), states[i].rows() * sizeof(double));
    logger.write((char*)&cd, sizeof(double));
    logger.write((char*)T_bm.data(), T_bm.rows() * sizeof(double));
  }
  logger.close();
  cout << "\nLogged " << filename << "\n\n";
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


// Derivative of state + state_delta w.r.t. state_delta
template<typename T>
Matrix<T,STATE_SIZE,DELTA_STATE_SIZE> dxpd_dd(const Matrix<T,STATE_SIZE,1>& x)
{
  Matrix<T,STATE_SIZE,DELTA_STATE_SIZE> dx;
  dx.setZero();
  dx.template block<3,3>(PX,DPX).setIdentity();
  dx.template block<3,3>(VX,DVX).setIdentity();
  dx.template block<4,3>(QW,DQX) = dqpd_dd<T>(x.template segment<4>(QW));
  dx.template block<3,3>(AX,DAX).setIdentity();
  dx.template block<3,3>(GX,DGX).setIdentity();
  return dx;
}


// State dynamics including input noise for Jacobian calculations
template<typename T>
void dynamics(const Matrix<T,3,1>& v, const common::Quaternion<T>& q,
                     const Matrix<T,3,1>& ba, const Matrix<T,3,1>& bg, const T& cd,
                     const Matrix<T,3,1>& acc, const Matrix<T,3,1>& omega,
                     const Matrix<T,3,1>& na, const Matrix<T,3,1>& ng,
                     Matrix<T,DELTA_STATE_SIZE,1>& dx)
{
  // Constants
  static Matrix<T,3,3> E3(common::e3.cast<T>() * common::e3.transpose().cast<T>());
  static Matrix<T,3,3> IE3(common::I_3x3.cast<T>() - common::e3.cast<T>() * common::e3.transpose().cast<T>());
  static Matrix<T,3,1> e3(common::e3.cast<T>());
  static T g = T(common::gravity);

  // Pack output
  dx.setZero();
  dx.template segment<3>(DPX) = q.inv().rot(v);
  dx.template segment<3>(DVX) = E3 * (acc - ba - na) + g * q.rot(e3) - cd * IE3 * v.cwiseProduct(v) - (omega - bg - ng).cross(v);
  dx.template segment<3>(DQX) = omega - bg - ng;
}


// Struct for calculation of Jacobian of dynamics w.r.t. the state
struct StateDot
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  StateDot(const double& _cd, const Vector3d& _acc, const Vector3d& _omega)
    : cd(_cd), acc(_acc), omega(_omega) {}

  template <typename T>
  bool operator()(const T* const x, T* residual) const
  {
    // Constants
    static const Matrix<T,3,1> z3(T(0), T(0), T(0));

    // Map states
    Map<const Matrix<T,3,1>> v(x+VX);
    const common::Quaternion<T> q(x+QW);
    Map<const Matrix<T,3,1>> ba(x+AX);
    Map<const Matrix<T,3,1>> bg(x+GX);

    // Compute output
    Matrix<T,DELTA_STATE_SIZE,1> dx(residual);
    dynamics<T>(v, q, ba, bg, T(cd), acc.cast<T>(), omega.cast<T>(), z3, z3, dx);
    Map<Matrix<T,DELTA_STATE_SIZE,1>> dx_(residual);
    dx_ = dx;
    return true;
  }

private:

  const double cd;
  const Vector3d acc, omega;

};


// Struct for calculation of Jacobian of dynamics w.r.t. the input noise
struct StateDot2
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  StateDot2(const State& _x, const double& _cd, const Vector3d& _acc, const Vector3d& _omega)
    : x(_x), cd(_cd), acc(_acc), omega(_omega) {}

  template <typename T>
  bool operator()(const T* const noise, T* residual) const
  {
    // Containers
    static Matrix<T,3,1> v, ba, bg;

    // Copy states for easy reading
    v = x.segment<3>(VX).cast<T>();
    common::Quaternion<T> q(x.segment<4>(QW).cast<T>());
    ba = x.segment<3>(AX).cast<T>();
    bg = x.segment<3>(GX).cast<T>();

    // Map noise
    Map<const Matrix<T,3,1>> na(noise);
    Map<const Matrix<T,3,1>> ng(noise+3);

    // Compute output
    Matrix<T,DELTA_STATE_SIZE,1> dx(residual);
    dynamics<T>(v, q, ba, bg, T(cd), acc.cast<T>(), omega.cast<T>(), na, ng, dx);
    Map<Matrix<T,DELTA_STATE_SIZE,1>> dx_(residual);
    dx_ = dx;
    return true;
  }

private:

  const double cd;
  const State x;
  const Vector3d acc, omega;

};


// Use automatic differentiation to calculate the Jacobian of state dynamics w.r.t. minimal state
template<typename T>
Matrix<T,DELTA_STATE_SIZE,DELTA_STATE_SIZE> getF(const Matrix<T,STATE_SIZE,1>& x,
                                                 const double& cd,
                                                 const Matrix<T,3,1>& acc,
                                                 const Matrix<T,3,1>& omega)
{
  // Calculate autodiff Jacobian
  T const* x_ptr = x.data();
  T const* const* x_ptr_ptr = &x_ptr;
  Matrix<T,DELTA_STATE_SIZE,1> r;
  Matrix<T,DELTA_STATE_SIZE,STATE_SIZE,RowMajor> J_autodiff;
  T* J_autodiff_ptr_ptr[1];
  J_autodiff_ptr_ptr[0] = J_autodiff.data();
  ceres::AutoDiffCostFunction<StateDot, DELTA_STATE_SIZE, STATE_SIZE> cost_function(new StateDot(cd, acc, omega));
  cost_function.Evaluate(x_ptr_ptr, r.data(), J_autodiff_ptr_ptr);

  // Convert autodiff Jacobian to minimal Jacobian
  return J_autodiff * dxpd_dd<T>(x);
}


// Use automatic differentiation to calculate the Jacobian of state dynamics w.r.t. input noise
template<typename T>
Matrix<T,DELTA_STATE_SIZE,6> getG(const Matrix<T,6,1>& n,
                                  const Matrix<T,STATE_SIZE,1>& x,
                                  const double& cd,
                                  const Matrix<T,3,1>& acc,
                                  const Matrix<T,3,1>& omega)
{
  // Calculate autodiff Jacobian
  T const* n_ptr = n.data();
  T const* const* n_ptr_ptr = &n_ptr;
  Matrix<T,DELTA_STATE_SIZE,1> r;
  Matrix<T,DELTA_STATE_SIZE,6,RowMajor> J_autodiff;
  T* J_autodiff_ptr_ptr[1];
  J_autodiff_ptr_ptr[0] = J_autodiff.data();
  ceres::AutoDiffCostFunction<StateDot2, DELTA_STATE_SIZE, 6> cost_function(new StateDot2(x, cd, acc, omega));
  cost_function.Evaluate(n_ptr_ptr, r.data(), J_autodiff_ptr_ptr);

  // Convert autodiff Jacobian to minimal Jacobian
  return J_autodiff;
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


struct PropagationFactor
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PropagationFactor(const vector<double>& _dts, const vector<Vector3d>& _accs,
                    const vector<Vector3d>& _gyros, CovMatrix& _P)
      : dts(_dts), accs(_accs), gyros(_gyros), P(_P) {}

  template <typename T>
  bool operator()(const T* const x1, const T* const x2, const T* const cd, T* residuals) const
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

    // Predict current state with IMU measurements
    Matrix<T,DELTA_STATE_SIZE,1> dx;
    for (int i = 0; i < dts.size(); ++i)
    {
      dynamics<T>(v2hat, q2hat, ba2hat, bg2hat, cd[0], accs[i].cast<T>(), gyros[i].cast<T>(), z3, z3, dx);
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

    // Weight residuals by information matrix
//    residuals_ = LLT<CovMatrix>(P.inverse()).matrixL().transpose() * residuals_;

    return true;
  }

private:

  const vector<double> dts;
  const vector<Vector3d> accs, gyros;
  const CovMatrix P;

};


struct MocapFactor
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MocapFactor(const Vector3d& p_meas, const Vector4d& q_meas)
      : T_meas_(p_meas, q_meas) {}

  template <typename T>
  bool operator()(const T* const x, const T* const _T_bm, T* residuals) const
  {
    Map<const Matrix<T,3,1>> p(x+PX);
    const common::Quaternion<T> q(x+QW);
    const common::Transform<T> T_bm(_T_bm);
    common::Transform<T> T_hat(p + q.inv().rot(T_bm.p()), q * T_bm.q());
    Map<Matrix<T,6,1>> residuals_(residuals);
    residuals_ = Matrix<T,6,1>(T_meas_.cast<T>() - T_hat);
    return true;
  }

private:

  const common::Transformd T_meas_;

};


struct DragFactor
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DragFactor(const Vector3d& acc_meas)
      : acc_meas_(acc_meas) {}

  template <typename T>
  bool operator()(const T* const x, const T* const cd, T* residuals) const
  {
    static Matrix<T,3,3> IE3(common::I_3x3.cast<T>() - common::e3.cast<T>() * common::e3.transpose().cast<T>());
    const common::Quaternion<T> q(x+QW);
    Map<const Matrix<T,3,1>> v(x+VX);
    Map<const Matrix<T,3,1>> ba(x+AX);
    Map<Matrix<T,2,1>> residuals_(residuals);
    residuals_ = common::I_2x3.cast<T>() * (acc_meas_.cast<T>() - (ba - cd[0] * IE3 * v.cwiseProduct(v)));
    return true;
  }

private:

  const Vector3d acc_meas_;

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
      xt[j].segment<3>(PX) = truth.block<3,1>(1,i);
      xt[j].segment<3>(VX) = truth.block<3,1>(4,i);
      xt[j].segment<4>(QW) = truth.block<4,1>(10,i);
      xt[j].segment<3>(AX) = acc.block<3,1>(4,i);
      xt[j].segment<3>(GX) = gyro.block<3,1>(4,i);
      ++j;
    }
    if (j > mocap.cols()-1) break;
  }
  common::Transformd T_bm_t(Vector3d(0.1, 0.1, 0.1),Vector4d(0.9928, 0.0447, 0.0547, 0.0971));
  log_data("../logs/mocap_opt_truth.bin", mocap.row(0), xt, 0.1, T_bm_t.toEigen());

  // Parameters
  const int N = mocap.cols();
  const int print_start = mocap.cols()-5;
  const int print_end = mocap.cols();
  Matrix<double,6,6> Qu;
  Qu << 2.5e-5, 0, 0, 0, 0, 0,
        0, 2.5e-5, 0, 0, 0, 0,
        0, 0, 2.5e-5, 0, 0, 0,
        0, 0, 0,   1e-6, 0, 0,
        0, 0, 0, 0,   1e-6, 0,
        0, 0, 0, 0, 0,   1e-6;

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
  log_data("../logs/mocap_opt_initial.bin", mocap.row(0), x, cd, T_bm.toEigen());

  // Initialize covariance of each state
  Matrix<double,DELTA_STATE_SIZE,1> cov0_vec;
  cov0_vec << 0.0001, 0.0001, 0.0001, // POS
              0.5, 0.5, 0.5, // VEL
              0.0001, 0.0001, 0.0001, // ATT
              0.2, 0.2, 0.2, // BIAS ACC
              0.1, 0.1, 0.1; // BIAS GYRO
  cov_vector P(N);
  P[0] = cov0_vec.asDiagonal();
  j = 0;
  Matrix<double,6,1> n; n.setOnes();
  CovMatrix I; I.setIdentity();
  for (int i = 0; i < N-1; ++i)
  {
    double dt = mocap(0,i+1) - mocap(0,i);
    Vector3d acc_, gyro_;
    for (int j = 0; j < acc.cols(); ++j)
    {
      // Accelerometer and gyro usually run on same time steps in an IMU
      if (acc(0,j) == mocap(0,i) && gyro(0,j) == mocap(0,i))
      {
        acc_ = acc.block<3,1>(1,j);
        gyro_ = gyro.block<3,1>(1,j);
        break;
      }
    }
    CovMatrix F = getF<double>(x[j], cd, acc.block<3,1>(1,i), gyro.block<3,1>(1,i));
    Matrix<double,DELTA_STATE_SIZE,6> G = getG<double>(n, x[j], cd, acc.block<3,1>(1,i), gyro.block<3,1>(1,i));
    CovMatrix A = I + F*dt + F*F*dt*dt + F*F*F*dt*dt*dt;
    Matrix<double,DELTA_STATE_SIZE,6> B = (dt*(I + F*dt/2.0 + F*F*dt*dt/3.0 + F*F*F*dt*dt*dt/4.0))*G;
    P[i+1] = A * P[i] * A.transpose() + B * Qu * B.transpose();
  }

//  // Print initial state
//  print_state("x0", x, print_start, print_end);

  // Build optimization problem with Ceres-Solver
  ceres::Problem problem;

  // Add parameter blocks
  ceres::LocalParameterization *state_local_parameterization =
      new ceres::AutoDiffLocalParameterization<StatePlus,STATE_SIZE,DELTA_STATE_SIZE>;
  ceres::LocalParameterization *transform_local_parameterization =
      new ceres::AutoDiffLocalParameterization<PosePlus,7,6>;
  for (int i = 0; i < N; ++i)
    problem.AddParameterBlock(x[i].data(), STATE_SIZE, state_local_parameterization);
  problem.AddParameterBlock(&cd, 1);
  problem.AddParameterBlock(T_bm.data(), 7, transform_local_parameterization);

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
      new ceres::AutoDiffCostFunction<PropagationFactor, DELTA_STATE_SIZE, STATE_SIZE, STATE_SIZE, 1>(
      new PropagationFactor(dts, accs, gyros, P[i]));
    problem.AddResidualBlock(cost_function, NULL, x[i].data(), x[i+1].data(), &cd);
  }

  // Add Mocap factors
  for (int i = 0; i < N; ++i)
  {
    ceres::CostFunction* cost_function =
      new ceres::AutoDiffCostFunction<MocapFactor, 6, STATE_SIZE, 7>(
      new MocapFactor(mocap.block<3,1>(1,i), mocap.block<4,1>(4,i)));
    problem.AddResidualBlock(cost_function, NULL, x[i].data(), T_bm.data());
  }

  // Add drag factors
  for (int i = 0; i < N; ++i)
  {
    for (int j = 0; j < acc.cols(); ++j)
    {
      if (mocap(0,i) == acc(0,j))
      {
        ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<DragFactor, 2, STATE_SIZE, 1>(
          new DragFactor(acc.block<3,1>(1,j)));
        problem.AddResidualBlock(cost_function, NULL, x[i].data(), &cd);
        break;
      }
    }
  }

  // Covariance - need the covariance to propagate all the way through during each step of optimization
  // - can save covariance of each state in factor struct
  // - probably need to put ceres::Solve in a loop where it takes one step, then updates
  //   covariance in each factor

  // Solve for the optimal rotation and translation direciton
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
  options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  options.max_num_iterations = 100;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  cout << summary.BriefReport() << "\n\n";

  log_data("../logs/mocap_opt_final.bin", mocap.row(0), x, cd, T_bm.toEigen());

//  // Print solution and truth
//  print_state("xf", x, print_start, print_end);
//  print_state("xt", xt, print_start, print_end);

//  cout << "\nF = \n" << getF<double>(x[999], acc.block<3,1>(1,999), gyro.block<3,1>(1,999)) << endl;
//  cout << "\nG = \n" << getG<double>(n, x[999], acc.block<3,1>(1,999), gyro.block<3,1>(1,999)) << endl;
//  cout << "\nP[N] = \n" << P[N-1] << endl;

  return 0;
}
