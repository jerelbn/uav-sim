#include "quadrotor.h"

namespace quadrotor
{


Quadrotor::Quadrotor()  : t_prev_(0), tt_prev_(0) {}


Quadrotor::Quadrotor(const std::string &filename)  : t_prev_(0), tt_prev_(0)
{
  load(filename);
}


Quadrotor::~Quadrotor()
{
  true_state_log_.close();
  command_log_.close();
}


void Quadrotor::load(const std::string &filename)
{
  // Initialize random number generator
  common::get_yaml_node("use_random_seed", filename, use_random_seed_);
  int seed;
  if (use_random_seed_)
    seed = std::chrono::system_clock::now().time_since_epoch().count();
  else
    seed = 0;
  rng_ = std::default_random_engine(seed);
  srand(seed);

  // Instantiate Sensors, Controller, and Estimator classes
  controller_.load(filename);
  sensors_.load(filename);
  ekf_.load(filename);

  // Load all Quadrotor parameters
  common::get_yaml_node("accurate_integration", filename, accurate_integration_);
  common::get_yaml_node("mass", filename, mass_);
  common::get_yaml_node("max_thrust", filename, max_thrust_);
  common::get_yaml_node("control_using_estimates", filename, control_using_estimates_);

  vehicle::xVector x0;
  Eigen::Vector3d inertia_diag, angular_drag_diag;
  common::get_yaml_eigen<vehicle::xVector>("x0", filename, x0);
  x_ = vehicle::State(x0);
  if (common::get_yaml_eigen<Eigen::Vector3d>("inertia", filename, inertia_diag))
  {
    inertia_matrix_ = inertia_diag.asDiagonal();
    inertia_inv_ = inertia_matrix_.inverse();
  }
  if (common::get_yaml_eigen<Eigen::Vector3d>("linear_drag", filename, linear_drag_))
    linear_drag_matrix_ = linear_drag_.asDiagonal();
  if (common::get_yaml_eigen<Eigen::Vector3d>("angular_drag", filename, angular_drag_diag))
    angular_drag_matrix_ = angular_drag_diag.asDiagonal();

  // Target estimation initializers
  double target_noise_stdev;
  common::get_yaml_node("bearing_only", filename, bearing_only_);
  common::get_yaml_node("use_target_truth", filename, use_target_truth_);
  common::get_yaml_node("target_noise_stdev", filename, target_noise_stdev);
  target_noise_dist_ = std::normal_distribution<double>(0.0, target_noise_stdev);
  target_noise_.setZero();

  Eigen::Matrix<double, 6, 1> P0_diag, Q_diag;
  Eigen::Vector3d R_diag;
  common::get_yaml_node("target_kz", filename, kz_);
  common::get_yaml_eigen("target_x0", filename, xt_);
  common::get_yaml_eigen("target_P0", filename, P0_diag);
  Pt_ = P0_diag.asDiagonal();
  common::get_yaml_eigen("target_Q", filename, Q_diag);
  Q_ = Q_diag.asDiagonal();
  common::get_yaml_eigen("target_R", filename, R_diag);
  R_ = R_diag.asDiagonal();
  z_ = xt_.segment<3>(0);
  vz_ = xt_.segment<3>(3);
  C_.setZero();
  C_.block<3,3>(0,0) = common::I_3x3;

  // Compute initial control and corresponding acceleration
  Eigen::Vector3d vw;
  controller_.computeControl(getTrueState(), 0, u_, Eigen::Vector3d(1,0,0), z_, vz_, bearing_only_);
//  controller_.computeControl(ekf_.getVehicleState(), 0, u_);
  vw.setZero(); // get vw from environment
  updateAccel(u_,vw);

  // Initialize loggers
  common::get_yaml_node("log_directory", filename, directory_);
  true_state_log_.open(directory_ + "/true_state.bin");
  command_log_.open(directory_ + "/command.bin");
  target_log_.open(directory_ + "/target.bin");
}


void Quadrotor::f(const vehicle::State& x, const commandVector& u, vehicle::dxVector& dx, const Eigen::Vector3d& vw)
{
  v_rel_ = x.v - x.q.rot(vw);
  dx.segment<3>(vehicle::DPX) = x.q.inv().rot(x.v);
  dx.segment<3>(vehicle::DQX) = x.omega;
  dx.segment<3>(vehicle::DVX) = -common::e3 * u(THRUST) * max_thrust_ / mass_ - linear_drag_.cwiseProduct(v_rel_).cwiseProduct(v_rel_) +
                                 common::gravity * x.q.rot(common::e3) - x.omega.cross(x.v);
  dx.segment<3>(vehicle::DWX) = inertia_inv_ * (u.segment<3>(TAUX) - x.omega.cross(inertia_matrix_ * x.omega) -
                                angular_drag_matrix_ * x.omega.cwiseProduct(x.omega));
}


void Quadrotor::propagate(const double &t, const commandVector& u, const Eigen::Vector3d& vw)
{
  // Time step
  double dt = t - t_prev_;
  t_prev_ = t;

  // Differential Equations
  if (accurate_integration_)
  {
    // 4th order Runge-Kutta integration
    f(x_, u, k1_, vw);

    x2_ = x_;
    x2_.p += k1_.segment<3>(vehicle::DPX) * dt / 2;
    x2_.q += k1_.segment<3>(vehicle::DQX) * dt / 2;
    x2_.v += k1_.segment<3>(vehicle::DVX) * dt / 2;
    x2_.omega += k1_.segment<3>(vehicle::DWX) * dt / 2;
    f(x2_, u, k2_, vw);

    x3_ = x_;
    x3_.p += k2_.segment<3>(vehicle::DPX) * dt / 2;
    x3_.q += k2_.segment<3>(vehicle::DQX) * dt / 2;
    x3_.v += k2_.segment<3>(vehicle::DVX) * dt / 2;
    x3_.omega += k2_.segment<3>(vehicle::DWX) * dt / 2;
    f(x3_, u, k3_, vw);

    x4_ = x_;
    x4_.p += k3_.segment<3>(vehicle::DPX) * dt;
    x4_.q += k3_.segment<3>(vehicle::DQX) * dt;
    x4_.v += k3_.segment<3>(vehicle::DVX) * dt;
    x4_.omega += k3_.segment<3>(vehicle::DWX) * dt;
    f(x4_, u, k4_, vw);

    dx_ = (k1_ + 2 * k2_ + 2 * k3_ + k4_) * dt / 6.0;
  }
  else
  {
    // Euler integration
    f(x_, u, dx_, vw);
    dx_ *= dt;
  }

  // Copy output
  x_.p += dx_.segment<3>(vehicle::DPX);
  x_.q += dx_.segment<3>(vehicle::DQX);
  x_.v += dx_.segment<3>(vehicle::DVX);
  x_.omega += dx_.segment<3>(vehicle::DWX);
}


void Quadrotor::run(const double &t, const environment::Environment& env)
{
  // Create noisy measurements of target vehicle and estimate it
  getOtherVehicles(env.getVehiclePositions());
  Eigen::Vector3d pt = other_vehicle_positions[0];
  if (!use_target_truth_)
    common::randomNormalMatrix(target_noise_, target_noise_dist_, rng_);
  Eigen::Vector3d z_true = x_.q.rot(pt - x_.p);
  Eigen::Vector3d z = z_true + target_noise_;
  Eigen::Vector3d ez = z / z.norm();
  estimateTarget(t, z);

  sensors_.updateMeasurements(t, x_, env.get_points().matrix()); // Update sensor measurements
  log(t); // Log current data
  ekf_.run(t, sensors_);
  propagate(t, u_, env.get_vw()); // Propagate truth to next time step
  if (control_using_estimates_)
    controller_.computeControl(ekf_.getVehicleState(), t, u_, ez, z_, vz_, bearing_only_); // Update control input with estimates
  else
    controller_.computeControl(getTrueState(), t, u_, ez, z_, vz_, bearing_only_); // Update control input with truth
  updateAccel(u_, env.get_vw()); // Update true acceleration
}


void Quadrotor::updateAccel(const commandVector &u, const Eigen::Vector3d &vw)
{
  static vehicle::dxVector dx;
  f(x_, u, dx, vw);
  x_.accel = dx.segment<3>(vehicle::DVX);
}


void Quadrotor::log(const double &t)
{
  // Write data to binary files and plot in another program
  Eigen::Matrix<double, vehicle::NUM_STATES, 1> x = x_.toEigen();
  true_state_log_.write((char*)&t, sizeof(double));
  true_state_log_.write((char*)x.data(), x.rows() * sizeof(double));
  controller::Controller::state_t commanded_state = controller_.getCommandedState();
  command_log_.write((char*)&commanded_state, sizeof(controller::Controller::state_t));
  target_log_.write((char*)&t, sizeof(double));
  target_log_.write((char*)z_.data(), z_.rows() * sizeof(double));
  target_log_.write((char*)vz_.data(), vz_.rows() * sizeof(double));
}


void Quadrotor::getOtherVehicles(const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &all_vehicle_positions)
{
  // Reserve memory for the other vehicle positions
  if (other_vehicle_positions.size() < all_vehicle_positions.size()-1)
    other_vehicle_positions.reserve(all_vehicle_positions.size()-1);

  // Store other vehicle positions
  int count = 0;
  for (int i = 0; i < all_vehicle_positions.size(); ++i)
  {
    Eigen::Vector3d error = all_vehicle_positions[i] - x_.p;
    if (error.norm() > 1e-6)
    {
      other_vehicle_positions[count] = all_vehicle_positions[i];
      ++count;
    }
  }
}


void Quadrotor::estimateTarget(const double& t, const Eigen::Vector3d& z)
{
  // Get time step
  double dt = t - tt_prev_;
  tt_prev_ = t;

  // Form bearing measurement
  Eigen::Vector3d ez = z / z.norm();

  // Target estimator
  Eigen::Vector3d zdot(0, 0, 0);
  if (bearing_only_)
  {
    zdot = -x_.v - x_.omega.cross(z_) - kz_ * (common::I_3x3 - ez * ez.transpose()) * z_;
    z_ += zdot * dt;
    vz_.setZero();
  }
  else
  {
    // Update
    if (t > 0)
    {
      static Eigen::Matrix<double, 6, 6> I_6x6 = Eigen::MatrixXd::Identity(6, 6);
      Eigen::Matrix<double, 6, 3> K = Pt_ * C_.transpose() * (R_ + C_ * Pt_ * C_.transpose()).inverse();
      xt_ += K * (z - C_ * xt_);
      Pt_ = (I_6x6 - K * C_) * Pt_ * (I_6x6 - K * C_).transpose() + K * R_ * K.transpose();
    }

    // Propagate
    A_.setZero();
    A_.block<3,3>(0,0) = -common::skew(x_.omega);
    A_.block<3,3>(0,3) = common::I_3x3;
    A_.block<3,3>(3,3) = -common::skew(x_.omega);
    ut_.setZero();
    ut_.segment<3>(3) = -(x_.accel + x_.omega.cross(x_.v));
    Eigen::Matrix<double, 6, 1> xtdot = A_ * xt_ + ut_;
    Eigen::Matrix<double, 6, 6> Ptdot = A_ * Pt_ + Pt_ * A_.transpose() + Q_;
    xt_ += xtdot * dt;
    Pt_ += Ptdot * dt;

    // Pack output
    z_ = xt_.segment<3>(0);
    vz_ = xt_.segment<3>(3);
  }
}


}
