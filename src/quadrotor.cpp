#include "quadrotor.h"

namespace quadrotor
{


Quadrotor::Quadrotor()  : t_prev_(0.0) {}


Quadrotor::Quadrotor(const std::string &filename, const environment::Environment& env, const bool& use_random_seed, const int& id)
  : t_prev_(0.0), id_(id)
{
  load(filename, env, use_random_seed);
}


Quadrotor::~Quadrotor() {}


void Quadrotor::load(const std::string &filename, const environment::Environment& env, const bool& use_random_seed)
{
  // Instantiate Sensors, Controller, and Estimator classes
  common::get_yaml_node("name", filename, name_);
  controller_.load(filename, use_random_seed, name_);
  sensors_.load(filename, use_random_seed, name_);
  estimator_.load("../params/pb_vi_ekf_params.yaml", name_);
  gimbal_.load("../params/gimbal.yaml");

  // Load all Quadrotor parameters
  common::get_yaml_node("accurate_integration", filename, accurate_integration_);
  common::get_yaml_node("mass", filename, mass_);
  common::get_yaml_node("max_thrust", filename, max_thrust_);
  common::get_yaml_node("control_using_estimates", filename, control_using_estimates_);

  vehicle::xVector x0;
  common::get_yaml_eigen("x0", filename, x0);
  common::get_yaml_eigen_diag("inertia", filename, inertia_matrix_);
  common::get_yaml_eigen_diag("linear_drag", filename, linear_drag_matrix_);
  common::get_yaml_eigen_diag("angular_drag", filename, angular_drag_matrix_);
  x_ = vehicle::Stated(x0);
  x_.drag = linear_drag_matrix_(0,0);
  inertia_inv_ = inertia_matrix_.inverse();

  // Randomly initialize estimator vel/roll/pitch/drag
  bool random_init;
  double v0_err, roll0_err, pitch0_err, drag0_err;
  common::get_yaml_node("ekf_random_init", filename, random_init);
  common::get_yaml_node("ekf_v0_err", filename, v0_err);
  common::get_yaml_node("ekf_roll0_err", filename, roll0_err);
  common::get_yaml_node("ekf_pitch0_err", filename, pitch0_err);
  common::get_yaml_node("ekf_drag0_err", filename, drag0_err);
  if (random_init)
  {
    double roll_new = x_.q.roll() + roll0_err * Vector1d::Random()(0);
    double pitch_new = x_.q.pitch() + pitch0_err * Vector1d::Random()(0);
    estimator_.setVelocity(x_.v + v0_err * Vector3d::Random());
    estimator_.setAttitude(quat::Quatd(roll_new,pitch_new,x_.q.yaw()).elements());
    estimator_.setDrag(x_.drag + drag0_err * Vector1d::Random()(0));
  }

  // Initialize other classes
  controller_.computeControl(getState(), 0, u_, other_vehicle_positions_[0]);
  updateAccels(u_, env.get_vw());
  gimbal_.update(0, x_, env);
  sensors_.updateMeasurements(0, x_, env.get_vw(), env.get_points());
  runEstimator(0, sensors_, env.get_vw(), getState(), env.get_points());

  // Initialize loggers and log initial data
  std::stringstream ss_s, ss_e;
  ss_s << "/tmp/" << name_ << "_true_state.log";
  ss_e << "/tmp/" << name_ << "_euler_angles.log";
  state_log_.open(ss_s.str());
  euler_log_.open(ss_e.str());
  log(0);
}


void Quadrotor::run(const double &t, const environment::Environment& env)
{
  getOtherVehicles(env.getVehiclePositions());
  propagate(t, u_, env.get_vw()); // Propagate truth to current time step
  if (control_using_estimates_)
    controller_.computeControl(getControlStateFromEstimator(), t, u_, other_vehicle_positions_[0]);
  else
    controller_.computeControl(getState(), t, u_, other_vehicle_positions_[0]);
  updateAccels(u_, env.get_vw()); // Update true acceleration
  gimbal_.update(t, x_, env);
  sensors_.updateMeasurements(t, x_, env.get_vw(), env.get_points());
  runEstimator(t, sensors_, env.get_vw(), getState(), env.get_points());
  log(t); // Log current data
}


void Quadrotor::f(const vehicle::Stated& x, const uVector& u,
                  const Vector3d& vw, vehicle::dxVector& dx)
{
  v_rel_ = x.v - x.q.rotp(vw);
  dx.segment<3>(vehicle::DP) = x.q.rota(x.v);
  dx.segment<3>(vehicle::DV) = -common::e3 * u(THRUST) * max_thrust_ / mass_ - linear_drag_matrix_ * v_rel_ +
                                 common::gravity * x.q.rotp(common::e3) - x.omega.cross(x.v);
  dx.segment<3>(vehicle::DQ) = x.omega;
  dx.segment<3>(vehicle::DW) = inertia_inv_ * (u.segment<3>(TAUX) - x.omega.cross(inertia_matrix_ * x.omega) -
                                angular_drag_matrix_ * x.omega);
}


void Quadrotor::propagate(const double &t, const uVector& u, const Vector3d& vw)
{
  // Time step
  double dt = t - t_prev_;
  t_prev_ = t;

  // Integration
  if (accurate_integration_)
  {
    // 4th order Runge-Kutta
    vehicle::rk4<COMMAND_SIZE>(std::bind(&Quadrotor::f, this,
                               std::placeholders::_1,std::placeholders::_2,
                               std::placeholders::_3,std::placeholders::_4),
                               dt, x_, u, vw, dx_);
  }
  else
  {
    // Euler
    f(x_, u, vw, dx_);
    dx_ *= dt;
  }
  x_ += dx_;
}


void Quadrotor::updateAccels(const uVector &u, const Vector3d &vw)
{
  static vehicle::dxVector dx;
  f(x_, u, vw, dx);
  x_.lin_accel = dx.segment<3>(vehicle::DV);
  x_.ang_accel = dx.segment<3>(vehicle::DW);
}


void Quadrotor::log(const double &t)
{
  // Write data to binary files and plot in another program
  state_log_.log(t);
  state_log_.logMatrix(x_.toEigen());
  euler_log_.log(t);
  euler_log_.logMatrix(x_.q.euler());
}


void Quadrotor::getOtherVehicles(const std::vector<Vector3d, aligned_allocator<Vector3d> > &all_vehicle_positions)
{
  other_vehicle_positions_.clear();
  for (int i = 0; i < all_vehicle_positions.size(); ++i)
    if (i != id_)
      other_vehicle_positions_.push_back(all_vehicle_positions[i]);
}


void Quadrotor::runEstimator(const double &t, const sensors::Sensors &sensors, const Vector3d& vw, const vehicle::Stated& xt, const MatrixXd& lm)
{
  // Run all sensor callbacks
  if (sensors.new_imu_meas_)
  {
    estimator_.imuCallback(sensors.imu_);
    if (estimator_.getFilterUpdateStatus())
      estimator_.logTruth(t, xt.p, xt.v, xt.q, sensors.getAccelBias(), sensors.getGyroBias(), xt.drag, xt.omega, lm);
  }
  if (sensors.new_camera_meas_)
    estimator_.cameraCallback(sensors.image_);
  if (sensors.new_gps_meas_)
    estimator_.gpsCallback(sensors.gps_);
  if (sensors.new_mocap_meas_)
    estimator_.mocapCallback(sensors.mocap_);
}


vehicle::Stated Quadrotor::getControlStateFromEstimator() const
{
  vehicle::Stated state;
  state.p = estimator_.getGlobalPosition();
  state.q = estimator_.getGlobalAttitude();
  state.v = estimator_.getState().v;
  return state;
}


} // namespace quadrotor
