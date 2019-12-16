#pragma once

#include <fstream>

#include "common_cpp/common.h"
#include "common_cpp/logger.h"
#include "pb_vi_ekf/ekf.h"
#include "quad_control.h"
#include "sensors.h"
#include "environment.h"
#include "gimbal.h"

using namespace Eigen;


namespace quadrotor
{


class Quadrotor
{

public:

  Quadrotor();
  Quadrotor(const std::string &filename, const environment::Environment& env, const bool &use_random_seed, const int& id);
  ~Quadrotor();

  void load(const std::string &filename, const environment::Environment &env, const bool &use_random_seed);
  void run(const double &t, const environment::Environment& env);

  const vehicle::Stated& getState() const { return x_; }

  int id_;
  std::string name_;

private:

  void f(const vehicle::Stated& x, const uVector& u,
         const Eigen::Vector3d& vw, vehicle::dxVector& dx);
  void propagate(const double &dt, const uVector& u, const Eigen::Vector3d& vw);
  void updateAccels(const uVector& u, const Eigen::Vector3d& vw);
  void getOtherVehicles(const std::vector<Eigen::Vector3d,
                        Eigen::aligned_allocator<Eigen::Vector3d> >& all_vehicle_positions);
  void log(const double &t);
  void runEstimator(const double &t, const sensors::Sensors &sensors, const Vector3d &vw, const vehicle::Stated &x_t, const MatrixXd &lm);
  vehicle::Stated getControlStateFromEstimator() const;

  Controller controller_;
  sensors::Sensors sensors_;
  pbviekf::EKF estimator_;
  gimbal::Gimbal gimbal_;

  vehicle::Stated x_;
  vehicle::dxVector dx_;
  uVector u_;

  bool accurate_integration_, control_using_estimates_;
  double mass_, max_thrust_, t_prev_;
  Eigen::Matrix3d inertia_matrix_, inertia_inv_;
  Eigen::Matrix3d linear_drag_matrix_;
  Eigen::Matrix3d angular_drag_matrix_;
  Eigen::Vector3d v_rel_;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > other_vehicle_positions_;

  common::Logger state_log_;
  common::Logger euler_log_;

};


} // namespace quadrotor
