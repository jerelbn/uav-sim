#pragma once

#include <fstream>

#include "common_cpp/common.h"
#include "controller.h"
#include "sensors.h"
#include "environment.h"

using namespace Eigen;


namespace quadrotor
{


class Quadrotor
{

public:

  Quadrotor();
  Quadrotor(const std::string &filename, const int& id);
  ~Quadrotor();

  void load(const std::string &filename);
  void run(const double &t, const environment::Environment& env);

  const vehicle::State& getTrueState() const { return x_; }

  int id_;

private:

  void f(const vehicle::State& x, const uVector& u,
         const Eigen::Vector3d& vw, vehicle::dxVector& dx);
  void rk4(std::function<void(const vehicle::State&, const uVector&,
                        const Eigen::Vector3d&, vehicle::dxVector&)> func,
           const double& dt, const vehicle::State& x, const uVector& u,
           const Eigen::Vector3d& vw, vehicle::dxVector& dx);
  void propagate(const double &dt, const uVector& u, const Eigen::Vector3d& vw);
  void updateAccels(const uVector& u, const Eigen::Vector3d& vw);
  void getOtherVehicles(const std::vector<Eigen::Vector3d,
                        Eigen::aligned_allocator<Eigen::Vector3d> >& all_vehicle_positions);
  void log(const double &t);

  controller::Controller controller_;
  sensors::Sensors sensors_;

  vehicle::State x_, x2_, x3_, x4_;
  vehicle::dxVector dx_, k1_, k2_, k3_, k4_;
  uVector u_;

  bool accurate_integration_, control_using_estimates_;
  double mass_, max_thrust_, t_prev_;
  Eigen::Matrix3d inertia_matrix_, inertia_inv_;
  Eigen::Matrix3d linear_drag_matrix_;
  Eigen::Vector3d linear_drag_;
  Eigen::Matrix3d angular_drag_matrix_;
  Eigen::Vector3d v_rel_;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > other_vehicle_positions_;

  std::ofstream true_state_log_;
  std::ofstream command_log_;

};


}
