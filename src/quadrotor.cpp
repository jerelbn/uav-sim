#include "quadrotor.h"

namespace quadrotor
{


Quadrotor::Quadrotor() {}


Quadrotor::Quadrotor(const std::string filename)
{
  load(filename);
}


void Quadrotor::load(std::string filename)
{
  // Instantiate Sensors, Controller, and Estimator classes
  controller_.load(filename);

  // Load all Quadrotor parameters
  common::get_yaml_node("accurate_integration", filename, accurate_integration_);
  common::get_yaml_node("mass", filename, mass_);
  common::get_yaml_node("max_thrust", filename, max_thrust_);

  Eigen::Vector3d inertia_diag, linear_drag_diag, angular_drag_diag;
  common::get_yaml_eigen<xVector>("x0", filename, x_);
  if (common::get_yaml_eigen<Eigen::Vector3d>("inertia", filename, inertia_diag))
  {
    inertia_matrix_ = inertia_diag.asDiagonal();
    inertia_inv_ = inertia_matrix_.inverse();
  }
  if (common::get_yaml_eigen<Eigen::Vector3d>("linear_drag", filename, linear_drag_diag))
    linear_drag_matrix_ = linear_drag_diag.asDiagonal();
  if (common::get_yaml_eigen<Eigen::Vector3d>("angular_drag", filename, angular_drag_diag))
    angular_drag_matrix_ = angular_drag_diag.asDiagonal();

  // Compute initial control and corresponding acceleration
  Eigen::Vector3d vw;
  controller_.computeControl(get_state(), 0, u_);
  vw.setZero(); // get vw from environment
  updateAccel(u_,vw);

  // Log initial data
}


void Quadrotor::f(const xVector& x, const commandVector& u, dxVector& dx, const Eigen::Vector3d& vw)
{
  common::Quaternion q_i2b(x.segment<4>(QW));
  v_rel_ = x.segment<3>(VX) - q_i2b.rot(vw);
  dx.segment<3>(PX) = q_i2b.inv().rot(x.segment<3>(VX));
  dx.segment<3>(QW) = x.segment<3>(WX);
  dx.segment<3>(DVX) = -common::e1 * u(THRUST)*max_thrust_ / mass_ - linear_drag_matrix_ * v_rel_.cwiseProduct(v_rel_) +
                      common::gravity * q_i2b.rot(common::e3) + x.segment<3>(VX).cross(x.segment<3>(WX));
  dx.segment<3>(DWX) = inertia_inv_ * (u.segment<3>(TAUX) - x.segment<3>(WX).cross(inertia_matrix_ * x.segment<3>(WX)) -
                       angular_drag_matrix_ * x.segment<3>(WX));
}


void Quadrotor::propagate(const double dt, const commandVector& u, const Eigen::Vector3d& vw)
{
  if (accurate_integration_)
  {
    // 4th order Runge-Kutta integration
    f(x_, u, k1_, vw);

    x2_ = x_;
    x2_.segment<3>(PX) += k1_.segment<3>(PX) * dt / 2;
    x2_.segment<4>(QW) = (common::Quaternion(x2_.segment<4>(QW)) + (k1_.segment<3>(QW)) * dt / 2).toEigen();
    x2_.segment<6>(VX) += k1_.segment<6>(DVX) * dt / 2;
    f(x2_, u, k2_, vw);

    x3_ = x_;
    x3_.segment<3>(PX) += k2_.segment<3>(PX) * dt / 2;
    x3_.segment<4>(QW) = (common::Quaternion(x3_.segment<4>(QW)) + (k2_.segment<3>(QW)) * dt / 2).toEigen();
    x3_.segment<6>(VX) += k2_.segment<6>(DVX) * dt / 2;
    f(x3_, u, k3_, vw);

    x4_ = x_;
    x4_.segment<3>(PX) += k3_.segment<3>(PX) * dt;
    x4_.segment<4>(QW) = (common::Quaternion(x4_.segment<4>(QW)) + (k3_.segment<3>(QW)) * dt).toEigen();
    x4_.segment<6>(VX) += k3_.segment<6>(DVX) * dt;
    f(x4_, u, k4_, vw);

    dx_ = (k1_ + 2 * k2_ + 2 * k3_ + k4_) * dt / 6.0;
  }
  else
  {
    // Euler integration
    f(x_, u, dx_, vw);
    dx_ = dx_ * dt;
  }

  // Copy output
  x_.segment<3>(PX) += dx_.segment<3>(PX);
  x_.segment<4>(QW) = (common::Quaternion(x_.segment<4>(QW)) + dx_.segment<3>(QW)).toEigen();
  x_.segment<6>(VX) += dx_.segment<6>(DVX);
}


void Quadrotor::run(const double t, const double dt, const Eigen::Vector3d& vw)
{
  propagate(dt, u_, vw); // propagate to next time step
  controller_.computeControl(get_state(), t, u_); // update control input
  updateAccel(u_, vw); // update acceleration
}


void Quadrotor::updateAccel(const commandVector &u, const Eigen::Vector3d &vw)
{
  dxVector dx;
  f(x_, u, dx, vw);
  x_.segment<3>(AX) = dx.segment<3>(DVX);
}


}
