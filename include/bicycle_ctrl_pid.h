#pragma once

#include <fstream>
#include "common_cpp/common.h"
#include "common_cpp/logger.h"
#include "vehicle.h"

using namespace Eigen;


namespace bicycle_ctrl_pid
{


class Controller
{

public:

  Controller();
  Controller(const std::string &filename, const std::string& name);
  ~Controller();

  void load(const std::string &filename, const std::string& name);
  void computeControl(const bicycle::State& x);

  const bicycle::uVector& u() const { return u_; }

private:

  void updateWaypoint(const bicycle::State& x);
  void log(const double &t);

  bicycle::uVector u_;

  bool initialized_;
  double mass_, inertia_, max_force_, max_torque_, max_steering_angle_, L_, t_prev_;
  double ku_, ktheta_, kpsi_;
  double vel_cmd_;

  common::Logger command_log_;

  // Waypoint Parameters
  MatrixXd waypoints_;
  int current_waypoint_id_;
  double waypoint_threshold_;
  Vector2d wp_;

};


} // namespace bicycle_ctrl_pid
