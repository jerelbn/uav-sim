#pragma once

#include <fstream>
#include "common_cpp/common.h"
#include "common_cpp/logger.h"
#include "vehicle.h"
#include "environment.h"

using namespace Eigen;


namespace bicycle
{


class Bicycle
{

public:

  Bicycle();
  Bicycle(const std::string &filename, const environment::Environment& env);
  ~Bicycle();

  void load(const std::string &filename, const environment::Environment& env);
  void propagate(const double &t, const uVector& u, const environment::Environment& env);
  
  const std::string& name() const { return name_; }
  const State& x() const { return x_; }


private:

  void f(const State& x, const uVector& u, xVector& dx);
  void updateElevation(const environment::Environment& env);
  void log(const double &t);

  std::string name_;
  State x_;
  xVector dx_;

  bool initialized_, flat_ground_;
  double mass_, inertia_, max_steering_angle_, L_, t_prev_;

  common::Logger state_log_;

};


} // namespace bicycle
