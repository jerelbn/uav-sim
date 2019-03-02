#pragma once

#include <fstream>
#include <ceres/ceres.h>

#include "common_cpp/common.h"
#include "fixed_wing_base.h"
#include "fixed_wing_control.h"
#include "fixed_wing_trim.h"
#include "sensors.h"
#include "environment.h"

using namespace Eigen;


namespace fixedwing
{


class FixedWing : public FixedWingBase
{

public:

  FixedWing();
  FixedWing(const std::string &filename, const environment::Environment& env, const bool &use_random_seed, const int& id);
  ~FixedWing();

  void load(const std::string &filename, const environment::Environment &env, const bool &use_random_seed);
  void run(const double &t, const environment::Environment& env);
  void computeTrim(const std::string& filename) const;

  const vehicle::Stated& getState() const { return x_; }

  int id_;
  std::string name_;

private:

  void propagate(const double &dt, const uVector& u, const Eigen::Vector3d& vw);
  void updateAccels(const uVector& u, const Eigen::Vector3d& vw);
  void getOtherVehicles(const std::vector<Eigen::Vector3d,
                        Eigen::aligned_allocator<Eigen::Vector3d> >& all_vehicle_positions);
  void log(const double &t);
  void computeLinearizedThrottle(const TrimState& x, const uVector& cmd, double &C_F_t, double &C_tau_t) const;

  Controller controller_;
  sensors::Sensors sensors_;

  vehicle::Stated x_;
  vehicle::dxVector dx_;
  uVector u_;

  bool accurate_integration_, control_using_estimates_;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > other_vehicle_positions_;
  double t_prev_;

  std::ofstream state_log_;

};


} // namespace fixedwing
