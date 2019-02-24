#pragma once

#include <fstream>
#include <random>
#include <chrono>
#include <eigen3/Eigen/Eigen>

#include "common_cpp/common.h"


namespace environment
{


class Environment
{

public:

  Environment();
  Environment(const std::string filename);
  ~Environment();

  void load(const std::string filename);
  void updateWind(const double t);
  void initVehicle(const Eigen::Vector3d& p, const int &id);
  void updateVehicle(const Eigen::Vector3d& p, const int &id);
  void logWind(const double t);
  const Eigen::MatrixXd& get_points() const { return points_; }
  const Eigen::Vector3d& get_vw() const { return vw_; }
  const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >& getVehiclePositions() const { return vehicle_positions_; }
  const double getElevation(const double& x, const double& y) const;

private:

  void buildRoom();
  void buildGround();

  std::default_random_engine rng_;
  Eigen::MatrixXd points_;
  bool fly_indoors_;
  double t_prev_;
  double lm_density_;
  double lm_deviation_;
  double indoor_north_dim_, indoor_east_dim_, indoor_height_;
  double outdoor_north_dim_, outdoor_east_dim_, outdoor_height_, hill_freq_;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > vehicle_positions_;

  bool enable_wind_;
  Eigen::Vector3d vw_, vw_walk_;
  std::normal_distribution<double> vw_walk_dist_;

  std::ofstream environment_log_;
  std::ofstream wind_log_;

};


}
