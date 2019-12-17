#pragma once

#include <fstream>
#include <random>
#include <chrono>
#include <eigen3/Eigen/Eigen>

#include "common_cpp/common.h"


namespace environment
{

typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> vectorVec3;

struct Plane
{
  Eigen::Vector3d r, n;
};


class Environment
{

public:

  Environment();
  Environment(const std::string filename);
  ~Environment();

  void load(const std::string filename);
  void addLandmark(const Eigen::Vector3d& p);
  void updateWind(const double t);
  void initVehicle(const Eigen::Vector3d& p, const int &id);
  void updateVehicle(const Eigen::Vector3d& p, const int &id);
  void logWind(const double t);
  const double& getGridCellFrac() const { return grid_cell_frac_; }
  const double& getDepthVariation() const { return lm_depth_variation_; }
  const std::vector<Plane>& getPlanes() const { return planes_; }
  const Eigen::Vector3d& getWindVel() const { return vw_; }
  const vectorVec3& getLandmarks() const { return landmarks_; }
  const vectorVec3& getVehiclePositions() const { return vehicle_positions_; }

private:

  std::default_random_engine rng_;
  double t_prev_;
  double grid_cell_frac_;
  double lm_depth_variation_;
  Plane plane_ground_;
  Plane plane_sky_;
  Plane plane_north_;
  Plane plane_south_;
  Plane plane_east_;
  Plane plane_west_;
  std::vector<Plane> planes_;
  vectorVec3 landmarks_; // use a K-D Tree or OcTree for better efficiency in the future
  vectorVec3 vehicle_positions_;

  bool enable_wind_, random_init_wind_;
  Eigen::Vector3d vw_, vw_walk_;
  std::normal_distribution<double> vw_north_walk_dist_;
  std::normal_distribution<double> vw_east_walk_dist_;
  std::normal_distribution<double> vw_down_walk_dist_;

  std::ofstream landmark_log_;
  std::ofstream wind_log_;

};


}
