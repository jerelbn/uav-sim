#pragma once

#include <fstream>
#include <random>
#include <chrono>
#include <eigen3/Eigen/Eigen>

#include "common_cpp/common.h"
#include "common_cpp/logger.h"


namespace environment
{

typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> vectorVec3;
typedef std::map<std::string, Eigen::Vector3d, std::less<std::string>, 
                 Eigen::aligned_allocator<std::pair<const std::string, Eigen::Vector3d> > > mapVec3;

struct Plane
{
  Eigen::Vector3d r, n;
};


class Environment
{

public:

  Environment();
  Environment(const std::string filename, const std::default_random_engine& rng);
  ~Environment();

  void load(const std::string filename, const std::default_random_engine& rng);
  void addLandmark(const Eigen::Vector3d& p);
  void updateWind(const double t);
  void insertVehicle(const std::string& name, const Eigen::Vector3d &position);
  void updateVehicle(const std::string& name, const Eigen::Vector3d &position);
  void logWind(const double t);
  const double& getGridCellFrac() const { return grid_cell_frac_; }
  const double& getDepthVariation() const { return lm_depth_variation_; }
  const std::vector<Plane>& getPlanes() const { return planes_; }
  const Eigen::Vector3d& vw() const { return vw_; }
  const vectorVec3& getLandmarks() const { return landmarks_; }
  const mapVec3& getVehiclePositions() const { return vehicle_positions_; }
  const Eigen::Vector3d& getVehiclePosition(const std::string& name) const // std::map::operator[] is non-const so this is not as clean as desired
  { 
    auto pos = vehicle_positions_.find(name);
    if (pos == vehicle_positions_.end())
    {
      std::stringstream ss;
      ss << "environment.h: Unable to find " << name << " in vehicle_positions_";
      throw std::runtime_error(ss.str());
    }
    return pos->second;
  }

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
  mapVec3 vehicle_positions_;

  bool enable_wind_;
  Eigen::Vector3d vw_, vw_walk_;
  std::normal_distribution<double> vw_north_walk_dist_;
  std::normal_distribution<double> vw_east_walk_dist_;
  std::normal_distribution<double> vw_down_walk_dist_;

  common::Logger landmark_log_;
  common::Logger wind_log_;

};


} // namespace environment
