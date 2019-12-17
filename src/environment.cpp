#include "environment.h"

namespace environment
{


Environment::Environment()
{
  t_prev_ = 0;
  landmarks_.reserve(1e6);
}


Environment::Environment(const std::string filename)
{
  t_prev_ = 0;
  landmarks_.reserve(1e6);
  load(filename);
}


Environment::~Environment()
{
  for (auto& lm : landmarks_)
    landmark_log_.write((char*)lm.data(), lm.rows() * sizeof(double));
  landmark_log_.close();
  wind_log_.close();
}


void Environment::load(const std::string filename)
{
  // Initialize random number generator
  bool use_random_seed;
  common::get_yaml_node("use_random_seed", filename, use_random_seed);
  if (use_random_seed)
    rng_ = std::default_random_engine(std::chrono::system_clock::now().time_since_epoch().count());

  // Initialize wind and its walk parameters
  double vw_north_init_var, vw_east_init_var, vw_down_init_var;
  double vw_north_walk_stdev, vw_east_walk_stdev, vw_down_walk_stdev;
  common::get_yaml_node("enable_wind", filename, enable_wind_);
  common::get_yaml_node("random_initial_wind", filename, random_init_wind_);
  common::get_yaml_node("wind_north_init_stdev", filename, vw_north_init_var);
  common::get_yaml_node("wind_east_init_stdev", filename, vw_east_init_var);
  common::get_yaml_node("wind_down_init_stdev", filename, vw_down_init_var);
  common::get_yaml_node("wind_north_walk_stdev", filename, vw_north_walk_stdev);
  common::get_yaml_node("wind_east_walk_stdev", filename, vw_east_walk_stdev);
  common::get_yaml_node("wind_down_walk_stdev", filename, vw_down_walk_stdev);
  common::get_yaml_eigen("wind_init_vector", filename, vw_);
  vw_north_walk_dist_ = std::normal_distribution<double>(0.0,vw_north_walk_stdev);
  vw_east_walk_dist_ = std::normal_distribution<double>(0.0,vw_east_walk_stdev);
  vw_down_walk_dist_ = std::normal_distribution<double>(0.0,vw_down_walk_stdev);
  if (random_init_wind_)
  {
    vw_ = Eigen::Vector3d::Random();
    vw_(0) *= vw_north_init_var;
    vw_(1) *= vw_east_init_var;
    vw_(2) *= vw_down_init_var;
  }
  if (!enable_wind_)
    vw_.setZero();

  // Load and define information for landmark generation
  double north_dim, east_dim, height_dim;
  common::get_yaml_node("grid_cell_fraction", filename, grid_cell_frac_);
  common::get_yaml_node("landmark_depth_variation", filename, lm_depth_variation_);
  common::get_yaml_node("north_dim", filename, north_dim);
  common::get_yaml_node("east_dim", filename, east_dim);
  common::get_yaml_node("height_dim", filename, height_dim);

  plane_ground_.r.setZero();
  plane_ground_.n << 0, 0, -1;
  planes_.push_back(plane_ground_);

  plane_sky_.r << 0, 0, -height_dim;
  plane_sky_.n << 0, 0, 1;
  planes_.push_back(plane_sky_);

  plane_north_.r << north_dim/2.0, 0, 0;
  plane_north_.n << -1, 0, 0;
  planes_.push_back(plane_north_);

  plane_south_.r << -north_dim/2.0, 0, 0;
  plane_south_.n << 1, 0, 0;
  planes_.push_back(plane_south_);

  plane_east_.r << 0, east_dim/2.0, 0;
  plane_east_.n << 0, -1, 0;
  planes_.push_back(plane_east_);

  plane_west_.r << 0, -east_dim/2.0, 0;
  plane_west_.n << 0, 1, 0;
  planes_.push_back(plane_west_);

  // Initialize loggers
  landmark_log_.open("/tmp/landmarks.log");
  wind_log_.open("/tmp/wind.log");

  // Log environment initial wind data
  logWind(0);
}


void Environment::addLandmark(const Eigen::Vector3d& p)
{
  landmarks_.push_back(p);
}


void Environment::logWind(const double t)
{
  // Write data to binary files and plot in another program
  wind_log_.write((char*)&t, sizeof(double));
  wind_log_.write((char*)vw_.data(), vw_.rows() * sizeof(double));
}


void Environment::updateWind(const double t)
{
  if (enable_wind_)
  {
    double north_walk = vw_north_walk_dist_(rng_);
    double east_walk = vw_east_walk_dist_(rng_);
    double down_walk = vw_down_walk_dist_(rng_);
    vw_walk_ << north_walk, east_walk, down_walk;
    vw_ += vw_walk_ * (t - t_prev_);
  }
  t_prev_ = t;
  logWind(t);
}


void Environment::initVehicle(const Eigen::Vector3d &p, const int& id)
{
  vehicle_positions_.push_back(p);
  if (vehicle_positions_.size() != id + 1)
    std::runtime_error("Vehicle ID does not match Environment conainer index!");
}


void Environment::updateVehicle(const Eigen::Vector3d &p, const int& id)
{
  vehicle_positions_[id] = p;
}


}
