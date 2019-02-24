#include "environment.h"

namespace environment
{


Environment::Environment()
{
  t_prev_ = 0;
}


Environment::Environment(const std::string filename)
{
  t_prev_ = 0;
  load(filename);
}


Environment::~Environment()
{
  environment_log_.close();
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
  double vw_init_var, vw_walk_stdev;
  common::get_yaml_node("wind_init_stdev", filename, vw_init_var);
  common::get_yaml_node("wind_walk_stdev", filename, vw_walk_stdev);
  common::get_yaml_node("enable_wind", filename, enable_wind_);
  vw_ = vw_init_var * Eigen::Vector3d::Random();
  vw_walk_dist_ = std::normal_distribution<double>(0.0,vw_walk_stdev);
  if (!enable_wind_)
    vw_.setZero();

  // Build the room or ground
  common::get_yaml_node("fly_indoors", filename, fly_indoors_);
  common::get_yaml_node("landmark_density", filename, lm_density_);
  common::get_yaml_node("landmark_deviation", filename, lm_deviation_);
  if (fly_indoors_)
  {
    common::get_yaml_node("indoor_north_dim", filename, indoor_north_dim_);
    common::get_yaml_node("indoor_east_dim", filename, indoor_east_dim_);
    common::get_yaml_node("indoor_height", filename, indoor_height_);
    buildRoom();
  }
  else
  {
    common::get_yaml_node("outdoor_north_dim", filename, outdoor_north_dim_);
    common::get_yaml_node("outdoor_east_dim", filename, outdoor_east_dim_);
    common::get_yaml_node("outdoor_height", filename, outdoor_height_);
    common::get_yaml_node("outdoor_hill_freq", filename, hill_freq_);
    buildGround();
  }

  // Initialize loggers
  environment_log_.open("/tmp/environment.bin");
  wind_log_.open("/tmp/wind.bin");

  // Log environment initial wind data
  environment_log_.write((char*)points_.data(), points_.rows() * points_.cols() * sizeof(double));
  logWind(0);
}


void Environment::buildRoom()
{
  // Origin is at the center of the room on the floor
  int num_pts_floor = indoor_north_dim_ * indoor_east_dim_ * lm_density_;
  int num_pts_ceil = indoor_north_dim_ * indoor_east_dim_ * lm_density_;
  int num_pts_north = indoor_height_ * indoor_east_dim_ * lm_density_;
  int num_pts_south = indoor_height_ * indoor_east_dim_ * lm_density_;
  int num_pts_east = indoor_height_ * indoor_north_dim_ * lm_density_;
  int num_pts_west = indoor_height_ * indoor_north_dim_ * lm_density_;
  int num_pts_total = num_pts_floor + num_pts_ceil + num_pts_north + num_pts_south + num_pts_east + num_pts_west;

  // Allocate space for all points
  points_.resize(3,num_pts_total);

  // Floor
  Eigen::ArrayXXd pts_floor(3,num_pts_floor);
  pts_floor.setRandom();
  pts_floor.row(0) *= indoor_north_dim_ / 2.0;
  pts_floor.row(1) *= indoor_east_dim_ / 2.0;
  pts_floor.row(2) *= lm_deviation_;

  // Ceiling
  Eigen::ArrayXXd pts_ceil(3,num_pts_ceil);
  pts_ceil.setRandom();
  pts_ceil.row(0) *= indoor_north_dim_ / 2.0;
  pts_ceil.row(1) *= indoor_east_dim_ / 2.0;
  pts_ceil.row(2) *= lm_deviation_;
  pts_ceil.row(2) -= indoor_height_; // Offset from origin

  // North wall
  Eigen::ArrayXXd pts_north(3,num_pts_north);
  pts_north.setRandom();
  pts_north.row(0) *= lm_deviation_;
  pts_north.row(1) *= indoor_east_dim_ / 2.0;
  pts_north.row(2) -= 1; // Shift random values to between 0 and -2
  pts_north.row(2) *= indoor_height_ / 2.0;
  pts_north.row(0) += indoor_north_dim_ / 2.0; // Offset from origin

  // South wall
  Eigen::ArrayXXd pts_south(3,num_pts_south);
  pts_south.setRandom();
  pts_south.row(0) *= lm_deviation_;
  pts_south.row(1) *= indoor_east_dim_ / 2.0;
  pts_south.row(2) -= 1; // Shift random values to between 0 and -2
  pts_south.row(2) *= indoor_height_ / 2.0;
  pts_south.row(0) -= indoor_north_dim_ / 2.0; // Offset from origin

  // East wall
  Eigen::ArrayXXd pts_east(3,num_pts_east);
  pts_east.setRandom();
  pts_east.row(0) *= indoor_north_dim_ / 2.0;
  pts_east.row(1) *= lm_deviation_;
  pts_east.row(2) -= 1; // Shift random values to between 0 and -2
  pts_east.row(2) *= indoor_height_ / 2.0;
  pts_east.row(1) += indoor_east_dim_ / 2.0; // Offset from origin

  // West wall
  Eigen::ArrayXXd pts_west(3,num_pts_west);
  pts_west.setRandom();
  pts_west.row(0) *= indoor_north_dim_ / 2.0;
  pts_west.row(1) *= lm_deviation_;
  pts_west.row(2) -= 1; // Shift random values to between 0 and -2
  pts_west.row(2) *= indoor_height_ / 2.0;
  pts_west.row(1) -= indoor_east_dim_ / 2.0; // Offset from origin

  // Concatenate all points
  points_ << pts_floor.matrix(), pts_ceil.matrix(), pts_north.matrix(),
             pts_south.matrix(), pts_east.matrix(), pts_west.matrix();
}


void Environment::buildGround()
{
  // Origin is at the center of the room on the floor
  int num_pts = outdoor_north_dim_ * outdoor_east_dim_ * lm_density_;

  // Allocate space for all points
  points_.resize(3,num_pts);

  // Create flat ground points
  Eigen::ArrayXXd pts_ground(3,num_pts);
  pts_ground.setRandom();
  pts_ground.row(0) *= outdoor_north_dim_ / 2.0;
  pts_ground.row(1) *= outdoor_east_dim_ / 2.0;
  pts_ground.row(2) *= lm_deviation_;

  // Add hills
  for (int i = 0; i < pts_ground.cols(); ++i)
    pts_ground.col(i).z() = outdoor_height_ * sin(hill_freq_ * pts_ground.col(i).x()) +
                            outdoor_height_ * sin(hill_freq_ * pts_ground.col(i).y());

  // Concatenate all points
  points_ << pts_ground.matrix();
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
    common::randomNormal(vw_walk_, vw_walk_dist_, rng_);
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
