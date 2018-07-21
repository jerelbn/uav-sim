#include "environment.h"

namespace environment
{


Environment::Environment() {}


Environment::Environment(const std::string filename)
{
  load(filename);
}


Environment::~Environment()
{
  environment_log_.close();
  wind_log_.close();
}


void Environment::load(const std::string filename)
{
  points_.resize(0,0);
  vw_.setZero();

  // Build the room
  common::get_yaml_node("wall_pts_density", filename, density_);
  common::get_yaml_node("wall_max_deviation", filename, max_deviation_);
  common::get_yaml_node("wall_north_dim", filename, north_dim_);
  common::get_yaml_node("wall_east_dim", filename, east_dim_);
  common::get_yaml_node("wall_height", filename, height_);
  buildRoom();

  // Initialize loggers
  common::get_yaml_node("log_directory", filename, directory_);
  environment_log_.open(directory_ + "/environment.bin");
  wind_log_.open(directory_ + "/wind.bin");

  // Log environment initial wind data
  environment_log_.write((char*)points_.data(), points_.rows() * points_.cols() * sizeof(double));
  log(0);
}


void Environment::buildRoom()
{
  // Origin is at the center of the room on the floor
  int num_pts_floor = north_dim_ * east_dim_ * density_;
  int num_pts_ceil = north_dim_ * east_dim_ * density_;
  int num_pts_north = height_ * east_dim_ * density_;
  int num_pts_south = height_ * east_dim_ * density_;
  int num_pts_east = height_ * north_dim_ * density_;
  int num_pts_west = height_ * north_dim_ * density_;
  int num_pts_total = num_pts_floor + num_pts_ceil + num_pts_north + num_pts_south + num_pts_east + num_pts_west;

  // Allocate space for all points
  points_.conservativeResize(3,num_pts_total);

  // Floor
  Eigen::ArrayXXd pts_floor(3,num_pts_floor);
  pts_floor.setRandom();
  pts_floor.row(0) *= north_dim_ / 2.0;
  pts_floor.row(1) *= east_dim_ / 2.0;
  pts_floor.row(2) *= max_deviation_;

  // Ceiling
  Eigen::ArrayXXd pts_ceil(3,num_pts_ceil);
  pts_ceil.setRandom();
  pts_ceil.row(0) *= north_dim_ / 2.0;
  pts_ceil.row(1) *= east_dim_ / 2.0;
  pts_ceil.row(2) *= max_deviation_;
  pts_ceil.row(2) -= height_; // Offset from origin

  // North wall
  Eigen::ArrayXXd pts_north(3,num_pts_north);
  pts_north.setRandom();
  pts_north.row(0) *= max_deviation_;
  pts_north.row(1) *= east_dim_ / 2.0;
  pts_north.row(2) -= 1; // Shift random values to between 0 and -2
  pts_north.row(2) *= height_ / 2.0;
  pts_north.row(0) += north_dim_ / 2.0; // Offset from origin

  // South wall
  Eigen::ArrayXXd pts_south(3,num_pts_south);
  pts_south.setRandom();
  pts_south.row(0) *= max_deviation_;
  pts_south.row(1) *= east_dim_ / 2.0;
  pts_south.row(2) -= 1; // Shift random values to between 0 and -2
  pts_south.row(2) *= height_ / 2.0;
  pts_south.row(0) -= north_dim_ / 2.0; // Offset from origin

  // East wall
  Eigen::ArrayXXd pts_east(3,num_pts_east);
  pts_east.setRandom();
  pts_east.row(0) *= north_dim_ / 2.0;
  pts_east.row(1) *= max_deviation_;
  pts_east.row(2) -= 1; // Shift random values to between 0 and -2
  pts_east.row(2) *= height_ / 2.0;
  pts_east.row(1) += east_dim_ / 2.0; // Offset from origin

  // West wall
  Eigen::ArrayXXd pts_west(3,num_pts_west);
  pts_west.setRandom();
  pts_west.row(0) *= north_dim_ / 2.0;
  pts_west.row(1) *= max_deviation_;
  pts_west.row(2) -= 1; // Shift random values to between 0 and -2
  pts_west.row(2) *= height_ / 2.0;
  pts_west.row(1) -= east_dim_ / 2.0; // Offset from origin

  // Concatenate all points
  points_ << pts_floor, pts_ceil, pts_north, pts_south, pts_east, pts_west;
}


void Environment::log(const double t)
{
  // Write data to binary files and plot in another program
  wind_log_.write((char*)&t, sizeof(double));
  wind_log_.write((char*)vw_.data(), vw_.rows() * sizeof(double));
}


}
