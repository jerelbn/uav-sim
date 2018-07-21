#pragma once

#include <fstream>
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
  const Eigen::ArrayXXd& get_points() const { return points_; }
  const Eigen::Vector3d& get_vw() const { return vw_; }

private:

  void log(const double t);
  void buildRoom();

  Eigen::Vector3d vw_;
  Eigen::ArrayXXd points_;
  double density_;
  double max_deviation_;
  double north_dim_, east_dim_, height_;

  std::string directory_;
  std::ofstream environment_log_;
  std::ofstream wind_log_;

};


}
