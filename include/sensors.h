#pragma once

#include <fstream>
#include <random>
#include <chrono>
#include "common_cpp/common.h"
#include "vehicle_common.h"


namespace sensors
{


class Sensors
{

public:

  Sensors();
  Sensors(const std::string filename);
  ~Sensors();

  void load(const std::string filename);
  void updateMeasurements(const double t, const vehicle::xVector& x);

private:

  void imu(const double t, const vehicle::xVector& x);
  void camera(const vehicle::xVector& x);
  void depth(const vehicle::xVector& x);
  void gps(const vehicle::xVector& x);
  void baro(const vehicle::xVector& x);
  void alt(const vehicle::xVector& x);
  void mag(const vehicle::xVector& x);

  void log(const double t);

  std::string directory_;
  bool use_random_seed_;
  std::default_random_engine rng_;
  Eigen::Vector3d body_gravity_;

  // IMU
  bool use_accel_truth_, use_gyro_truth_;
  double last_imu_update_;
  double imu_update_rate_;
  Eigen::Vector3d accel_, accel_bias_, accel_noise_, accel_walk_;
  Eigen::Vector3d gyro_, gyro_bias_, gyro_noise_, gyro_walk_;
  std::normal_distribution<double> accel_noise_dist_, accel_walk_dist_;
  std::normal_distribution<double> gyro_noise_dist_, gyro_walk_dist_;
  std::ofstream accel_log_, gyro_log_;

  // Camera
  double last_camera_update_;
  double cam_update_rate_;
  std::normal_distribution<double> pix_dist_;
  Eigen::Vector2d pix_noise_;
  std::vector<Eigen::Vector2d> cam_;
  Eigen::Matrix3d K_, K_inv_;
  Eigen::Vector2d image_size_;
  common::Quaternion q_b2c_;
  Eigen::Vector3d p_b2c_;

};


}
