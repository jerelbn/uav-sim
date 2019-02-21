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
  void updateMeasurements(const double t, const vehicle::State &x, const Eigen::MatrixXd &lm);

  Eigen::Vector3d gyro_, accel_;
  Eigen::Matrix<double, 7, 1> mocap_;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > cam_; // 3rd row contains labels

  bool new_imu_meas_, new_camera_meas_, new_mocap_meas_;

private:

  void imu(const double t, const vehicle::State &x);
  void camera(const double t, const vehicle::State& x, const Eigen::MatrixXd& lm);
  void mocap(const double t, const vehicle::State& x);
  void depth(const double t, const vehicle::State& x);
  void gps(const double t, const vehicle::State& x);
  void baro(const double t, const vehicle::State& x);
  void alt(const double t, const vehicle::State& x);
  void mag(const double t, const vehicle::State& x);

  bool use_random_seed_;
  std::default_random_engine rng_;

  // IMU
  bool use_accel_truth_, use_gyro_truth_;
  double last_imu_update_;
  double imu_update_rate_;
  Eigen::Vector3d accel_bias_, accel_noise_, accel_walk_;
  Eigen::Vector3d gyro_bias_, gyro_noise_, gyro_walk_;
  std::normal_distribution<double> accel_noise_dist_, accel_walk_dist_;
  std::normal_distribution<double> gyro_noise_dist_, gyro_walk_dist_;
  std::ofstream accel_log_, gyro_log_;

  // Camera
  bool use_camera_truth_, save_pixel_measurements_;
  double last_camera_update_;
  double camera_update_rate_;
  std::normal_distribution<double> pixel_noise_dist_;
  Eigen::Vector2d pixel_noise_;
  Eigen::Matrix3d K_, K_inv_;
  Eigen::Vector2d image_size_;
  quat::Quatd q_bc_, q_bu_; // rotations body-to-camera and body-to-IMU
  Eigen::Vector3d p_bc_, p_bu_; // translations body-to-camera and body-to-IMU in body frame
  std::ofstream cam_log_;

  // Motion Capture
  bool use_mocap_truth_;
  double last_mocap_update_;
  double mocap_update_rate_;
  std::normal_distribution<double> mocap_noise_dist_;
  Eigen::Matrix<double, 6, 1> mocap_noise_;
  Eigen::Vector3d p_bm_; // translation body-to-mocap-body in body frame
  quat::Quatd q_bm_; // rotation body-to-mocap-body
  std::ofstream mocap_log_;

};


}
