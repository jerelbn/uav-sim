#pragma once

#include <fstream>
#include <random>
#include <chrono>
#include "common_cpp/common.h"
#include "vehicle.h"


namespace sensors
{


class Sensors
{

public:

  Sensors();
  ~Sensors();

  void load(const std::string &filename, const bool &use_random_seed, const std::string &name);
  void updateMeasurements(const double t, const vehicle::Stated &x, const Vector3d& vw, const Eigen::MatrixXd &lm);

  Eigen::Vector3d gyro_, accel_;
  Eigen::Matrix<double, 7, 1> mocap_;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > cam_; // 3rd row contains labels
  double baro_;
  double pitot_;
  double wvane_;
  Eigen::Matrix<double, 6, 1> gps_;

  bool new_imu_meas_;
  bool new_camera_meas_;
  bool new_mocap_meas_;
  bool new_baro_meas_;
  bool new_pitot_meas_;
  bool new_wvane_meas_;
  bool new_gps_meas_;

private:

  void imu(const double t, const vehicle::Stated &x);
  void camera(const double t, const vehicle::Stated& x, const Eigen::MatrixXd& lm);
  void mocap(const double t, const vehicle::Stated& x);
  void baro(const double t, const vehicle::Stated& x);
  void pitot(const double t, const vehicle::Stated& x, const Vector3d& vw);
  void wvane(const double t, const vehicle::Stated& x, const Vector3d& vw);
  void gps(const double t, const vehicle::Stated& x);

  std::default_random_engine rng_;
  double t_round_off_; // number of decimals to round off for time stamps
  double origin_lat_; // latitude at flight location (radians)
  double origin_lon_; // longitude at flight location (radians)
  double origin_alt_; // altitude above sea level at flight location (meters)
  double rho_; // air density at flight location

  // IMU
  bool use_accel_truth_, use_gyro_truth_, imu_enabled_;
  double last_imu_update_;
  double imu_update_rate_;
  Eigen::Vector3d accel_bias_, accel_noise_, accel_walk_;
  Eigen::Vector3d gyro_bias_, gyro_noise_, gyro_walk_;
  std::normal_distribution<double> accel_noise_dist_, accel_walk_dist_;
  std::normal_distribution<double> gyro_noise_dist_, gyro_walk_dist_;
  quat::Quatd q_bu_; // rotations body-to-IMU
  Eigen::Vector3d p_bu_; // translations body-to-IMU in body frame
  std::ofstream accel_log_, gyro_log_;

  // Camera
  bool use_camera_truth_, save_pixel_measurements_, camera_enabled_;
  int cam_max_feat_;
  double last_camera_update_;
  double camera_update_rate_;
  std::normal_distribution<double> pixel_noise_dist_;
  Eigen::Vector2d pixel_noise_;
  Eigen::Matrix3d K_, K_inv_;
  Eigen::Vector2d image_size_;
  quat::Quatd q_bc_; // rotations body-to-camera
  Eigen::Vector3d p_bc_; // translations body-to-camera in body frame
  std::ofstream cam_log_;

  // Motion Capture
  bool use_mocap_truth_, mocap_enabled_;
  double last_mocap_update_;
  double mocap_update_rate_;
  std::normal_distribution<double> mocap_noise_dist_;
  Eigen::Matrix<double, 6, 1> mocap_noise_;
  Eigen::Vector3d p_bm_; // translation body-to-mocap-body in body frame
  quat::Quatd q_bm_; // rotation body-to-mocap-body
  std::ofstream mocap_log_;

  // Barometer
  bool use_baro_truth_, baro_enabled_;
  double last_baro_update_;
  double baro_update_rate_;
  std::normal_distribution<double> baro_walk_dist_;
  std::normal_distribution<double> baro_noise_dist_;
  double baro_bias_, baro_walk_, baro_noise_;
  std::ofstream baro_log_;

  // Pitot Tube (for air speed along some axis)
  bool use_pitot_truth_, pitot_enabled_;
  double last_pitot_update_;
  double pitot_update_rate_;
  std::normal_distribution<double> pitot_walk_dist_;
  std::normal_distribution<double> pitot_noise_dist_;
  double pitot_bias_, pitot_walk_, pitot_noise_;
  quat::Quatd q_b2pt_; // rotation from body to pitot tube frame
  std::ofstream pitot_log_;

  // Weather vane (for sideslip angle)
  bool use_wvane_truth_, wvane_enabled_;
  double last_wvane_update_;
  double wvane_update_rate_;
  std::normal_distribution<double> wvane_noise_dist_;
  int wvane_resolution_; // rotary encoder discrete measurement steps
  double wvane_noise_;
  quat::Quatd q_b2wv_; // rotation from body to weather vane frame
  std::ofstream wvane_log_;

  // GPS
  bool use_gps_truth_, gps_enabled_;
  double last_gps_update_;
  double gps_update_rate_;
  std::normal_distribution<double> gps_hpos_walk_dist_;
  std::normal_distribution<double> gps_hpos_noise_dist_;
  std::normal_distribution<double> gps_hvel_walk_dist_;
  std::normal_distribution<double> gps_hvel_noise_dist_;
  std::normal_distribution<double> gps_vpos_walk_dist_;
  std::normal_distribution<double> gps_vpos_noise_dist_;
  std::normal_distribution<double> gps_vvel_walk_dist_;
  std::normal_distribution<double> gps_vvel_noise_dist_;
  Vector2d gps_hpos_walk_, gps_hvel_walk_;
  Vector2d gps_hpos_bias_, gps_hvel_bias_;
  Vector2d gps_hpos_noise_, gps_hvel_noise_;
  double gps_vpos_walk_, gps_vvel_walk_;
  double gps_vpos_bias_, gps_vvel_bias_;
  double gps_vpos_noise_, gps_vvel_noise_;
  std::ofstream gps_log_;

};


}
