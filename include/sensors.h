#pragma once

#include <random>
#include "common_cpp/common.h"
#include "common_cpp/logger.h"
#include "common_cpp/measurement.h"
#include "geometry/xform.h"
#include "vehicle.h"
#include "environment.h"
#include "wgs84.h"

using namespace std;
using namespace Eigen;


namespace sensors
{


class Sensors
{

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Sensors();
  ~Sensors();

  void load(const string &filename, const bool &use_random_seed, const string &name);
  void updateMeasurements(const double t, const vehicle::Stated &x, environment::Environment& env);
  const Vector3d& getAccelBias() const { return accel_bias_; }
  const Vector3d& getGyroBias() const { return gyro_bias_; }
  const double& getBaroBias() const { return baro_bias_; }
  const Vector3d& getMagBias() const { return mag_bias_; }
  const double& getPitotBias() const { return pitot_bias_; }

  common::Imud imu_;
  common::Mocapd mocap_;
  common::Imaged image_;
  common::Gpsd gps_;
  common::Barod baro_;
  common::Magd mag_;
  common::Pitotd pitot_;
  common::Wvaned wvane_;

  bool new_imu_meas_;
  bool new_mocap_meas_;
  bool new_camera_meas_;
  bool new_gps_meas_;
  bool new_baro_meas_;
  bool new_mag_meas_;
  bool new_pitot_meas_;
  bool new_wvane_meas_;

private:

  void imu(const double t, const vehicle::Stated &x);
  void mocap(const double t, const vehicle::Stated& x);
  void camera(const double t, const vehicle::Stated& x, environment::Environment &env);
  void gps(const double t, const vehicle::Stated& x);
  void baro(const double t, const vehicle::Stated& x);
  void mag(const double t, const vehicle::Stated& x);
  void pitot(const double t, const vehicle::Stated& x, const Vector3d& vw);
  void wvane(const double t, const vehicle::Stated& x, const Vector3d& vw);

  default_random_engine rng_;
  double t_round_off_; // number of decimals to round off for time stamps
  double origin_lat_; // latitude at flight location (radians)
  double origin_lon_; // longitude at flight location (radians)
  double origin_alt_; // altitude above sea level at flight location (meters)
  double rho_; // air density at flight location
  Vector3d mnp_ecef_; // magnetic north pole direction in ECEF coordinates
  quat::Quatd q_ecef_to_mnp_; // rotation from ECEF to MNP (magnetic north pole) coordinates

  // IMU
  bool use_accel_truth_, use_gyro_truth_, imu_enabled_;
  int imu_id_;
  double last_imu_update_;
  double imu_update_rate_;
  Vector3d accel_bias_, accel_noise_, accel_walk_;
  Vector3d gyro_bias_, gyro_noise_, gyro_walk_;
  normal_distribution<double> accel_noise_dist_, accel_walk_dist_;
  normal_distribution<double> gyro_noise_dist_, gyro_walk_dist_;
  quat::Quatd q_ub_; // rotations IMU-to-body
  Vector3d p_ub_; // translations IMU-to-body in IMU frame
  common::Logger accel_log_, gyro_log_;

  // Camera
  bool use_camera_truth_, save_pixel_measurements_, camera_enabled_;
  int image_id_, cam_max_feat_;
  double last_camera_update_;
  double camera_update_rate_;
  double camera_time_delay_;
  vector<common::Imaged> cam_buffer_;
  normal_distribution<double> pixel_noise_dist_;
  normal_distribution<double> depth_noise_dist_;
  Vector2d pixel_noise_;
  Matrix3d K_, K_inv_;
  Vector2d image_size_;
  quat::Quatd q_uc_; // rotations IMU-to-camera
  Vector3d p_uc_; // translations IMU-to-camera in IMU frame
  common::Logger cam_log_;

  // Motion Capture
  bool use_mocap_truth_, mocap_enabled_;
  int mocap_id_;
  double last_mocap_update_;
  double mocap_update_rate_;
  double mocap_time_delay_;
  vector<common::Mocapd> mocap_buffer_;
  normal_distribution<double> mocap_noise_dist_;
  Matrix<double, 6, 1> mocap_noise_;
  xform::Xformd mocap_truth_;
  Vector3d p_um_; // translation IMU-to-mocap-body in IMU frame
  quat::Quatd q_um_; // rotation IMU-to-mocap-body
  common::Logger mocap_log_;

  // Barometer
  bool use_baro_truth_, baro_enabled_;
  int baro_id_;
  double last_baro_update_;
  double baro_update_rate_;
  normal_distribution<double> baro_walk_dist_;
  normal_distribution<double> baro_noise_dist_;
  double baro_bias_, baro_walk_, baro_noise_;
  common::Logger baro_log_;

  // Magnetometer
  bool use_mag_truth_, mag_enabled_;
  int mag_id_;
  double last_mag_update_;
  double mag_update_rate_;
  normal_distribution<double> mag_walk_dist_;
  normal_distribution<double> mag_noise_dist_;
  Vector3d mag_bias_, mag_walk_, mag_noise_;
  common::Logger mag_log_;

  // Pitot Tube (for air speed along some axis)
  bool use_pitot_truth_, pitot_enabled_;
  int pitot_id_;
  double last_pitot_update_;
  double pitot_update_rate_;
  normal_distribution<double> pitot_walk_dist_;
  normal_distribution<double> pitot_noise_dist_;
  double pitot_bias_, pitot_walk_, pitot_noise_;
  quat::Quatd q_b2pt_; // rotation from body to pitot tube frame
  common::Logger pitot_log_;

  // Weather vane (for sideslip angle)
  bool use_wvane_truth_, wvane_enabled_;
  int wvane_id_;
  double last_wvane_update_;
  double wvane_update_rate_;
  normal_distribution<double> wvane_noise_dist_;
  int wvane_resolution_; // rotary encoder discrete measurement steps
  double wvane_noise_;
  quat::Quatd q_b2wv_; // rotation from body to weather vane frame
  common::Logger wvane_log_;

  // GPS
  bool use_gps_truth_, gps_enabled_;
  int gps_id_;
  double last_gps_update_;
  double gps_update_rate_;
  double gps_time_constant_;
  normal_distribution<double> gps_hpos_noise_dist_;
  normal_distribution<double> gps_hvel_noise_dist_;
  normal_distribution<double> gps_vpos_noise_dist_;
  normal_distribution<double> gps_vvel_noise_dist_;
  Vector2d gps_hpos_bias_;
  double gps_vpos_bias_;
  Vector2d gps_hpos_noise_, gps_hvel_noise_;
  double gps_vpos_noise_, gps_vvel_noise_;
  xform::Xformd X_ecef2ned_;
  common::Logger gps_log_;

};


}
