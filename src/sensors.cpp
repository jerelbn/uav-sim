#include "sensors.h"


namespace sensors
{


Sensors::Sensors() {}


Sensors::~Sensors()
{
  accel_log_.close();
  gyro_log_.close();
  cam_log_.close();
  mocap_log_.close();
  baro_log_.close();
  pitot_log_.close();
  wvane_log_.close();
  gps_log_.close();
}


void Sensors::load(const std::string& filename, const bool& use_random_seed, const std::string& name)
{
  // Initialize random number generator
  int seed;
  if (use_random_seed)
    seed = std::chrono::system_clock::now().time_since_epoch().count();
  else
    seed = 0;
  rng_ = std::default_random_engine(seed);
  srand(seed);

  // General parameters
  double origin_temp;
  t_round_off_ = 1e6;
  common::get_yaml_node("origin_latitude", filename, origin_lat_);
  common::get_yaml_node("origin_longitude", filename, origin_lon_);
  common::get_yaml_node("origin_altitude", filename, origin_alt_);
  common::get_yaml_node("origin_temperature", filename, origin_temp);
  rho_ = common::airDense(origin_alt_, origin_temp);

  // IMU
  std::stringstream ss_accel;
  double accel_bias_init_bound, accel_noise_stdev, accel_walk_stdev;
  Eigen::Vector4d q_bu;
  common::get_yaml_node("imu_enabled", filename, imu_enabled_);
  common::get_yaml_node("imu_update_rate", filename, imu_update_rate_);
  common::get_yaml_eigen("q_bu", filename, q_bu);
  common::get_yaml_eigen("p_bu", filename, p_bu_);
  common::get_yaml_node("use_accel_truth", filename, use_accel_truth_);
  common::get_yaml_node("accel_noise_stdev", filename, accel_noise_stdev);
  common::get_yaml_node("accel_walk_stdev", filename, accel_walk_stdev);
  common::get_yaml_node("accel_bias_init_bound", filename, accel_bias_init_bound);
  q_bu_ = quat::Quatd(q_bu.data());
  q_bu_.normalize();
  accel_noise_dist_ = std::normal_distribution<double>(0.0, accel_noise_stdev);
  accel_walk_dist_ = std::normal_distribution<double>(0.0, accel_walk_stdev);
  accel_bias_.setRandom();
  accel_bias_ *= accel_bias_init_bound;
  accel_noise_.setZero();
  if (use_accel_truth_)
    accel_bias_.setZero();
  ss_accel << "/tmp/" << name << "_accel.log";
  accel_log_.open(ss_accel.str());

  std::stringstream ss_gyro;
  double gyro_bias_init_bound, gyro_noise_stdev, gyro_walk_stdev;
  common::get_yaml_node("use_gyro_truth", filename, use_gyro_truth_);
  common::get_yaml_node("gyro_noise_stdev", filename, gyro_noise_stdev);
  common::get_yaml_node("gyro_walk_stdev", filename, gyro_walk_stdev);
  common::get_yaml_node("gyro_bias_init_bound", filename, gyro_bias_init_bound);
  gyro_noise_dist_ = std::normal_distribution<double>(0.0, gyro_noise_stdev);
  gyro_walk_dist_ = std::normal_distribution<double>(0.0, gyro_walk_stdev);
  gyro_bias_.setRandom();
  gyro_bias_ *= gyro_bias_init_bound;
  gyro_noise_.setZero();
  if (use_gyro_truth_)
    gyro_bias_.setZero();
  new_imu_meas_ = false;
  last_imu_update_ = 0.0;
  ss_gyro << "/tmp/" << name << "_gyro.log";
  gyro_log_.open(ss_gyro.str());

  // Camera
  std::stringstream ss_cam;
  double pixel_noise_stdev;
  Eigen::Vector4d q_bc;
  common::get_yaml_node("camera_enabled", filename, camera_enabled_);
  common::get_yaml_node("camera_max_features", filename, cam_max_feat_);
  common::get_yaml_node("use_camera_truth", filename, use_camera_truth_);
  common::get_yaml_node("save_pixel_measurements", filename, save_pixel_measurements_);
  common::get_yaml_node("camera_update_rate", filename, camera_update_rate_);
  common::get_yaml_node("pixel_noise_stdev", filename, pixel_noise_stdev);
  common::get_yaml_eigen("image_size", filename, image_size_);
  common::get_yaml_eigen("camera_matrix", filename, K_);
  common::get_yaml_eigen("q_bc", filename, q_bc);
  common::get_yaml_eigen("p_bc", filename, p_bc_);
  pixel_noise_dist_ = std::normal_distribution<double>(0.0, pixel_noise_stdev);
  K_inv_ = K_.inverse();
  q_bc_ = quat::Quatd(q_bc.data());
  q_bc_.normalize();
  pixel_noise_.setZero();
  new_camera_meas_ = false;
  last_camera_update_ = 0.0;
  cam_.reserve(cam_max_feat_);
  ss_cam << "/tmp/" << name << "_camera.log";
  cam_log_.open(ss_cam.str());

  // Motion Capture
  std::stringstream ss_mocap;
  double mocap_noise_stdev;
  Eigen::Vector4d q_bm;
  common::get_yaml_node("mocap_enabled", filename, mocap_enabled_);
  common::get_yaml_node("use_mocap_truth", filename, use_mocap_truth_);
  common::get_yaml_node("mocap_update_rate", filename, mocap_update_rate_);
  common::get_yaml_node("mocap_noise_stdev", filename, mocap_noise_stdev);
  common::get_yaml_eigen("q_bm", filename, q_bm);
  common::get_yaml_eigen("p_bm", filename, p_bm_);
  q_bm_ = quat::Quatd(q_bm.data());
  q_bm_.normalize();
  mocap_noise_dist_ = std::normal_distribution<double>(0.0, mocap_noise_stdev);
  mocap_noise_.setZero();
  new_mocap_meas_ = false;
  last_mocap_update_ = 0.0;
  ss_mocap << "/tmp/" << name << "_mocap.log";
  mocap_log_.open(ss_mocap.str());

  // Barometer
  std::stringstream ss_baro;
  double baro_noise_stdev, baro_walk_stdev, baro_bias_init_bound;
  common::get_yaml_node("baro_enabled", filename, baro_enabled_);
  common::get_yaml_node("use_baro_truth", filename, use_baro_truth_);
  common::get_yaml_node("baro_update_rate", filename, baro_update_rate_);
  common::get_yaml_node("baro_noise_stdev", filename, baro_noise_stdev);
  common::get_yaml_node("baro_walk_stdev", filename, baro_walk_stdev);
  common::get_yaml_node("baro_bias_init_bound", filename, baro_bias_init_bound);
  baro_noise_dist_ = std::normal_distribution<double>(0.0, baro_noise_stdev);
  baro_noise_ = 0;
  baro_walk_dist_ = std::normal_distribution<double>(0.0, baro_walk_stdev);
  baro_bias_ = 2.0 * baro_bias_init_bound * (std::rand() / double(RAND_MAX) - 0.5);
  if (use_baro_truth_)
    baro_bias_ = 0;
  new_baro_meas_ = false;
  last_baro_update_ = 0.0;
  ss_baro << "/tmp/" << name << "_baro.log";
  baro_log_.open(ss_baro.str());

  // Pitot Tube
  std::stringstream ss_pitot;
  double pitot_noise_stdev, pitot_walk_stdev, pitot_bias_init_bound;
  double pitot_azimuth, pitot_elevation;
  common::get_yaml_node("pitot_enabled", filename, pitot_enabled_);
  common::get_yaml_node("use_pitot_truth", filename, use_pitot_truth_);
  common::get_yaml_node("pitot_update_rate", filename, pitot_update_rate_);
  common::get_yaml_node("pitot_noise_stdev", filename, pitot_noise_stdev);
  common::get_yaml_node("pitot_walk_stdev", filename, pitot_walk_stdev);
  common::get_yaml_node("pitot_bias_init_bound", filename, pitot_bias_init_bound);
  common::get_yaml_node("pitot_azimuth", filename, pitot_azimuth);
  common::get_yaml_node("pitot_elevation", filename, pitot_elevation);
  q_b2pt_ = quat::Quatd(0, pitot_elevation, pitot_azimuth);
  pitot_noise_dist_ = std::normal_distribution<double>(0.0, pitot_noise_stdev);
  pitot_noise_ = 0;
  pitot_walk_dist_ = std::normal_distribution<double>(0.0, pitot_walk_stdev);
  pitot_bias_ = 2.0 * pitot_bias_init_bound * (std::rand() / double(RAND_MAX) - 0.5);
  if (use_pitot_truth_)
    pitot_bias_ = 0;
  new_pitot_meas_ = false;
  last_pitot_update_ = 0.0;
  ss_pitot << "/tmp/" << name << "_pitot.log";
  pitot_log_.open(ss_pitot.str());
}


void Sensors::updateMeasurements(const double t, const vehicle::Stated &x, const Vector3d &vw, const Eigen::MatrixXd& lm)
{
  // Update enabled sensor measurements
  if (imu_enabled_)
    imu(t, x);
  if (camera_enabled_)
    camera(t, x, lm);
  if (mocap_enabled_)
    mocap(t, x);
  if (baro_enabled_)
    baro(t, x);
  if (pitot_enabled_)
    pitot(t, x, vw);
  if (wvane_enabled_)
    wvane(t, x, vw);
  if (gps_enabled_)
    gps(t, x);
}


void Sensors::imu(const double t, const vehicle::Stated& x)
{
  double dt = common::decRound(t - last_imu_update_, t_round_off_);
  if (t == 0 || dt >= 1.0 / imu_update_rate_)
  {
    new_imu_meas_ = true;
    last_imu_update_ = t;
    if (!use_accel_truth_)
    {
      common::randomNormal(accel_noise_,accel_noise_dist_,rng_);
      common::randomNormal(accel_walk_,accel_walk_dist_,rng_);
      accel_bias_ += accel_walk_ * dt;
    }
    if (!use_gyro_truth_)
    {
      common::randomNormal(gyro_noise_,gyro_noise_dist_,rng_);
      common::randomNormal(gyro_walk_,gyro_walk_dist_,rng_);
      gyro_bias_ += gyro_walk_ * dt;
    }
    static quat::Quatd q_i2u;
    q_i2u = x.q * q_bu_;
    accel_ = q_bu_.rotp(x.lin_accel + x.omega.cross(x.v) + x.omega.cross(x.omega.cross(p_bu_)) +
             x.ang_accel.cross(p_bu_)) - common::gravity * q_i2u.rotp(common::e3) + accel_bias_ + accel_noise_;
    gyro_ = q_bu_.rotp(x.omega) + gyro_bias_ + gyro_noise_;

    // Log IMU data
    accel_log_.write((char*)&t, sizeof(double));
    accel_log_.write((char*)accel_.data(), accel_.rows() * sizeof(double));
    accel_log_.write((char*)accel_bias_.data(), accel_bias_.rows() * sizeof(double));
    accel_log_.write((char*)accel_noise_.data(), accel_noise_.rows() * sizeof(double));
    gyro_log_.write((char*)&t, sizeof(double));
    gyro_log_.write((char*)gyro_.data(), gyro_.rows() * sizeof(double));
    gyro_log_.write((char*)gyro_bias_.data(), gyro_bias_.rows() * sizeof(double));
    gyro_log_.write((char*)gyro_noise_.data(), gyro_noise_.rows() * sizeof(double));
  }
  else
  {
    new_imu_meas_ = false;
  }
}


void Sensors::camera(const double t, const vehicle::Stated &x, const Eigen::MatrixXd &lm)
{
  double dt = common::decRound(t - last_camera_update_, t_round_off_);
  if (t == 0 || dt >= 1.0 / camera_update_rate_)
  {
    new_camera_meas_ = true;
    last_camera_update_ = t;
    if (!use_camera_truth_)
      common::randomNormal(pixel_noise_,pixel_noise_dist_,rng_);

    // Compute camera pose
    quat::Quatd q_i2c = x.q * q_bc_;
    Eigen::Vector3d p_i2c = x.p + x.q.rota(p_bc_);

    // Project landmarks into image
    cam_.clear();
    for (int i = 0; i < lm.cols(); ++i)
    {
      // Landmark vector in camera frame
      Eigen::Vector3d l = q_i2c.rotp(lm.col(i) - p_i2c);

      // Check if landmark is in front of camera
      if (l(2) < 0)
        continue;

      // Project landmark into image and save it to the list
      Eigen::Vector2d pix;
      common::projToImg(pix, l, K_);
      pix += pixel_noise_;
      if (pix(0) >= 1 && pix(1) >= 1 && pix(0) <= image_size_(0) && pix(1) <= image_size_(1))
        cam_.push_back(Eigen::Vector3d(pix(0), pix(1), i));

      if (cam_.size() == cam_max_feat_) break;
    }

    if (cam_.size() > 0 && save_pixel_measurements_)
    {
      static const Vector3d a(-1,-1,-1);
      cam_log_.write((char*)&t, sizeof(double));
      for (int i = 0; i < cam_max_feat_; ++i)
      {
        if (i <= cam_.size())
          cam_log_.write((char*)cam_[i].data(), cam_[i].rows() * sizeof(double));
        else
          cam_log_.write((char*)a.data(), a.rows() * sizeof(double));
      }
    }
  }
  else
  {
    new_camera_meas_ = false;
  }
}


void Sensors::mocap(const double t, const vehicle::Stated &x)
{
  double dt = common::decRound(t - last_mocap_update_, t_round_off_);
  if (t == 0 || dt >= 1.0 / mocap_update_rate_)
  {
    new_mocap_meas_ = true;
    last_mocap_update_ = t;
    if (!use_mocap_truth_)
      common::randomNormal(mocap_noise_,mocap_noise_dist_,rng_);

    // Populate mocap measurement
    mocap_.head<3>() = x.p + x.q.rota(p_bm_) + mocap_noise_.head<3>();
    mocap_.tail<4>() = (x.q * q_bm_ + mocap_noise_.tail<3>()).elements();

    // Log Mocap data
    mocap_log_.write((char*)&t, sizeof(double));
    mocap_log_.write((char*)mocap_.data(), mocap_.rows() * sizeof(double));
    mocap_log_.write((char*)p_bm_.data(), 3 * sizeof(double));
    mocap_log_.write((char*)q_bm_.data(), 4 * sizeof(double));
    mocap_log_.write((char*)mocap_noise_.data(), mocap_noise_.rows() * sizeof(double));
  }
  else
  {
    new_mocap_meas_ = false;
  }
}


void Sensors::baro(const double t, const vehicle::Stated& x)
{
  double dt = common::decRound(t - last_baro_update_, t_round_off_);
  if (t == 0 || dt >= 1.0 / baro_update_rate_)
  {
    new_baro_meas_ = true;
    last_baro_update_ = t;
    if (!use_baro_truth_)
    {
      baro_noise_ = baro_noise_dist_(rng_);
      baro_walk_ = baro_walk_dist_(rng_);
      baro_bias_ += baro_walk_ * dt;
    }

    // Populate barometer measurement
    baro_ = rho_ * common::gravity * -x.p(2) + baro_bias_ + baro_noise_;

    // Log Mocap data
    baro_log_.write((char*)&t, sizeof(double));
    baro_log_.write((char*)&baro_, sizeof(double));
    baro_log_.write((char*)&baro_bias_, sizeof(double));
    baro_log_.write((char*)&baro_noise_, sizeof(double));
  }
  else
  {
    new_baro_meas_ = false;
  }
}


void Sensors::pitot(const double t, const vehicle::Stated& x, const Vector3d& vw)
{
  double dt = common::decRound(t - last_pitot_update_, t_round_off_);
  if (t == 0 || dt >= 1.0 / pitot_update_rate_)
  {
    new_pitot_meas_ = true;
    last_pitot_update_ = t;
    if (!use_pitot_truth_)
    {
      pitot_noise_ = pitot_noise_dist_(rng_);
      pitot_walk_ = pitot_walk_dist_(rng_);
      pitot_bias_ += pitot_walk_ * dt;
    }

    // Populate pitot tube measurement
    double Va = common::e1.dot(q_b2pt_.rotp(x.v - x.q.rotp(vw)));
    pitot_ = 0.5 * rho_ * Va * Va + pitot_bias_ + pitot_noise_;

    // Log Mocap data
    pitot_log_.write((char*)&t, sizeof(double));
    pitot_log_.write((char*)&pitot_, sizeof(double));
    pitot_log_.write((char*)&pitot_bias_, sizeof(double));
    pitot_log_.write((char*)&pitot_noise_, sizeof(double));
  }
  else
  {
    new_pitot_meas_ = false;
  }
}


void Sensors::wvane(const double t, const vehicle::Stated& x, const Vector3d& vw)
{
  //
}


void Sensors::gps(const double t, const vehicle::Stated& x)
{
  //
}


}
