#include "sensors.h"


namespace sensors
{


Sensors::Sensors() {}


Sensors::Sensors(const std::string filename)
{
  load(filename);
}


Sensors::~Sensors()
{
  accel_log_.close();
  gyro_log_.close();
  cam_log_.close();
  mocap_log_.close();
}


void Sensors::load(const std::string filename)
{
  // Initialize random number generator
  common::get_yaml_node("use_random_seed", filename, use_random_seed_);
  int seed;
  if (use_random_seed_)
    seed = std::chrono::system_clock::now().time_since_epoch().count();
  else
    seed = 0;
  rng_ = std::default_random_engine(seed);
  srand(seed);

  // IMU
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
  last_imu_update_ = 0.0;
  q_bu_ = quat::Quatd(q_bu.data());
  q_bu_.normalize();
  accel_noise_dist_ = std::normal_distribution<double>(0.0, accel_noise_stdev);
  accel_walk_dist_ = std::normal_distribution<double>(0.0, accel_walk_stdev);
  accel_bias_.setRandom();
  accel_bias_ *= accel_bias_init_bound;
  accel_noise_.setZero();
  if (use_accel_truth_)
    accel_bias_.setZero();

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

  // Camera
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
  last_camera_update_ = 0.0;
  pixel_noise_dist_ = std::normal_distribution<double>(0.0, pixel_noise_stdev);
  K_inv_ = K_.inverse();
  q_bc_ = quat::Quatd(q_bc.data());
  q_bc_.normalize();
  pixel_noise_.setZero();
  new_camera_meas_ = false;
  cam_.reserve(cam_max_feat_);

  // Motion Capture
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

  // Initialize loggers
  accel_log_.open("/tmp/accel.bin");
  gyro_log_.open("/tmp/gyro.bin");
  cam_log_.open("/tmp/camera.bin");
  mocap_log_.open("/tmp/mocap.bin");
}


void Sensors::updateMeasurements(const double t, const vehicle::State &x, const Eigen::MatrixXd& lm)
{
  // Update enabled sensor measurements
  if (imu_enabled_)
    imu(t, x);
  if (camera_enabled_)
    camera(t, x, lm);
  if (mocap_enabled_)
    mocap(t, x);
}


void Sensors::imu(const double t, const vehicle::State& x)
{
  double dt = common::decRound(t - last_imu_update_, 1e6);
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


void Sensors::camera(const double t, const vehicle::State &x, const Eigen::MatrixXd &lm)
{
  double dt = common::decRound(t - last_camera_update_, 1e6);
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

    if (save_pixel_measurements_)
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


void Sensors::mocap(const double t, const vehicle::State &x)
{
  double dt = common::decRound(t - last_mocap_update_, 1e6);
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


void Sensors::depth(const double t, const vehicle::State& x) {}
void Sensors::gps(const double t, const vehicle::State& x) {}
void Sensors::baro(const double t, const vehicle::State& x) {}
void Sensors::alt(const double t, const vehicle::State& x) {}
void Sensors::mag(const double t, const vehicle::State& x) {}


}
