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
}


void Sensors::load(const std::string filename)
{
  // Initialize random number generator
  common::get_yaml_node("use_random_seed", filename, use_random_seed_);
  if (use_random_seed_)
    rng_ = std::default_random_engine(std::chrono::system_clock::now().time_since_epoch().count());

  // IMU
  double accel_bias_init_bound, accel_noise_stdev, accel_walk_stdev;
  common::get_yaml_node("imu_update_rate", filename, imu_update_rate_);
  common::get_yaml_node("use_accel_truth", filename, use_accel_truth_);
  common::get_yaml_node("accel_noise_stdev", filename, accel_noise_stdev);
  common::get_yaml_node("accel_walk_stdev", filename, accel_walk_stdev);
  common::get_yaml_node("accel_bias_init_bound", filename, accel_bias_init_bound);
  last_imu_update_ = 0.0;
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

  // Camera
  double pixel_noise_stdev;
  Eigen::Vector2d focal_lengths, image_center;
  Eigen::Vector4d q_b2c;
  common::get_yaml_node("camera_update_rate", filename, cam_update_rate_);
  common::get_yaml_node("pixel_noise_stdev", filename, pixel_noise_stdev);
  common::get_yaml_eigen("focal_len", filename, focal_lengths);
  common::get_yaml_eigen("image_center", filename, image_center);
  common::get_yaml_eigen("image_size", filename, image_size_);
  common::get_yaml_eigen("q_b2c", filename, q_b2c);
  common::get_yaml_eigen("p_b2c", filename, p_b2c_);
  last_camera_update_ = 0.0;
  pix_dist_ = std::normal_distribution<double>(0.0, pixel_noise_stdev);
  K_ << focal_lengths(0), 0, image_center(0), 0, focal_lengths(1), image_center(1), 0, 0, 1;
  K_inv_ = K_.inverse();
  q_b2c_ = common::Quaternion(q_b2c);
  q_b2c_.normalize();


  // Initialize loggers
  common::get_yaml_node("log_directory", filename, directory_);
  accel_log_.open(directory_ + "/accel.bin");
  gyro_log_.open(directory_ + "/gyro.bin");
}


void Sensors::updateMeasurements(const double t, const vehicle::xVector &x)
{
  // Update all sensor measurements
  imu(t, x);

  // Log all sensor measurements
  log(t);
}


void Sensors::log(const double t)
{
  accel_log_.write((char*)&t, sizeof(double));
  accel_log_.write((char*)accel_.data(), accel_.rows() * sizeof(double));
  accel_log_.write((char*)accel_bias_.data(), accel_bias_.rows() * sizeof(double));
  accel_log_.write((char*)accel_noise_.data(), accel_noise_.rows() * sizeof(double));
  gyro_log_.write((char*)&t, sizeof(double));
  gyro_log_.write((char*)gyro_.data(), gyro_.rows() * sizeof(double));
  gyro_log_.write((char*)gyro_bias_.data(), gyro_bias_.rows() * sizeof(double));
  gyro_log_.write((char*)gyro_noise_.data(), gyro_noise_.rows() * sizeof(double));
}


void Sensors::imu(const double t, const vehicle::xVector& x)
{
  double dt = t - last_imu_update_;
  if (t == 0 || dt >= 1.0 / imu_update_rate_)
  {
    last_imu_update_ = t;
    if (!use_accel_truth_)
    {
      common::randomNormalMatrix(accel_noise_,accel_noise_dist_,rng_);
      common::randomNormalMatrix(accel_walk_,accel_walk_dist_,rng_);
      accel_bias_ += accel_walk_ * dt;
    }
    if (!use_gyro_truth_)
    {
      common::randomNormalMatrix(gyro_noise_,gyro_noise_dist_,rng_);
      common::randomNormalMatrix(gyro_walk_,gyro_walk_dist_,rng_);
      gyro_bias_ += gyro_walk_ * dt;
    }
    body_gravity_ = common::gravity * common::Quaternion(x.segment<4>(vehicle::QW)).rot(common::e3);
    accel_ = x.segment<3>(vehicle::AX) - body_gravity_ + accel_bias_ + accel_noise_;
    gyro_ = x.segment<3>(vehicle::WX) + gyro_bias_ + gyro_noise_;
  }
}


void Sensors::camera(const vehicle::xVector& x) {}
void Sensors::depth(const vehicle::xVector& x) {}
void Sensors::gps(const vehicle::xVector& x) {}
void Sensors::baro(const vehicle::xVector& x) {}
void Sensors::alt(const vehicle::xVector& x) {}
void Sensors::mag(const vehicle::xVector& x) {}


}
