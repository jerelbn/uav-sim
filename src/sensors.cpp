#include "sensors.h"


namespace sensors
{


Sensors::Sensors() {}


Sensors::Sensors(const std::string filename)
{
  cam_.reserve(10000);
  load(filename);
}


Sensors::~Sensors()
{
  accel_log_.close();
  gyro_log_.close();
  cam_log_.close();
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

  new_imu_meas_ = false;

  // Camera
  double pixel_noise_stdev;
  Eigen::Vector4d q_bc;
  common::get_yaml_node("use_camera_truth", filename, use_camera_truth_);
  common::get_yaml_node("camera_update_rate", filename, camera_update_rate_);
  common::get_yaml_node("pixel_noise_stdev", filename, pixel_noise_stdev);
  common::get_yaml_eigen("image_size", filename, image_size_);
  common::get_yaml_eigen("camera_matrix", filename, K_);
  common::get_yaml_eigen("q_bc", filename, q_bc);
  common::get_yaml_eigen("p_bc", filename, p_bc_);
  last_camera_update_ = 0.0;
  pixel_noise_dist_ = std::normal_distribution<double>(0.0, pixel_noise_stdev);
  K_inv_ = K_.inverse();
  q_bc_ = common::Quaternion(q_bc);
  q_bc_.normalize();
  pixel_noise_.setZero();
  new_camera_meas_ = false;

  // Initialize loggers
  common::get_yaml_node("log_directory", filename, directory_);
  accel_log_.open(directory_ + "/accel.bin");
  gyro_log_.open(directory_ + "/gyro.bin");
  cam_log_.open(directory_ + "/camera.bin");
}


void Sensors::updateMeasurements(const double t, const vehicle::xVector &x, const Eigen::MatrixXd& lm)
{
  // Update all sensor measurements
  imu(t, x);
  camera(t, x, lm);
}


void Sensors::imu(const double t, const vehicle::xVector& x)
{
  double dt = t - last_imu_update_;
  if (t == 0 || dt >= 1.0 / imu_update_rate_)
  {
    new_imu_meas_ = true;
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


void Sensors::camera(const double t, const vehicle::xVector& x, const Eigen::MatrixXd &lm)
{
  double dt = t - last_camera_update_;
  if (t == 0 || dt >= 1.0 / camera_update_rate_)
  {
    new_camera_meas_ = true;
    last_camera_update_ = t;
    if (!use_camera_truth_)
      common::randomNormalMatrix(pixel_noise_,pixel_noise_dist_,rng_);

    // Compute camera pose
    common::Quaternion q_i2b = common::Quaternion(x.segment<4>(vehicle::QW));
    common::Quaternion q_i2c = q_i2b * q_bc_;
    Eigen::Vector3d p_i2c = x.segment<3>(vehicle::PX) + q_i2b.inv().rot(p_bc_);

    // Project landmarks into image
    cam_.clear();
    for (int i = 0; i < lm.cols(); ++i)
    {
      // Landmark vector in camera frame
      Eigen::Vector3d l = q_i2c.rot(lm.col(i) - p_i2c);

      // Check if landmark is in front of camera
      if (l(2) < 0)
        continue;

      // Project landmark into image and save it to the list
      Eigen::Vector2d pix;
      common::projToImg(pix, l, K_);
      pix += pixel_noise_;
      if (pix(0) >= 1 && pix(1) >= 1 && pix(0) <= image_size_(0) && pix(1) <= image_size_(1))
        cam_.push_back(Eigen::Vector3d(pix(0), pix(1), i));
    }

    // Log up to 5000 pixel measurements per measurement cycle, can't allocate much more on the stack...
    if (cam_.size() > 0)
    {
      cam_log_.write((char*)&t, sizeof(double));
      Eigen::Matrix<double,3*5000,1> cam_save = Eigen::Matrix<double,3*5000,1>::Constant(-1);
      int num_saves = std::min(5000, int(cam_.size()));
      for (int i = 0; i < num_saves; ++i)
        cam_save.segment<3>(3*i) = cam_[i];
      cam_log_.write((char*)cam_save.data(), cam_save.rows() * sizeof(double));
    }
  }
  else
  {
    new_camera_meas_ = false;
  }
}


void Sensors::depth(const double t, const vehicle::xVector& x) {}
void Sensors::gps(const double t, const vehicle::xVector& x) {}
void Sensors::baro(const double t, const vehicle::xVector& x) {}
void Sensors::alt(const double t, const vehicle::xVector& x) {}
void Sensors::mag(const double t, const vehicle::xVector& x) {}


}
