#include "sensors.h"


namespace sensors
{


Sensors::Sensors()
  : imu_id_(0), mocap_id_(0), image_id_(0), gps_id_(0), baro_id_(0), pitot_id_(0), wvane_id_(0)
{}
Sensors::~Sensors() {}


void Sensors::load(const string& filename, const bool& use_random_seed, const string& name)
{
  // Initialize random number generator
  int seed;
  if (use_random_seed)
    seed = chrono::system_clock::now().time_since_epoch().count();
  else
    seed = 0;
  rng_ = default_random_engine(seed);
  srand(seed);

  // General parameters
  double origin_temp;
  t_round_off_ = 6;
  common::get_yaml_node("origin_latitude", filename, origin_lat_);
  common::get_yaml_node("origin_longitude", filename, origin_lon_);
  common::get_yaml_node("origin_altitude", filename, origin_alt_);
  common::get_yaml_node("origin_temperature", filename, origin_temp);
  rho_ = common::airDense(origin_alt_, origin_temp);

  // IMU
  stringstream ss_accel;
  double accel_bias_init_bound, accel_noise_stdev, accel_walk_stdev;
  Vector4d q_ub;
  common::get_yaml_node("imu_enabled", filename, imu_enabled_);
  common::get_yaml_node("imu_update_rate", filename, imu_update_rate_);
  common::get_yaml_eigen("q_ub", filename, q_ub);
  common::get_yaml_eigen("p_ub", filename, p_ub_);
  common::get_yaml_node("use_accel_truth", filename, use_accel_truth_);
  common::get_yaml_node("accel_noise_stdev", filename, accel_noise_stdev);
  common::get_yaml_node("accel_walk_stdev", filename, accel_walk_stdev);
  common::get_yaml_node("accel_bias_init_bound", filename, accel_bias_init_bound);
  q_ub_ = quat::Quatd(q_ub.normalized());
  accel_noise_dist_ = normal_distribution<double>(0.0, accel_noise_stdev);
  accel_walk_dist_ = normal_distribution<double>(0.0, accel_walk_stdev);
  accel_bias_ = accel_bias_init_bound * Vector3d::Random();
  accel_noise_.setZero();
  if (use_accel_truth_)
    accel_bias_.setZero();
  ss_accel << "/tmp/" << name << "_accel.log";
  accel_log_.open(ss_accel.str());

  stringstream ss_gyro;
  double gyro_bias_init_bound, gyro_noise_stdev, gyro_walk_stdev;
  common::get_yaml_node("use_gyro_truth", filename, use_gyro_truth_);
  common::get_yaml_node("gyro_noise_stdev", filename, gyro_noise_stdev);
  common::get_yaml_node("gyro_walk_stdev", filename, gyro_walk_stdev);
  common::get_yaml_node("gyro_bias_init_bound", filename, gyro_bias_init_bound);
  gyro_noise_dist_ = normal_distribution<double>(0.0, gyro_noise_stdev);
  gyro_walk_dist_ = normal_distribution<double>(0.0, gyro_walk_stdev);
  gyro_bias_ = gyro_bias_init_bound * Vector3d::Random();
  gyro_noise_.setZero();
  if (use_gyro_truth_)
    gyro_bias_.setZero();
  new_imu_meas_ = false;
  last_imu_update_ = 0.0;
  ss_gyro << "/tmp/" << name << "_gyro.log";
  gyro_log_.open(ss_gyro.str());

  // Camera
  stringstream ss_cam;
  double pixel_noise_stdev;
  Vector4d q_uc;
  common::get_yaml_node("camera_enabled", filename, camera_enabled_);
  common::get_yaml_node("camera_max_features", filename, cam_max_feat_);
  common::get_yaml_node("use_camera_truth", filename, use_camera_truth_);
  common::get_yaml_node("save_pixel_measurements", filename, save_pixel_measurements_);
  common::get_yaml_node("camera_update_rate", filename, camera_update_rate_);
  common::get_yaml_node("camera_time_delay", filename, camera_time_delay_);
  common::get_yaml_node("pixel_noise_stdev", filename, pixel_noise_stdev);
  common::get_yaml_eigen("image_size", filename, image_size_);
  common::get_yaml_eigen("camera_matrix", filename, K_);
  common::get_yaml_eigen("q_uc", filename, q_uc);
  common::get_yaml_eigen("p_uc", filename, p_uc_);
  pixel_noise_dist_ = normal_distribution<double>(0.0, pixel_noise_stdev);
  K_inv_ = K_.inverse();
  q_uc_ = quat::Quatd(q_uc.normalized());
  pixel_noise_.setZero();
  new_camera_meas_ = false;
  last_camera_update_ = 0.0;
  image_.feats.reserve(cam_max_feat_);
  ss_cam << "/tmp/" << name << "_camera.log";
  cam_log_.open(ss_cam.str());

  // Motion Capture
  stringstream ss_mocap;
  double mocap_noise_stdev;
  Vector4d q_um;
  common::get_yaml_node("mocap_enabled", filename, mocap_enabled_);
  common::get_yaml_node("use_mocap_truth", filename, use_mocap_truth_);
  common::get_yaml_node("mocap_update_rate", filename, mocap_update_rate_);
  common::get_yaml_node("mocap_time_delay", filename, mocap_time_delay_);
  if (mocap_time_delay_ < 0)
    throw runtime_error("Cannot have a negative motion capture time delay!");
  common::get_yaml_node("mocap_noise_stdev", filename, mocap_noise_stdev);
  common::get_yaml_eigen("q_um", filename, q_um);
  common::get_yaml_eigen("p_um", filename, p_um_);
  q_um_ = quat::Quatd(q_um.normalized());
  mocap_noise_dist_ = normal_distribution<double>(0.0, mocap_noise_stdev);
  mocap_noise_.setZero();
  new_mocap_meas_ = false;
  last_mocap_update_ = 0.0;
  ss_mocap << "/tmp/" << name << "_mocap.log";
  mocap_log_.open(ss_mocap.str());

  // Barometer
  stringstream ss_baro;
  double baro_noise_stdev, baro_walk_stdev, baro_bias_init_bound;
  common::get_yaml_node("baro_enabled", filename, baro_enabled_);
  common::get_yaml_node("use_baro_truth", filename, use_baro_truth_);
  common::get_yaml_node("baro_update_rate", filename, baro_update_rate_);
  common::get_yaml_node("baro_noise_stdev", filename, baro_noise_stdev);
  common::get_yaml_node("baro_walk_stdev", filename, baro_walk_stdev);
  common::get_yaml_node("baro_bias_init_bound", filename, baro_bias_init_bound);
  baro_noise_dist_ = normal_distribution<double>(0.0, baro_noise_stdev);
  baro_noise_ = 0;
  baro_walk_dist_ = normal_distribution<double>(0.0, baro_walk_stdev);
  baro_bias_ = 2.0 * baro_bias_init_bound * (rand() / double(RAND_MAX) - 0.5);
  if (use_baro_truth_)
    baro_bias_ = 0;
  new_baro_meas_ = false;
  last_baro_update_ = 0.0;
  ss_baro << "/tmp/" << name << "_baro.log";
  baro_log_.open(ss_baro.str());

  // Pitot Tube
  stringstream ss_pitot;
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
  pitot_noise_dist_ = normal_distribution<double>(0.0, pitot_noise_stdev);
  pitot_noise_ = 0;
  pitot_walk_dist_ = normal_distribution<double>(0.0, pitot_walk_stdev);
  pitot_bias_ = 2.0 * pitot_bias_init_bound * (rand() / double(RAND_MAX) - 0.5);
  if (use_pitot_truth_)
    pitot_bias_ = 0;
  new_pitot_meas_ = false;
  last_pitot_update_ = 0.0;
  ss_pitot << "/tmp/" << name << "_pitot.log";
  pitot_log_.open(ss_pitot.str());

  // Weather Vane
  stringstream ss_wvane;
  double wvane_noise_stdev;
  double wvane_roll;
  common::get_yaml_node("wvane_enabled", filename, wvane_enabled_);
  common::get_yaml_node("use_wvane_truth", filename, use_wvane_truth_);
  common::get_yaml_node("wvane_update_rate", filename, wvane_update_rate_);
  common::get_yaml_node("wvane_noise_stdev", filename, wvane_noise_stdev);
  common::get_yaml_node("wvane_resolution", filename, wvane_resolution_);
  common::get_yaml_node("wvane_roll", filename, wvane_roll);
  q_b2wv_ = quat::Quatd(wvane_roll, 0, 0);
  wvane_noise_dist_ = normal_distribution<double>(0.0, wvane_noise_stdev);
  wvane_noise_ = 0;
  new_wvane_meas_ = false;
  last_wvane_update_ = 0.0;
  ss_wvane << "/tmp/" << name << "_wvane.log";
  wvane_log_.open(ss_wvane.str());

  // GPS
  stringstream ss_gps;
  double gps_hpos_noise_stdev, gps_hvel_noise_stdev;
  double gps_vpos_noise_stdev, gps_vvel_noise_stdev;
  double gps_hpos_bias_init_bound, gps_vpos_bias_init_bound;
  common::get_yaml_node("gps_enabled", filename, gps_enabled_);
  common::get_yaml_node("use_gps_truth", filename, use_gps_truth_);
  common::get_yaml_node("gps_update_rate", filename, gps_update_rate_);
  common::get_yaml_node("gps_time_constant", filename, gps_time_constant_);
  common::get_yaml_node("gps_horizontal_position_noise_stdev", filename, gps_hpos_noise_stdev);
  common::get_yaml_node("gps_horizontal_position_bias_init_bound", filename, gps_hpos_bias_init_bound);
  common::get_yaml_node("gps_horizontal_velocity_noise_stdev", filename, gps_hvel_noise_stdev);
  common::get_yaml_node("gps_vertical_position_noise_stdev", filename, gps_vpos_noise_stdev);
  common::get_yaml_node("gps_vertical_position_bias_init_bound", filename, gps_vpos_bias_init_bound);
  common::get_yaml_node("gps_vertical_velocity_noise_stdev", filename, gps_vvel_noise_stdev);
  gps_hpos_noise_dist_ = normal_distribution<double>(0.0, gps_hpos_noise_stdev);
  gps_hvel_noise_dist_ = normal_distribution<double>(0.0, gps_hvel_noise_stdev);
  gps_vpos_noise_dist_ = normal_distribution<double>(0.0, gps_vpos_noise_stdev);
  gps_vvel_noise_dist_ = normal_distribution<double>(0.0, gps_vvel_noise_stdev);
  gps_hpos_noise_.setZero();
  gps_hvel_noise_.setZero();
  gps_vpos_noise_ = 0;
  gps_vvel_noise_ = 0;
  gps_hpos_bias_ = gps_hpos_bias_init_bound * Vector2d::Random();
  gps_vpos_bias_ = 2.0 * gps_vpos_bias_init_bound * (rand() / double(RAND_MAX) - 0.5);
  if (use_gps_truth_)
  {
    gps_hpos_bias_.setZero();
    gps_vpos_bias_ = 0;
  }
  new_gps_meas_ = false;
  last_gps_update_ = 0.0;
  ss_gps << "/tmp/" << name << "_gps.log";
  gps_log_.open(ss_gps.str());

  // Calculate ECEF to NED transform
  Vector3d origin_ecef = WGS84::lla2ecef(Vector3d(origin_lat_,origin_lon_,origin_alt_));
  X_ecef2ned_ = WGS84::x_ecef2ned(origin_ecef);

}


void Sensors::updateMeasurements(const double t, const vehicle::Stated &x, const Vector3d &vw, const MatrixXd& lm)
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
  double dt = common::round2dec(t - last_imu_update_, t_round_off_);
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
    quat::Quatd q_i2u = x.q * q_ub_.inverse();
    Vector3d p_bu = q_ub_.rotp(-p_ub_);
    imu_.accel = q_ub_.rota(x.lin_accel + x.omega.cross(x.v) + x.omega.cross(x.omega.cross(p_bu)) +
                 x.ang_accel.cross(p_bu)) - common::gravity * q_i2u.rotp(common::e3) + accel_bias_ + accel_noise_;
    imu_.gyro = q_ub_.rota(x.omega) + gyro_bias_ + gyro_noise_;
    imu_.t = t;
    imu_.id = imu_id_++;

    // Log data
    accel_log_.log(t);
    accel_log_.logMatrix(imu_.accel, accel_bias_, accel_noise_);
    gyro_log_.log(t);
    gyro_log_.logMatrix(imu_.gyro, gyro_bias_, gyro_noise_);
  }
  else
  {
    new_imu_meas_ = false;
  }
}


void Sensors::camera(const double t, const vehicle::Stated &x, const MatrixXd &lm)
{
  double dt = common::round2dec(t - last_camera_update_, t_round_off_);
  if (t == 0 || dt >= 1.0 / camera_update_rate_)
  {
    last_camera_update_ = t;
    if (!use_camera_truth_)
      common::randomNormal(pixel_noise_,pixel_noise_dist_,rng_);

    // Project landmarks into image
    image_.feats.clear();
    for (int i = 0; i < lm.cols(); ++i)
    {
      // Landmark vector in camera frame
      Vector3d p_cl = q_uc_.rotp(q_ub_.rota(x.q.rotp(lm.col(i) - x.p)) - p_uc_ + p_ub_);

      // Check if landmark is in front of camera
      if (p_cl(2) < 0)
        continue;

      // Project landmark into image and save it to the list
      Vector2d pix;
      common::projToImg(pix, p_cl, K_);
      pix += pixel_noise_;
      if (pix(0) >= 1 && pix(1) >= 1 && pix(0) <= image_size_(0) && pix(1) <= image_size_(1))
        image_.feats.push_back(common::Featd(i,pix,p_cl)); // feature labels correspond to its column in the inertial points matrix

      if (image_.feats.size() == cam_max_feat_) break;
    }
    image_.t = t;
    image_.id = image_id_++;

    // Save camera measurement
    if (image_.feats.size() > 0 && save_pixel_measurements_)
    {
      static const Vector2d a = Vector2d::Constant(NAN);
      cam_log_.log(t);
      for (int i = 0; i < cam_max_feat_; ++i)
      {
        if (i <= image_.feats.size())
          cam_log_.logMatrix(image_.feats[i].pix);
        else
          cam_log_.logMatrix(a);
      }
    }

    // Store measurements in a buffer to be published later
    cam_buffer_.push_back(image_);
  }

  // Publish measurement after sufficient time delay
  if (fabs(t - cam_buffer_.begin()->t) >= camera_time_delay_ && cam_buffer_.size() > 0)
  {
    new_camera_meas_ = true;
    image_= cam_buffer_[0];
    cam_buffer_.erase(cam_buffer_.begin());
  }
  else
  {
    new_camera_meas_ = false;
  }
}


void Sensors::mocap(const double t, const vehicle::Stated &x)
{
  double dt = common::round2dec(t - last_mocap_update_, t_round_off_);
  if (t == 0 || dt >= 1.0 / mocap_update_rate_)
  {
    last_mocap_update_ = t;
    if (!use_mocap_truth_)
      common::randomNormal(mocap_noise_,mocap_noise_dist_,rng_);

    // Populate mocap measurement
    mocap_truth_.t_ = x.p + x.q.rota(q_ub_.rotp(p_um_ - p_ub_));
    mocap_truth_.q_ = x.q * q_ub_.inverse() * q_um_;
    mocap_.transform.t_ = mocap_truth_.t_ + mocap_noise_.head<3>();
    mocap_.transform.q_ = mocap_truth_.q_ + mocap_noise_.tail<3>();
    mocap_.t = t;
    mocap_.id = mocap_id_++;

    // Log data
    mocap_log_.log(t);
    mocap_log_.logMatrix(mocap_.transform.arr_, mocap_truth_.arr_, p_um_, q_um_.arr_);

    // Store measurements in a buffer to be published later
    mocap_buffer_.push_back(mocap_);
  }

  // Publish measurement after sufficient time delay
  if (fabs(t - mocap_buffer_.begin()->t) >= mocap_time_delay_ && mocap_buffer_.size() > 0)
  {
    new_mocap_meas_ = true;
    mocap_= mocap_buffer_[0];
    mocap_buffer_.erase(mocap_buffer_.begin());
  }
  else
  {
    new_mocap_meas_ = false;
  }

}


void Sensors::baro(const double t, const vehicle::Stated& x)
{
  double dt = common::round2dec(t - last_baro_update_, t_round_off_);
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
    baro_.t = t;
    baro_.id = baro_id_++;
    baro_.pres = rho_ * common::gravity * -x.p(2) + baro_bias_ + baro_noise_;

    // Log data
    baro_log_.log(t, baro_.pres, baro_bias_, baro_noise_);
  }
  else
  {
    new_baro_meas_ = false;
  }
}


void Sensors::pitot(const double t, const vehicle::Stated& x, const Vector3d& vw)
{
  double dt = common::round2dec(t - last_pitot_update_, t_round_off_);
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
    pitot_.t = t;
    pitot_.id = pitot_id_++;
    double Va = common::e1.dot(q_b2pt_.rotp(x.v - x.q.rotp(vw)));
    pitot_.pres = 0.5 * rho_ * Va * Va + pitot_bias_ + pitot_noise_;

    // Log data
    pitot_log_.log(t, pitot_.pres, pitot_bias_, pitot_noise_);
  }
  else
  {
    new_pitot_meas_ = false;
  }
}


void Sensors::wvane(const double t, const vehicle::Stated& x, const Vector3d& vw)
{
  double dt = common::round2dec(t - last_wvane_update_, t_round_off_);
  if (t == 0 || dt >= 1.0 / wvane_update_rate_)
  {
    new_wvane_meas_ = true;
    last_wvane_update_ = t;
    if (!use_wvane_truth_)
      wvane_noise_ = wvane_noise_dist_(rng_);

    // Populate weather vane measurement
    wvane_.t = t;
    wvane_.id = wvane_id_++;
    Vector3d v_aI_b = x.v - x.q.rotp(vw);
    double wvane_true = asin(common::e2.dot(q_b2wv_.rotp(v_aI_b)) / v_aI_b.norm());
    int num_ticks = round((wvane_true + wvane_noise_) * wvane_resolution_ / (2.0 * M_PI));
    wvane_.angle = 2.0 * M_PI * num_ticks / wvane_resolution_;

    // Log data
    wvane_log_.log(t, wvane_.angle, wvane_true);
  }
  else
  {
    new_wvane_meas_ = false;
  }
}


void Sensors::gps(const double t, const vehicle::Stated& x)
{
  double dt = common::round2dec(t - last_gps_update_, t_round_off_);
  if (t == 0 || dt >= 1.0 / gps_update_rate_)
  {
    new_gps_meas_ = true;
    last_gps_update_ = t;
    if (!use_gps_truth_)
    {
      // Create noise
      common::randomNormal(gps_hpos_noise_,gps_hpos_noise_dist_,rng_);
      common::randomNormal(gps_hvel_noise_,gps_hvel_noise_dist_,rng_);
      gps_vpos_noise_ = gps_vpos_noise_dist_(rng_);
      gps_vvel_noise_ = gps_vvel_noise_dist_(rng_);

      // Update position biases
      gps_hpos_bias_(0) = exp(-gps_time_constant_ * dt) * gps_hpos_bias_(0) + gps_hpos_noise_(0);
      gps_hpos_bias_(1) = exp(-gps_time_constant_ * dt) * gps_hpos_bias_(1) + gps_hpos_noise_(1);
      gps_vpos_bias_ = exp(-gps_time_constant_ * dt) * gps_vpos_bias_ + gps_vpos_noise_;
    }

    // Calculate NED measurement
    gps_.t = t;
    gps_.id = gps_id_++;
    Vector3d gps_pos = x.p;
    Vector3d gps_vel = x.q.rota(x.v);

    // Add bias and noise to NED measurement
    gps_.pos.head<2>() = gps_pos.head<2>() + gps_hpos_bias_ + gps_hpos_noise_;
    gps_.pos(2) = gps_pos(2) + gps_vpos_bias_ + gps_vpos_noise_;
    gps_.vel.head<2>() = gps_vel.head<2>() + gps_hvel_noise_;
    gps_.vel(2) = gps_vel(2) + gps_vvel_noise_;

    // Convert measurement to ECEF
    gps_.pos = X_ecef2ned_.transforma(gps_.pos);
    gps_.vel = X_ecef2ned_.q_.rota(gps_.vel);

    // Log data
    gps_log_.log(t);
    gps_log_.logMatrix(gps_.pos, gps_.vel, gps_hpos_bias_);
    gps_log_.log(gps_vpos_bias_);
    gps_log_.logMatrix(gps_hpos_noise_);
    gps_log_.log(gps_vpos_noise_);
    gps_log_.logMatrix(gps_hvel_noise_);
    gps_log_.log(gps_vvel_noise_);
  }
  else
  {
    new_gps_meas_ = false;
  }
}


}
