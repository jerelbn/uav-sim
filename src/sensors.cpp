#include "sensors.h"


namespace sensors
{


Sensors::Sensors()
  : imu_id_(0), mocap_id_(0), image_id_(0), gps_id_(0), baro_id_(0), pitot_id_(0), wvane_id_(0)
{
  q_cbc_ = quat::Quatd(M_PI/2, 0, M_PI/2);
}


Sensors::Sensors(const Sensors& sensors)
{
  *this = sensors;
}


Sensors::~Sensors() {}


Sensors& Sensors::operator=(const Sensors& sensors) // Maybe not a good idea to copy all of this data often...
{
  imu_ = sensors.imu_;
  mocap_ = sensors.mocap_;
  image_ = sensors.image_;
  gps_ = sensors.gps_;
  baro_ = sensors.baro_;
  mag_ = sensors.mag_;
  pitot_ = sensors.pitot_;
  wvane_ = sensors.wvane_;

  new_imu_meas_ = sensors.new_imu_meas_;
  new_mocap_meas_ = sensors.new_mocap_meas_;
  new_camera_meas_ = sensors.new_camera_meas_;
  new_gps_meas_ = sensors.new_gps_meas_;
  new_baro_meas_ = sensors.new_baro_meas_;
  new_mag_meas_ = sensors.new_mag_meas_;
  new_pitot_meas_ = sensors.new_pitot_meas_;
  new_wvane_meas_ = sensors.new_wvane_meas_;

  rng_ = sensors.rng_;
  t_round_off_ = sensors.t_round_off_;
  origin_lat_ = sensors.origin_lat_;
  origin_lon_ = sensors.origin_lon_;
  origin_alt_ = sensors.origin_alt_;
  rho_ = sensors.rho_;
  mnp_ecef_ = sensors.mnp_ecef_;
  q_ecef_to_mnp_ = sensors.q_ecef_to_mnp_;

  // IMU
  use_accel_truth_ = sensors.use_accel_truth_;
  use_gyro_truth_ = sensors.use_gyro_truth_;
  imu_enabled_ = sensors.imu_enabled_;
  imu_id_ = sensors.imu_id_;
  last_imu_update_ = sensors.last_imu_update_;
  imu_update_rate_ = sensors.imu_update_rate_;
  accel_bias_ = sensors.accel_bias_;
  accel_noise_ = sensors.accel_noise_;
  accel_walk_ = sensors.accel_walk_;
  gyro_bias_ = sensors.gyro_bias_;
  gyro_noise_ = sensors.gyro_noise_;
  gyro_walk_ = sensors.gyro_walk_;
  accel_noise_dist_ = sensors.accel_noise_dist_;
  accel_walk_dist_ = sensors.accel_walk_dist_;
  gyro_noise_dist_ = sensors.gyro_noise_dist_;
  gyro_walk_dist_ = sensors.gyro_walk_dist_;
  q_bu_ = sensors.q_bu_;
  p_bu_ = sensors.p_bu_;
  // accel_log_ = sensors.accel_log_;
  // gyro_log_ = sensors.gyro_log_;

  // Camera
  use_camera_truth_ = sensors.use_camera_truth_;
  save_pixel_measurements_ = sensors.save_pixel_measurements_;
  camera_enabled_ = sensors.camera_enabled_;
  image_id_ = sensors.image_id_;
  cam_max_feat_ = sensors.cam_max_feat_;
  last_camera_update_ = sensors.last_camera_update_;
  camera_update_rate_ = sensors.camera_update_rate_;
  camera_time_delay_ = sensors.camera_time_delay_;
  cam_buffer_ = sensors.cam_buffer_;
  pixel_noise_dist_ = sensors.pixel_noise_dist_;
  depth_noise_dist_ = sensors.depth_noise_dist_;
  pixel_noise_ = sensors.pixel_noise_;
  K_inv_ = sensors.K_inv_;
  image_size_ = sensors.image_size_;
  q_cbc_ = sensors.q_cbc_;
  q_bcb_ = sensors.q_bcb_;
  p_bcb_ = sensors.p_bcb_;
  // cam_log_ = sensors.cam_log_;

  // Motion Capture
  use_mocap_truth_ = sensors.use_mocap_truth_;
  mocap_enabled_ = sensors.mocap_enabled_;
  mocap_id_ = sensors.mocap_id_;
  last_mocap_update_ = sensors.last_mocap_update_;
  mocap_update_rate_ = sensors.mocap_update_rate_;
  mocap_time_delay_ = sensors.mocap_time_delay_;
  mocap_buffer_ = sensors.mocap_buffer_;
  mocap_noise_dist_ = sensors.mocap_noise_dist_;
  mocap_noise_ = sensors.mocap_noise_;
  mocap_truth_ = sensors.mocap_truth_;
  p_bm_ = sensors.p_bm_;
  q_bm_ = sensors.q_bm_;
  // mocap_log_ = sensors.mocap_log_;

  // Barometer
  use_baro_truth_ = sensors.use_baro_truth_;
  baro_enabled_ = sensors.baro_enabled_;
  baro_id_ = sensors.baro_id_;
  last_baro_update_ = sensors.last_baro_update_;
  baro_update_rate_ = sensors.baro_update_rate_;
  baro_walk_dist_ = sensors.baro_walk_dist_;
  baro_noise_dist_ = sensors.baro_noise_dist_;
  baro_bias_ = sensors.baro_bias_;
  baro_walk_ = sensors.baro_walk_;
  baro_noise_ = sensors.baro_noise_;
  // baro_log_ = sensors.baro_log_;

  // Magnetometer
  use_mag_truth_ = sensors.use_mag_truth_;
  mag_enabled_ = sensors.mag_enabled_;
  mag_id_ = sensors.mag_id_;
  last_mag_update_ = sensors.last_mag_update_;
  mag_update_rate_ = sensors.mag_update_rate_;
  mag_walk_dist_ = sensors.mag_walk_dist_;
  mag_noise_dist_ = sensors.mag_noise_dist_;
  mag_bias_ = sensors.mag_bias_;
  mag_walk_ = sensors.mag_walk_;
  mag_noise_ = sensors.mag_noise_;
  q_bmag_ = sensors.q_bmag_;
  // mag_log_ = sensors.mag_log_;

  // Pitot Tube (for air speed along some axis)
  use_pitot_truth_ = sensors.use_pitot_truth_;
  pitot_enabled_ = sensors.pitot_enabled_;
  pitot_id_ = sensors.pitot_id_;
  last_pitot_update_ = sensors.last_pitot_update_;
  pitot_update_rate_ = sensors.pitot_update_rate_;
  pitot_walk_dist_ = sensors.pitot_walk_dist_;
  pitot_noise_dist_ = sensors.pitot_noise_dist_;
  pitot_bias_ = sensors.pitot_bias_;
  pitot_walk_ = sensors.pitot_walk_;
  pitot_noise_ = sensors.pitot_noise_;
  q_bpt_ = sensors.q_bpt_;
  // pitot_log_ = sensors.pitot_log_;

  // Weather vane (for sideslip angle)
  use_wvane_truth_ = sensors.use_wvane_truth_;
  wvane_enabled_ = sensors.wvane_enabled_;
  wvane_id_ = sensors.wvane_id_;
  last_wvane_update_ = sensors.last_wvane_update_;
  wvane_update_rate_ = sensors.wvane_update_rate_;
  wvane_noise_dist_ = sensors.wvane_noise_dist_;
  wvane_resolution_ = sensors.wvane_resolution_;
  wvane_noise_ = sensors.wvane_noise_;
  q_bwv_ = sensors.q_bwv_;
  // wvane_log_ = sensors.wvane_log_;

  // GPS
  use_gps_truth_ = sensors.use_gps_truth_;
  gps_enabled_ = sensors.gps_enabled_;
  gps_id_ = sensors.gps_id_;
  last_gps_update_ = sensors.last_gps_update_;
  gps_update_rate_ = sensors.gps_update_rate_;
  gps_time_constant_ = sensors.gps_time_constant_;
  gps_hpos_noise_dist_ = sensors.gps_hpos_noise_dist_;
  gps_hvel_noise_dist_ = sensors.gps_hvel_noise_dist_;
  gps_vpos_noise_dist_ = sensors.gps_vpos_noise_dist_;
  gps_vvel_noise_dist_ = sensors.gps_vvel_noise_dist_;
  gps_hpos_bias_ = sensors.gps_hpos_bias_;
  gps_vpos_bias_ = sensors.gps_vpos_bias_;
  gps_hpos_noise_ = sensors.gps_hpos_noise_;
  gps_hvel_noise_ = sensors.gps_hvel_noise_;
  gps_vpos_noise_ = sensors.gps_vpos_noise_;
  gps_vvel_noise_ = sensors.gps_vvel_noise_;
  X_ecef2ned_ = sensors.X_ecef2ned_;
  // gps_log_ = sensors.gps_log_;

  return *this;
}


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
  mnp_ecef_ = (WGS84::lla2ecef(Vector3d(common::MNP_lat*M_PI/180.0, common::MNP_lon*M_PI/180.0, 0.0))).normalized();

  Vector3d axis = (common::e3.cross(mnp_ecef_)).normalized();
  double angle = common::angDiffBetweenVecs(common::e3, mnp_ecef_);
  q_ecef_to_mnp_.from_axis_angle(axis, angle);

  // IMU
  stringstream ss_accel;
  double accel_bias_init_bound, accel_noise_stdev, accel_walk_stdev;
  Vector4d q_bu;
  common::get_yaml_node("imu_enabled", filename, imu_enabled_);
  common::get_yaml_node("imu_update_rate", filename, imu_update_rate_);
  common::get_yaml_eigen("q_bu", filename, q_bu);
  common::get_yaml_eigen("p_bu", filename, p_bu_);
  common::get_yaml_node("use_accel_truth", filename, use_accel_truth_);
  common::get_yaml_node("accel_noise_stdev", filename, accel_noise_stdev);
  common::get_yaml_node("accel_walk_stdev", filename, accel_walk_stdev);
  common::get_yaml_node("accel_bias_init_bound", filename, accel_bias_init_bound);
  q_bu_ = quat::Quatd(q_bu.normalized());
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
  double pixel_noise_stdev, depth_noise_stdev;
  Vector4d q_bcb;
  common::get_yaml_node("camera_enabled", filename, camera_enabled_);
  common::get_yaml_node("camera_max_features", filename, cam_max_feat_);
  common::get_yaml_node("use_camera_truth", filename, use_camera_truth_);
  common::get_yaml_node("save_pixel_measurements", filename, save_pixel_measurements_);
  common::get_yaml_node("camera_update_rate", filename, camera_update_rate_);
  common::get_yaml_node("camera_time_delay", filename, camera_time_delay_);
  common::get_yaml_node("pixel_noise_stdev", filename, pixel_noise_stdev);
  common::get_yaml_node("depth_noise_stdev", filename, depth_noise_stdev);
  common::get_yaml_eigen("image_size", filename, image_size_);
  common::get_yaml_eigen("camera_matrix", filename, K_);
  common::get_yaml_eigen("q_bcb", filename, q_bcb);
  common::get_yaml_eigen("p_bcb", filename, p_bcb_);
  pixel_noise_dist_ = normal_distribution<double>(0.0, pixel_noise_stdev);
  depth_noise_dist_ = normal_distribution<double>(0.0, depth_noise_stdev);
  K_inv_ = K_.inverse();
  q_bcb_ = quat::Quatd(q_bcb.normalized());
  pixel_noise_.setZero();
  new_camera_meas_ = false;
  last_camera_update_ = 0.0;
  image_.feats.reserve(cam_max_feat_);
  ss_cam << "/tmp/" << name << "_camera.log";
  cam_log_.open(ss_cam.str());

  // Motion Capture
  stringstream ss_mocap;
  double mocap_noise_stdev;
  Vector4d q_bm;
  common::get_yaml_node("mocap_enabled", filename, mocap_enabled_);
  common::get_yaml_node("use_mocap_truth", filename, use_mocap_truth_);
  common::get_yaml_node("mocap_update_rate", filename, mocap_update_rate_);
  common::get_yaml_node("mocap_time_delay", filename, mocap_time_delay_);
  if (mocap_time_delay_ < 0)
    throw runtime_error("Cannot have a negative motion capture time delay!");
  common::get_yaml_node("mocap_noise_stdev", filename, mocap_noise_stdev);
  common::get_yaml_eigen("q_bm", filename, q_bm);
  common::get_yaml_eigen("p_bm", filename, p_bm_);
  q_bm_ = quat::Quatd(q_bm.normalized());
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

  // Magnetometer
  stringstream ss_mag;
  double mag_bias_init_bound, mag_noise_stdev, mag_walk_stdev;
  Vector4d q_bmag;
  common::get_yaml_node("mag_enabled", filename, mag_enabled_);
  common::get_yaml_node("mag_update_rate", filename, mag_update_rate_);
  common::get_yaml_node("use_mag_truth", filename, use_mag_truth_);
  common::get_yaml_node("mag_noise_stdev", filename, mag_noise_stdev);
  common::get_yaml_node("mag_walk_stdev", filename, mag_walk_stdev);
  common::get_yaml_node("mag_bias_init_bound", filename, mag_bias_init_bound);
  common::get_yaml_eigen("q_bmag", filename, q_bmag);
  q_bmag_ = quat::Quatd(q_bmag.normalized());
  mag_noise_dist_ = normal_distribution<double>(0.0, mag_noise_stdev);
  mag_walk_dist_ = normal_distribution<double>(0.0, mag_walk_stdev);
  mag_bias_ = mag_bias_init_bound * Vector3d::Random();
  mag_noise_.setZero();
  if (use_mag_truth_)
    mag_bias_.setZero();
  new_mag_meas_ = false;
  last_mag_update_ = 0.0;
  ss_mag << "/tmp/" << name << "_mag.log";
  mag_log_.open(ss_mag.str());

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
  q_bpt_ = quat::Quatd(0, pitot_elevation, pitot_azimuth);
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
  q_bwv_ = quat::Quatd(wvane_roll, 0, 0);
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


void Sensors::updateMeasurements(const double t, const vehicle::Stated &x, environment::Environment& env)
{
  // Update enabled sensor measurements
  if (imu_enabled_)
    imu(t, x);
  if (camera_enabled_)
    camera(t, x, env);
  if (mocap_enabled_)
    mocap(t, x);
  if (baro_enabled_)
    baro(t, x);
  if (mag_enabled_)
    mag(t, x);
  if (pitot_enabled_)
    pitot(t, x, env.getWindVel());
  if (wvane_enabled_)
    wvane(t, x, env.getWindVel());
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
    quat::Quatd q_Iu = x.q * q_bu_;
    imu_.accel = q_bu_.rotp(x.lin_accel + x.omega.cross(x.v) + x.omega.cross(x.omega.cross(p_bu_)) +
                 x.ang_accel.cross(p_bu_)) - common::gravity * q_Iu.rotp(common::e3) + accel_bias_ + accel_noise_;
    imu_.gyro = q_bu_.rotp(x.omega) + gyro_bias_ + gyro_noise_;
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


void Sensors::camera(const double t, const vehicle::Stated &x, environment::Environment &env)
{
  double dt = common::round2dec(t - last_camera_update_, t_round_off_);
  if (t == 0 || dt >= 1.0 / camera_update_rate_)
  {
    last_camera_update_ = t;

    // Project landmarks into image
    image_.feats.clear();
    for (int i = 0; i < env.getLandmarks().size(); ++i)
    {
      // Landmark vector in camera frame
      Vector3d p_cl = q_cbc_.rotp(q_bcb_.rotp(x.q.rotp(env.getLandmarks()[i] - x.p) - p_bcb_));

      // Check if landmark is in front of camera
      if (p_cl(2) < 0)
        continue;

      // Project landmark into image and save it to the list
      Vector2d pix;
      common::projToImg(pix, p_cl, K_);
      if (!use_camera_truth_)
      {
        common::randomNormal(pixel_noise_,pixel_noise_dist_,rng_);
        pix += pixel_noise_;
      }
      if (pix(0) >= 1 && pix(1) >= 1 && pix(0) <= image_size_(0) && pix(1) <= image_size_(1))
        image_.feats.push_back(common::Featd(i,pix,p_cl));

      if (image_.feats.size() == cam_max_feat_) break;
    }
    image_.t = t;
    image_.id = image_id_++;

    // Partition image into a grid
    const int cell_size = env.getGridCellFrac() * image_size_(1);
    const int grid_width = (image_size_(0) + cell_size - 1) / cell_size;
    const int grid_height = (image_size_(1) + cell_size - 1) / cell_size;
    std::vector<bool> grid(grid_width*grid_height, 0);

    // Add each existing image feature to the grid
    for(int i = 0; i < image_.feats.size(); ++i)
    {
      // Get feature point components
      double pix_x = image_.feats[i].pix(0);
      double pix_y = image_.feats[i].pix(1);

      // Determine points position in the grid
      int x_cell = (int)pix_x / cell_size;
      int y_cell = (int)pix_y / cell_size;

      // Indicate that this cell is populated
      grid[y_cell*grid_width+x_cell] = 1;
    }

    // Create new image feature in empty grid cells
    for (int i = 0; i < grid.size(); ++i)
    {
      if (grid[i] == 0)
      {
        // Unpack image coordinates
        int y_cell = i / grid_width;
        int x_cell = i - y_cell*grid_width;

        // Randomly choose pixel position within the cell
        double pix_x = x_cell*cell_size + std::rand()/double(RAND_MAX)*cell_size;
        double pix_y = y_cell*cell_size + std::rand()/double(RAND_MAX)*cell_size;
        Vector2d new_pix(pix_x, pix_y);

        // Get direction vector from pixel points and rotate it to inertial frame
        Vector3d l_c;
        common::dirFromPix(l_c, new_pix, K_inv_);
        Vector3d l = x.q.rota(q_bcb_.rota(q_cbc_.rota(l_c)));

        // Find intersection between direction vector and the environment box (assuming we're inside the box)
        Vector3d l0 = x.p + x.q.rota(p_bcb_);
        double d = 1e9;
        for (auto& plane : env.getPlanes())
        {
          Vector3d p0 = plane.r;
          Vector3d n = plane.n;
          double l_dot_n = l.dot(n);
          if (l_dot_n >= 0)
            continue;
          double d_new = (p0 - l0).dot(n) / l_dot_n;
          if (d_new < d)
            d = d_new;
        }

        // Add some variance to depth of new landmark and add noise to pixel measurement
        new_pix += pixel_noise_;
        d += env.getDepthVariation() * 2.0*(std::rand()/double(RAND_MAX) - 0.5);

        // Store new landmark in environment and in the image
        env.addLandmark(l0 + d*l);
        image_.feats.push_back(common::Featd(i, new_pix, d*l_c));
      }
    }

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
    mocap_truth_.t_ = x.p + x.q.rota(p_bm_);
    mocap_truth_.q_ = x.q * q_bm_;
    mocap_.transform.t_ = mocap_truth_.t_ + mocap_noise_.head<3>();
    mocap_.transform.q_ = mocap_truth_.q_ + mocap_noise_.tail<3>();
    mocap_.t = t;
    mocap_.id = mocap_id_++;

    // Log data
    mocap_log_.log(t);
    mocap_log_.logMatrix(mocap_.transform.arr_, mocap_truth_.arr_, p_bm_, q_bm_.arr_);

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


void Sensors::mag(const double t, const vehicle::Stated& x)
{
  double dt = common::round2dec(t - last_mag_update_, t_round_off_);
  if (t == 0 || dt >= 1.0 / mag_update_rate_)
  {
    new_mag_meas_ = true;
    last_mag_update_ = t;
    if (!use_mag_truth_)
    {
      common::randomNormal(mag_noise_,mag_noise_dist_,rng_);
      common::randomNormal(mag_walk_,mag_walk_dist_,rng_);
      mag_bias_ += mag_walk_ * dt;
    }

    // Calculate magnetic field vector at current location
    // NOTE: MNP coordinates are rotated by the shortest rotation between ECEF Z-axis and the magnetic north pole
    Vector3d ecef_pos = X_ecef2ned_.transforma(x.p);
    double theta = common::angDiffBetweenVecs(ecef_pos, mnp_ecef_);
    double Re_r = common::R_earth / ecef_pos.norm();
    double Re_r3 = Re_r * Re_r * Re_r;
    double Br = -2.0 * common::B0 * Re_r3 * cos(theta);
    double Btheta = -common::B0 * Re_r3 * sin(theta);
    Vector3d B_mnp(-Btheta, 0.0, -Br);

    // Populate magnetic field measurement (currently uses simple dipole model)
    mag_.field = q_bmag_.rotp(x.q.rotp(X_ecef2ned_.q_.rotp(q_ecef_to_mnp_.rota(B_mnp)) + mag_bias_)) + mag_noise_;
    mag_.t = t;
    mag_.id = mag_id_++;

    // Log data
    mag_log_.log(t);
    mag_log_.logMatrix(mag_.field, x.q.rota(mag_bias_), mag_noise_);
  }
  else
  {
    new_mag_meas_ = false;
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
    double Va = common::e1.dot(q_bpt_.rotp(x.v - x.q.rotp(vw)));
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
    double wvane_true = asin(common::e2.dot(q_bwv_.rotp(v_aI_b)) / v_aI_b.norm());
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
    Vector3d ned_pos = x.p;
    Vector3d ned_vel = x.q.rota(x.v);

    // Add bias and noise to NED measurement
    gps_.pos.head<2>() = ned_pos.head<2>() + gps_hpos_bias_ + gps_hpos_noise_;
    gps_.pos(2) = ned_pos(2) + gps_vpos_bias_ + gps_vpos_noise_;
    gps_.vel.head<2>() = ned_vel.head<2>() + gps_hvel_noise_;
    gps_.vel(2) = ned_vel(2) + gps_vvel_noise_;

    // Convert measurement to ECEF
    gps_.pos = X_ecef2ned_.transforma(gps_.pos);
    gps_.vel = X_ecef2ned_.q_.rota(gps_.vel);

    // Log data
    gps_log_.log(t);
    gps_log_.logMatrix(gps_.pos, gps_.vel, X_ecef2ned_.transforma(ned_pos), X_ecef2ned_.q_.rota(ned_vel));
  }
  else
  {
    new_gps_meas_ = false;
  }
}


}
