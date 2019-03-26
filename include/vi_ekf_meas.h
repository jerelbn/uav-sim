#pragma once

#include <Eigen/Dense>
#include <set>
#include "vi_ekf_meas.h"

using namespace std;
using namespace Eigen;


namespace qviekf
{


enum
{
  IMU = 0,
  IMAGE = 1,
  GPS = 2,
  MOCAP = 3
};


class Measurement
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int type;
  double stamp;
  Vector6d imu, gps;
  xform::Xformd mocap;
  sensors::FeatVec image;

  Measurement(const int& _type, const double& _stamp, const Vector6d& _imu_or_gps)
    : type(_type), stamp(_stamp), imu(_imu_or_gps), gps(_imu_or_gps)
  {
    if (type == IMU)
      gps.setConstant(NAN);
    else if (type == GPS)
      imu.setConstant(NAN);
    mocap.arr_.setConstant(NAN);
    image.clear();
  }

  Measurement(const int& _type, const double& _stamp, const sensors::FeatVec& _image)
    : type(_type), stamp(_stamp), image(_image)
  {
    imu.setConstant(NAN);
    gps.setConstant(NAN);
    mocap.arr_.setConstant(NAN);
  }

  Measurement(const int& _type, const double& _stamp, const xform::Xformd& _mocap)
    : type(_type), stamp(_stamp), mocap(_mocap)
  {
    imu.setConstant(NAN);
    gps.setConstant(NAN);
    image.clear();
  }

  bool operator< (const Measurement& other) const
  {
    return stamp < other.stamp;
  }
};
typedef multiset<Measurement> Measurements;


} // namespace qviekf
