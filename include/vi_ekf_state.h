#pragma once

#include <Eigen/Dense>
#include "geometry/quat.h"
#include "sensors.h"

using namespace std;
using namespace Eigen;


namespace qviekf
{


// State Indices
enum
{
  P = 0,    // POSITION
  V = 3,    // VELOCITY
  Q = 6,    // ATTITUDE
  BA = 10,  // ACCEL BIAS
  BG = 13,  // GYRO BIAS
  MU = 16,  // DRAG COEFFICIENT
  QU = 17,  // IMU TO BODY ROTATION
  QC = 21,  // IMU TO CAMERA ROTATION
  PC = 25   // IMU TO CAMERA TRANSLATION IN IMU FRAME
};

// Derivative indices
enum
{
  DP = 0,
  DV = 3,
  DQ = 6,
  DBA = 9,
  DBG = 12,
  DMU = 15,
  DQU = 16,
  DQC = 19,
  DPC = 22
};

// Input indices
enum
{
  UA = 0,
  UG = 3,
  NI = 6 // number of propagation inputs (imu)
};

typedef Matrix<double, NI, 1> uVector;
typedef Matrix<double, NI, NI> uMatrix;


template<typename T>
struct State
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  State() {}

  State(const bool& _drag_enabled, const bool& _q_ub_enabled,
        const bool& _q_uc_enabled, const bool& _p_uc_enabled, const int& _nf)
    : nbs(16), nbd(15)
  {
    drag_enabled = _drag_enabled;
    q_ub_enabled = _q_ub_enabled;
    q_uc_enabled = _q_uc_enabled;
    p_uc_enabled = _p_uc_enabled;
    if (drag_enabled)
    {
      nbs += 1;
      nbd += 1;
    }
    if (q_ub_enabled)
    {
      nbs += 4;
      nbd += 3;
    }
    if (q_uc_enabled)
    {
      nbs += 4;
      nbd += 3;
    }
    if (p_uc_enabled)
    {
      nbs += 3;
      nbd += 3;
    }

    nf = _nf;
    for (int i = 0; i < nf; ++i)
      feats.push_back(sensors::Feat());
  }

  void initBaseState(const VectorXd& x)
  {
    p = x.template segment<3>(P);
    v = x.template segment<3>(V);
    q = quat::Quat<T>(x.template segment<4>(Q).normalized());
    ba = x.template segment<3>(BA);
    bg = x.template segment<3>(BG);

    if (drag_enabled)
      mu = x(MU);
    if (q_ub_enabled)
      q_ub = quat::Quat<T>(x.template segment<4>(QU).normalized());
    if (q_uc_enabled)
      q_uc = quat::Quat<T>(x.template segment<4>(QC).normalized());
    if (p_uc_enabled)
      p_uc = x.template segment<3>(PC);
  }

  State<T> operator+(const VectorXd &delta) const
  {
    State<T> x(drag_enabled, q_ub_enabled, q_uc_enabled, p_uc_enabled, nf);
    x.p = p + delta.template segment<3>(DP);
    x.v = v + delta.template segment<3>(DV);
    x.q = q + delta.template segment<3>(DQ);
    x.ba = ba + delta.template segment<3>(DBA);
    x.bg = bg + delta.template segment<3>(DBG);
    if (drag_enabled)
      x.mu = mu + delta(DMU);
    if (q_ub_enabled)
      x.q_ub = q_ub + delta.template segment<3>(DQU);
    if (q_uc_enabled)
      x.q_uc = q_uc + delta.template segment<3>(DQC);
    if (p_uc_enabled)
      x.p_uc = p_uc + delta.template segment<3>(DPC);
    for (int i = 0; i < nf; ++i)
    {
      sensors::Feat f;
      f.pix = feats[i].pix + delta.template segment<2>(nbd+3*i);
      f.rho = feats[i].rho + delta(nbd+3*i+2);
      f.id = feats[i].id;
      x.feats[i] = f;
    }
    return x;
  }

  VectorXd operator-(const State<T> &x2) const
  {
    VectorXd dx(nbd+3*nf);
    dx.template segment<3>(DP) = p - x2.p;
    dx.template segment<3>(DV) = v - x2.v;
    dx.template segment<3>(DQ) = q - x2.q;
    dx.template segment<3>(DBA) = ba - x2.ba;
    dx.template segment<3>(DBG) = bg - x2.bg;
    if (drag_enabled)
      dx(DMU) = mu - x2.mu;
    if (q_ub_enabled)
      dx.template segment<3>(DQU) = q_ub - x2.q_ub;
    if (q_uc_enabled)
      dx.template segment<3>(DQC) = q_uc - x2.q_uc;
    if (p_uc_enabled)
      dx.template segment<3>(DPC) = p_uc - x2.p_uc;
    for (int i = 0; i < nf; ++i)
    {
      dx.template segment<2>(nbd+3*i) = feats[i].pix - x2.feats[i].pix;
      dx(nbd+3*i+2) = feats[i].rho - x2.feats[i].rho;
    }
    return dx;
  }

  void operator+=(const VectorXd &delta)
  {
    *this = *this + delta;
  }

  VectorXd vec() const
  {
    VectorXd x(nbs+3*nf);
    x.template segment<3>(P) = p;
    x.template segment<3>(V) = v;
    x.template segment<4>(Q) = q.elements();
    x.template segment<3>(BA) = ba;
    x.template segment<3>(BG) = bg;
    if (drag_enabled)
      x(MU) = mu;
    if (q_ub_enabled)
      x(QU) = q_ub.elements();
    if (q_uc_enabled)
      x(QC) = q_uc.elements();
    if (p_uc_enabled)
      x(PC) = p_uc;
    for (int i = 0; i < nf; ++i)
    {
      x.template segment<2>(nbs+3*i) = feats[i].pix;
      x(nbs+3*i+2) = feats[i].rho;
    }
    return x;
  }

  Matrix<T,3,1> p;
  Matrix<T,3,1> v;
  quat::Quat<T> q;
  Matrix<T,3,1> ba;
  Matrix<T,3,1> bg;
  T mu;
  quat::Quat<T> q_ub;
  quat::Quat<T> q_uc;
  Matrix<T,3,1> p_uc;
  sensors::FeatVec feats;
  int nf; // number of features
  int nbs; // number of base states
  int nbd; // number of base degrees of freedom
  bool drag_enabled;
  bool q_ub_enabled;
  bool q_uc_enabled;
  bool p_uc_enabled;

};
typedef State<double> Stated;


} // namespace qviekf
