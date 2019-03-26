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
  MU = 16    // DRAG COEFFICIENT
};

// Derivative indices
enum
{
  DP = 0,
  DV = 3,
  DQ = 6,
  DBA = 9,
  DBG = 12,
  DMU = 15
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

  State(const double& _t, const Vector6d& _imu, const int& _nbs, const int& _nfm, const int& _nfa)
  {
    t = _t;
    imu = _imu;
    nfm = _nfm;
    nfa = _nfa;
    nbs = _nbs;
    nbd = nbs - 1;
    for (int i = 0; i < nfm; ++i)
      feats.push_back(sensors::Feat());
  }

  State(const double& _t, const Vector6d& _imu, const int& _nbs, const int& _nfm, const int& _nfa, const VectorXd &x)
  {
    t = _t;
    imu = _imu;
    nfm = _nfm;
    nfa = _nfa;
    nbs = _nbs;
    nbd = nbs - 1;
    p = x.template segment<3>(P);
    v = x.template segment<3>(V);
    q = quat::Quat<T>(x.template segment<4>(Q));
    ba = x.template segment<3>(BA);
    bg = x.template segment<3>(BG);
    if (nbs == 17)
      mu = x(MU);
    for (int i = 0; i < nfm; ++i)
      feats.push_back(sensors::Feat());
  }

  State<T> operator+(const VectorXd &delta) const
  {
    State<T> x(t, imu, nbs, nfm, nfa);
    x.p = p + delta.template segment<3>(DP);
    x.v = v + delta.template segment<3>(DV);
    x.q = q + delta.template segment<3>(DQ);
    x.ba = ba + delta.template segment<3>(DBA);
    x.bg = bg + delta.template segment<3>(DBG);
    if (nbs == 17)
      x.mu = mu + delta(DMU);
    for (int i = 0; i < nfm; ++i)
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
    VectorXd dx(nbd+3*nfm);
    dx.template segment<3>(DP) = p - x2.p;
    dx.template segment<3>(DV) = v - x2.v;
    dx.template segment<3>(DQ) = q - x2.q;
    dx.template segment<3>(DBA) = ba - x2.ba;
    dx.template segment<3>(DBG) = bg - x2.bg;
    if (nbs == 17)
      dx(DMU) = mu - x2.mu;
    for (int i = 0; i < nfm; ++i)
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
    VectorXd x(nbs+3*nfm);
    x.template segment<3>(P) = p;
    x.template segment<3>(V) = v;
    x.template segment<4>(Q) = q.elements();
    x.template segment<3>(BA) = ba;
    x.template segment<3>(BG) = bg;
    if (nbs == 17)
      x(MU) = mu;
    for (int i = 0; i < nfm; ++i)
    {
      x.template segment<2>(nbs+3*i) = feats[i].pix;
      x(nbs+3*i+2) = feats[i].rho;
    }
    return x;
  }

  double t; // time
  int nfa; // number of active features
  int nfm; // max number of features
  int nbs; // number of base states
  int nbd; // number of base degrees of freedom
  Vector6d imu;
  Matrix<T,3,1> p;
  Matrix<T,3,1> v;
  quat::Quat<T> q;
  Matrix<T,3,1> ba;
  Matrix<T,3,1> bg;
  T mu;
  sensors::FeatVec feats;

};
typedef State<double> Stated;


} // namespace qviekf
