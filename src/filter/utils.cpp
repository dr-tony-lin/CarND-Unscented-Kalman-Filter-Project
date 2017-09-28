#include "utils.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::ArrayXd;
using std::vector;
using namespace utils;

VectorXd utils::RadarToPV(const VectorXd &radar) {
  VectorXd pv(4);
  float c = cos(radar(1));
  float s = sin(radar(1));
  pv << radar(0) * c, radar(0) * s, radar(2) * c, radar(2) * s;
  return pv;
}

VectorXd utils::RadarToCTRV(const  VectorXd& radar) {
  VectorXd pv(5);
  float c = cos(radar(1));
  float s = sin(radar(1));
  pv << radar(0) * c, radar(0) * s, radar(2), radar(1), 0;
  return pv;
}

float utils::NormalizeAngle(float a) {
  while (a > M_PI) a -= 2. * M_PI;
  while (a <= -M_PI) a += 2. * M_PI;
  return a;
}

VectorXd utils::PVToRadar(const VectorXd& pv) {
  VectorXd radar(3);
  float r = sqrt(pv(0) * pv(0) + (pv(1) * pv(1)));
  if (r > EPSLION) {
    float phi = atan2(pv(1), pv(0));
    float v = (pv(0) * pv(2) + pv(1) * pv(3)) / r;
    radar << r, phi, v;
  }
  return radar;
}

MatrixXd utils::PVToRadar(const MatrixXd& pv) {
  MatrixXd radar(3, pv.cols());
  radar.row(0) = (pv.row(0).array() * pv.row(0).array() +
                  pv.row(1).array() * pv.row(1).array())
                     .sqrt();
  for (int i = 0; i < radar.cols(); i++) {
    if (radar(0, i) < EPSLION) {
      radar(0, i) = EPSLION;
    }
    radar(1, i) = atan2(pv(1, i), pv(0, i));
  }

  radar.row(2) = (pv.row(0).array() * pv.row(3).array().cos() +
                  pv.row(1).array() * pv.row(3).array().sin()) *
                 pv.row(2).array() / radar.row(0).array();

  return radar;
}

VectorXd utils::CTRVToRadar(const VectorXd& ctrv) {
  VectorXd radar(3);
  float r = sqrt(ctrv(0) * ctrv(0) + (ctrv(1) * ctrv(1)));
  if (r > EPSLION) {
    float v = (ctrv(0) * ctrv(2) + ctrv(1) * ctrv(3)) / r;
    radar << r, ctrv(3), v;
  }
  return radar;
}

MatrixXd utils::CTRVToRadar(const MatrixXd& ctrv) {
  MatrixXd radar(3, ctrv.cols());
  radar.row(0) = (ctrv.row(0).array() * ctrv.row(0).array() +
                  ctrv.row(1).array() * ctrv.row(1).array()).sqrt();
  radar.row(1) = ctrv.row(3);
  radar.row(2) = (ctrv.row(0).array() * ctrv.row(3).array().cos() +
                  ctrv.row(1).array() * ctrv.row(3).array().sin()) *
                 ctrv.row(2).array() / radar.row(0).array();

  return radar;
}