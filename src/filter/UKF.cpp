#include "UKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "utils.h"

#ifdef VERBOSE_OUT
#include <iostream>
#endif

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::ArrayXd;
using std::vector;
using namespace utils;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // State dimension
  n_x = 5;

  // Augmented state dimension
  n_aug = 7;

  // initial ctrv state vector
  ctrv = VectorXd(n_x);

  // initial state vector px, py, vx, vy
  x() = VectorXd(4);

  // initial covariance matrix
  P() = MatrixXd(n_x, n_x);

  Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a = 0.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd = 0.65;

  // Laser measurement noise standard deviation position1 in m
  std_laspx = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd = 0.3;

  // Sigma point spreading parameter
  lambda = 3. - n_aug;

  // predicted sigma points matrix
  Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);

  // set weights
  weights = VectorXd(2 * n_aug + 1);
  weights.fill(1.0 / (2 * (lambda + n_aug)));
  weights(0) = lambda / (lambda + n_aug);
}

UKF::~UKF() {}

bool UKF::ProcessMeasurement(
    const MeasurementPackage<SensorType>& meas_package) {
  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if ((meas_package.sensor_type() == SensorType::RADAR && !use_radar) ||
      (meas_package.sensor_type() == SensorType::LASER &&
       !use_lidar)) {  // skip
    return false;
  }

  if (!is_initialized) {  // first measurement
#ifdef VERBOSE_OUT
    std::cout << "Initializing UKF ... " << std::endl;
#endif
    previous_timestamp = meas_package.timestamp();
    if (meas_package.sensor_type() == SensorType::RADAR) {
      // Convert radar from polar to cartesian coordinates and initialize state.
      ctrv = RadarToCTRV(meas_package.measurements());
      x() << ctrv(0), ctrv(1), ctrv(2) * cos(ctrv(3)), ctrv(2) * sin(ctrv(3));
    } else if (meas_package.sensor_type() == SensorType::LASER) {
      // Initialize state from laser measurement
      VectorXd measurements = meas_package.measurements();
      // px, py, v, psi, dpsi/dt
      ctrv << measurements[0], measurements[1], 0,
          atan2(measurements[1], measurements[0]), 0;
      x() << ctrv(0), ctrv(1), 0, 0;
    }

    // Initialize P matrices
    P() << 1, 0, 0, 0, 0,
           0, 1, 0, 0, 0,
           0, 0, 1, 0, 0,
           0, 0, 0, 1, 0,
           0, 0, 0, 0, 1;

#ifdef VERBOSE_OUT
    std::cout << "Initial x: " << x() << std::endl;
    std::cout << "Initial CTRV: " << ctrv << std::endl;
    std::cout << "Initial P: " << P() << std::endl;
#endif
    // done initializing, no need to predict or update
    is_initialized = true;
    return false;
  }

  double dt = (meas_package.timestamp() - previous_timestamp) / 1000000.0;
  previous_timestamp = meas_package.timestamp();

#ifdef VERBOSE_OUT
  std::cout << "dt: " << dt
            << (meas_package.sensor_type() == SensorType::RADAR ? "R, " : "L: ")
            << meas_package.measurements().transpose() << std::endl;
#endif

  // Run prediction
  Predict(dt);

  // Run sensor update
  Update(meas_package);
  return true;
}

void UKF::Predict(double dt) {
#ifdef VERBOSE_OUT
  std::cout << "Creating sigma points ... " << std::endl;
#endif
  CreateSigmaPoints();
#ifdef VERBOSE_OUT
  std::cout << "Predict by sigma points ..." << std::endl;
#endif
PredictWithSigmaPoints(dt);
#ifdef VERBOSE_OUT
  std::cout << "Computing means ..." << std::endl;
#endif
  ComputeMeanOfSigmaPoints(ctrv, P(), Xsig_pred);
}

void UKF::Update(const MeasurementPackage<SensorType>& meas_package) {
#ifdef VERBOSE_OUT
  std::cout << "Updating ..." << std::endl;
#endif
  PredictMeasurementFromSigmPoints(meas_package.sensor_type() ==
                                   SensorType::RADAR);

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x, meas_package.measurements().rows());

  // Vectorize operations
  MatrixXd DX = Xsig_pred.colwise() - ctrv;
  MatrixXd DZ = Zsig.colwise() - z_pred;

  // calculate cross correlation matrix
  Tc.setZero();
  for (int i = 0; i < 2 * n_aug + 1; i++) {
    VectorXd dx = DX.col(i);
    VectorXd dz = DZ.col(i);
    if (meas_package.sensor_type() == SensorType::RADAR) {
      // normalize bearing angle to [pi, -pi)
      dx(3) = NormalizeAngle(dx(3));
      dz(1) = NormalizeAngle(dz(1));
    }

    MatrixXd D = (dx.matrix() * dz.matrix().transpose());
    D *= weights(i);
    Tc += D;
  }

#ifdef VERBOSE_OUT
  std::cout << "Calculate Kalman gain ..." << std::endl;
#endif

  // calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();

  // update state mean and covariance matrix
  z_diff = meas_package.measurements() - z_pred;
  if (meas_package.sensor_type() == SensorType::RADAR) {
    // Normalize the angle difference
    z_diff(1) = NormalizeAngle(z_diff(1));
  }
  ctrv += K * z_diff;
  // normalize the bearing angle
  ctrv(3) = NormalizeAngle(ctrv(3));
  x() << ctrv(0), ctrv(1), ctrv(2) * cos(ctrv(3)), ctrv(2) * sin(ctrv(3));
  P() -= K * S * K.transpose();

#ifdef VERBOSE_OUT
  std::cout << "Updated state x: " << ctrv << std::endl;
  std::cout << "Updated state covariance P: " << P() << std::endl;
#endif
}

MatrixXd UKF::CreateSigmaPoints() {
  // create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug);

  // create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug, n_aug);

  // create augmented mean state
  x_aug.block(0, 0, n_x, 1) = ctrv;
  x_aug(n_x) = 0;
  x_aug(n_x + 1) = 0;

  // create augmented covariance matrix
  P_aug.block(0, 0, n_x, n_x) = P();
  P_aug.block(0, n_x, n_aug, 2).setZero();
  P_aug.block(n_x, 0, 2, n_aug).setZero();
  P_aug(n_x, n_x) = std_a * std_a;
  P_aug(n_x + 1, n_x + 1) = std_yawdd * std_yawdd;

  // create square root matrix
  MatrixXd A = P_aug.llt().matrixL();
  A *= sqrt(lambda + n_aug);

  // create augmented sigma points
  Xsig_aug.col(0) = x_aug;
  Xsig_aug.block(0, 1, n_aug, n_aug) = A.colwise() + x_aug;
  Xsig_aug.block(0, n_aug + 1, n_aug, n_aug) = (-A).colwise() + x_aug;

#ifdef VERBOSE_OUT
  // Write verbose information to cout
  std::cout << "Sigma points: " << Xsig_aug << std::endl;
#endif
  return Xsig_aug;
}

/**
 * Compute state, and covarance matrix from the sigma points
 */
void UKF::ComputeMeanOfSigmaPoints(VectorXd& x, MatrixXd& P,
                                   const MatrixXd& sigma) {
  // compute mean
  x = sigma * weights;

  // Normalize angle
  if (sigma.rows() > 3) {
    x(3) = NormalizeAngle(x(3));
  } else if (sigma.rows() > 2) {
    x(1) = NormalizeAngle(x(1));
  }

  // compute covariance matrix
  P.setZero();

  // Vectorize operations
  MatrixXd DX = sigma.colwise() - x;

  // Compute the covariance matrix
  for (int i = 0; i < sigma.cols(); i++) {
    VectorXd d = DX.col(i);
    // Normalize angle
    if (sigma.rows() > 3) {
      d(3) = NormalizeAngle(d(3));
    } else if (sigma.rows() > 2) {
      d(1) = NormalizeAngle(d(1));
    }

    MatrixXd D = (d * d.transpose());
    D *= weights(i);
    P += D;
  }

#ifdef VERBOSE_OUT
  std::cout << "Mean x: " << x.transpose() << std::endl;
  std::cout << "Mean P: " << P << std::endl;
#endif
}

void UKF::PredictWithSigmaPoints(const double dt) {
  Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
  double dt2 = dt * dt / 2.0;

  // Vectorize matrix/vector operations
  ArrayXd cos_psi = Xsig_aug.row(3).array().cos();
  ArrayXd sin_psi = Xsig_aug.row(3).array().sin();
  ArrayXd x5dt2 = Xsig_aug.row(5) * dt2; // will be 15x1!
  ArrayXd dt2_cos = cos_psi * x5dt2;
  ArrayXd dt2_sin = sin_psi * x5dt2;
  ArrayXd delta_psi = Xsig_aug.row(4) * dt; // will be 15x1!
  ArrayXd npsi = Xsig_aug.row(3).array() + delta_psi.transpose();
  ArrayXd npsi_cos = cos_psi - npsi.cos();
  ArrayXd npsi_sin = npsi.sin() - sin_psi;

  // Compute the perdiction for row 2, 3, and 4
  Xsig_pred.row(2) = Xsig_aug.row(2).array() + Xsig_aug.row(5).array()*dt;
  Xsig_pred.row(3) = Xsig_aug.row(3).array() + Xsig_aug.row(6).array()*dt2;
  Xsig_pred.row(4) = Xsig_aug.row(4).array() + Xsig_aug.row(6).array()*dt;

  // Compute the prediction of row 0, and 1
  for (int i = 0; i < 2 * n_aug + 1; i++) {
    VectorXd col = Xsig_aug.col(i);
    VectorXd pred = Xsig_pred.col(i);
    if (fabs(col(4)) < EPSLION) { // Avoid divide by 0
      double vkdt = col(2) * dt;
      Xsig_pred.col(i)(0) = col(0) + vkdt*cos_psi(i) + dt2_cos(i);
      Xsig_pred.col(i)(1) = col(1) + vkdt*sin_psi(i) + dt2_sin(i);
    } else { 
      double c1 = col(2) / col(4);
      Xsig_pred.col(i)(0) = col(0) + c1*npsi_sin(i) + dt2_cos(i);
      Xsig_pred.col(i)(1) = col(1) + c1*npsi_cos(i) + dt2_sin(i);
      Xsig_pred.col(i)(3) += delta_psi(i); // adjust row 3
    }
  }

#ifdef VERBOSE_OUT
  std::cout << "Xsig_pred = " << Xsig_pred << std::endl;
#endif
}

void UKF::PredictMeasurementFromSigmPoints(const bool radar) {
  // create matrix for sigma points in measurement space
  if (radar) {
    S = MatrixXd(3, 3);
    z_pred = VectorXd(3);
  } else {
    S = MatrixXd(2, 2);
    z_pred = VectorXd(2);
  }

  Zsig = radar ? CTRVToRadar(Xsig_pred) : 
                 Xsig_pred.block(0, 0, 2, Xsig_pred.cols());

#ifdef VERBOSE_OUT
  std::cout << "Compute mean, Zsig: " << Zsig.rows() << "x" << Zsig.cols()
            << ": " << Zsig << std::endl;
#endif

  // Compute state, and covariance matrix from the sigma points
  ComputeMeanOfSigmaPoints(z_pred, S, Zsig);

#ifdef VERBOSE_OUT
  std::cout << "Mean z_pred: " << z_pred.transpose() << std::endl;
#endif

  if (radar) {
    S(0, 0) += std_radr * std_radr;
    S(1, 1) += std_radphi * std_radphi;
    S(2, 2) += std_radrd * std_radrd;
  } else {
    S(0, 0) += std_laspx * std_laspx;
    S(1, 1) += std_laspy * std_laspy;
  }

#ifdef VERBOSE_OUT
  std::cout << "z_pred: " << z_pred << std::endl;
  std::cout << "Zsig: " << Zsig << std::endl;
  std::cout << "S: " << S << std::endl;
#endif
}

double UKF::ComputeNIS() { return z_diff.transpose() * S.inverse() * z_diff; }