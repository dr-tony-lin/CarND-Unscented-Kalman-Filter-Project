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

  // initial state vector
  x() = VectorXd(n_x);

  // initial covariance matrix
  P() = MatrixXd(n_x, n_x);

  Xsig_aug = MatrixXd(n_aug, 2 * n_aug + 1);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a = 20;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd = 0.8;

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

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
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
      x() = RadarToCRTV(meas_package.measurements());
    } else if (meas_package.sensor_type() == SensorType::LASER) {
      // Initialize state from laser measurement
      VectorXd measurements = meas_package.measurements();
      // px, py, v, psi, dpsi/dt
      x() << measurements[0], measurements[1], 0,
          atan2(measurements[1], measurements[0]), 0;
    }

    // Initialize P matrices
    P() << 1, 0, 0, 0, 0,
           0, 1, 0, 0, 0,
           0, 0, 1, 0, 0,
           0, 0, 0, 1, 0,
           0, 0, 0, 0, 1;

#ifdef VERBOSE_OUT
    std::cout << "Initial x: " << x() << std::endl;
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
  PredictBySigmaPoints(dt);
#ifdef VERBOSE_OUT
  std::cout << "Computing means ..." << std::endl;
#endif
  ComputeMeanOfSigmaPoints(x(), P(), Xsig_pred);
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::Update(const MeasurementPackage<SensorType>& meas_package) {
#ifdef VERBOSE_OUT
  std::cout << "Updating ..." << std::endl;
#endif
  PredictMeasurementFromSigmPoints(meas_package.sensor_type() ==
                                   SensorType::RADAR);

  // create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x, meas_package.measurements().rows());

  // calculate cross correlation matrix
  Tc.setZero();
  for (int i = 0; i < 2 * n_aug + 1; i++) {
    VectorXd dx = Xsig_pred.col(i) - x();
    VectorXd dz = Zsig.col(i) - z_pred;
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
  x() += K * z_diff;
  P() -= K * S * K.transpose();

#ifdef VERBOSE_OUT
  std::cout << "Updated state x: " << x() << std::endl;
  std::cout << "Updated state covariance P: " << P() << std::endl;
#endif
}

MatrixXd UKF::CreateSigmaPoints() {
  // create augmented mean vector
  VectorXd x_aug = VectorXd(n_aug);

  // create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug, n_aug);

  // create augmented mean state
  x_aug.block(0, 0, n_x, 1) = x();
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
  for (int i = 0; i < sigma.cols(); i++) {
    VectorXd d = sigma.col(i) - x;
    // Normalize angle
    if (sigma.rows() > 3) {
      d(3) = NormalizeAngle(d(3));
    } else if (sigma.rows() > 2) {
      d(1) = NormalizeAngle(d(1));
    }

    MatrixXd D = (d.matrix() * d.matrix().transpose());
    D *= weights(i);
    P += D;
  }

#ifdef VERBOSE_OUT
  std::cout << "Mean x: " << x << std::endl;
  std::cout << "Mean P: " << P << std::endl;
#endif
}

void UKF::PredictBySigmaPoints(const double dt) {
  // predict sigma points
  // avoid division by zero
  // write predicted sigma points into right column
  Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
  for (int i = 0; i < 2 * n_aug + 1; i++) {
    VectorXd col = Xsig_aug.col(i);
    if (col(4) == 0.0) {
      double vkdt = col(2) * dt;
      double dt2 = dt * dt / 2.0;
      double cos_psi = cos(col(3));
      double sin_psi = sin(col(3));
      Xsig_pred.col(i) << col(0) + vkdt * cos_psi + dt2 * cos_psi * col(5),
          col(1) + vkdt * sin_psi + dt2 * sin_psi * col(5),
          col(2) + dt * col(5), col(3) + dt2 * col(6), col(4) + dt * col(6);
    } else {
      double c1 = col(2) / col(4);
      double delta_psi = col(4) * dt;
      double npsi = col(3) + delta_psi;
      double dt2 = dt * dt / 2.0;
      double cos_psi = cos(col(3));
      double sin_psi = sin(col(3));
      Xsig_pred.col(i) << col(0) + c1 * (sin(npsi) - sin_psi) +
                              dt2 * cos_psi * col(5),
          col(1) + c1 * (-cos(npsi) + cos_psi) + dt2 * sin_psi * col(5),
          col(2) + dt * col(5), col(3) + delta_psi + dt2 * col(6),
          col(4) + dt * col(6);
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

  Zsig = radar ? PVToRadar(Xsig_pred) : 
                 Xsig_pred.block(0, 0, 2, Xsig_pred.cols());

#ifdef VERBOSE_OUT
  std::cout << "Compute mean, Zsig: " << Zsig.rows() << "x" << Zsig.cols()
            << ": " << Zsig << std::endl;
#endif

  // Compute state, and covarance matrix from the sigma points
  ComputeMeanOfSigmaPoints(z_pred, S, Zsig);
  std::cout << "Mean: " << z_pred << Zsig << std::endl;
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