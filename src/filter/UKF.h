#ifndef FILTER_UKF_H_
#define FILTER_UKF_H_

#include <string>
#include <vector>
#include "Eigen/Dense"
#include "KalmanFilter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

class UKF : public KalmanFilter {
 private:

  // check whether the filter has been initialized or not (first measurement)
  bool is_initialized;

  // Timestamp of the last measurement
  long long previous_timestamp;

  // if this is false, lidar measurements will be ignored (except for init)
  bool use_lidar = true;

  // if this is false, radar measurements will be ignored (except for init)
  bool use_radar = true;

  // predicted sigma points matrix
  MatrixXd Xsig_pred;

  // Augumented sigma points
  MatrixXd Xsig_aug;

  // Sigma points in measurement space
  MatrixXd Zsig;

  ///* time when the state is true, in us
  long long time_us;

  ///* Process noise standard deviation longitudinal acceleration in m/s^2
  double std_a;

  ///* Process noise standard deviation yaw acceleration in rad/s^2
  double std_yawdd;

  ///* Laser measurement noise standard deviation position1 in m
  double std_laspx;

  ///* Laser measurement noise standard deviation position2 in m
  double std_laspy;

  ///* Radar measurement noise standard deviation radius in m
  double std_radr;

  ///* Radar measurement noise standard deviation angle in rad
  double std_radphi;

  ///* Radar measurement noise standard deviation radius change in m/s
  double std_radrd;

  ///* Weights of sigma points
  VectorXd weights;

  VectorXd z_pred;
  
  // Measurement and measurement prediction difference
  VectorXd z_diff;

  // The Measurement covariance matrix
  MatrixXd S;

  ///* State dimension
  int n_x;

  ///* Augmented state dimension
  int n_aug;

  ///* Sigma point spreading parameter
  double lambda;

 protected:
  /**
   * Compute mean and covariance matrix from the sigma points
   * @param[out] x to store the computed mean
   * @param[out] P to store the computed covarance matrix
   * @param sigma the sigma points
   */
  void ComputeMeanOfSigmaPoints(VectorXd& x, MatrixXd& P,
                                const MatrixXd& sigma);

  /**
   * Create sigma points from the current prediction state
   * @return the sigma point
   */
  MatrixXd CreateSigmaPoints();

  /**
   * Run perdiction against the sigma points with the given delat time
   * @param dt delta time
   */
  void PredictBySigmaPoints(const double dt);

  /**
   * Predict state and its covariance from the means of sigma points.
   */
  void PredictStateFromSigmPoints();

  /**
   * Predict measurement and its covariance from the means of sigma points.
   * @param radar true for radar, false for lidar
   */
  void PredictMeasurementFromSigmPoints(const bool radar);

 public:
  /**
   * Constructor
   */
  UKF();

  /**
   * Destructor
   */
  virtual ~UKF();

  /**
   * Reset the filter to start another run
   */
  void Reset() { is_initialized = false; }

  /**
   * Turn use lidar flag on or off
   * @param value true to use lidar measurements
   */
  void UseLidar(bool value) { use_lidar = value; }

  /**
   * Get use lidar flag
   */
  bool UseLidar() { return use_lidar; }

  /**
   * Turn use radar flag on or off
   * @param value true to use radar measurements
   */
  void UseRadar(bool value) { use_radar = value; }

  /**
   * Get use radar flag
   */
  bool UseRadar() { return use_radar; }

  /**
   * Run the Kalman Filter estimation on the given sensor
   * measurement.
   * @param measurement_pack the MeasurementPackage
   * @return true if the estimate was made, false if the initialization was
   * performed.
   */
  bool ProcessMeasurement(
      const MeasurementPackage<SensorType>& measurement_pack);

  /**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} dt the change in time (in seconds) between the last
 * measurement and this one.
 */
  void Predict(double dt);

  /**
   * Updates the state and the state covariance matrix using a laser measurement
   * @param measurement_pack The measurement
   */
  void Update(const MeasurementPackage<SensorType>& measurement_pack);

  /**
   * Compute normalized innovation squared with the current update
   */
  double ComputeNIS();

  /**
   * Set standard deviation for acceleration
   * @param value the value
   */ 
  void StdA(double value) { std_a = value;}
  
  /**
   * Set standard deviation for yaw derivative
   * @param value the value
   */ 
  void StdYawD(double value) { std_yawdd = value;}
};

#endif /* UKF_H */
