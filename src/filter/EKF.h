#ifndef FILTER_FUSIONEKF_H_
#define FILTER_FUSIONEKF_H_

#include <fstream>
#include <string>
#include <vector>
#include "Eigen/Dense"
#include "KalmanFilter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

class EKF : public KalmanFilter {
 protected:
  /**
   * The minimal time threshold that a measurement and its prior measurement
   * are considered happening at different time.
   */
  const float MINIMAL_TIME_THRESHOLD = 0.000001;

  /**
   * Calculate the Jacobian matrix from the state
   * @param state the state vector of [px, py, vx, vy]
   */
  static MatrixXd CalculateJacobian(const VectorXd &state);

  /**
   * Update state with lidar measurements
   * @param z the measurements
   */
  void UpdateLidar(const VectorXd &z);

  /**
   * Update state with radar measurements
   * @param z the measurements
   */
  void UpdateRadar(const VectorXd &z);

 public:
  /**
  * Constructor.
  */
  EKF();

  /**
   * Destructor.
   */
  virtual ~EKF();

  /**
   * Reset the filter to start another run
   */
  void Reset() { is_initialized = false; }

  /**
    * Run the Kalman Filter estimation on the given sensor
    * measurement.
    * @param measurement_pack the MeasurementPackage
    * @return true if the estimate was made, false if the initialization was
   * performed.
    */
  bool ProcessMeasurement(
      const MeasurementPackage<SensorType> &measurement_pack);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param delta_T Time between k and k+1 in s
   */
  void Predict(const double dt);

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const MeasurementPackage<SensorType> &measurement_pack);

  /**
     * Set standard deviation for acceleration
     * @param value the value
     */
  void StdA(double value) { s_ax = s_ay = value; }

 private:
  // check whether the tracking toolbox was initialized or not (first
  // measurement)
  bool is_initialized;

  // previous timestamp
  long long previous_timestamp;

  // state transition matrix
  MatrixXd F;

  // process covariance matrix
  MatrixXd Q;

  // measurement matrix
  MatrixXd H;

  // measurement covariance matrix
  MatrixXd R;

  // measurement covariance matrix for laser
  MatrixXd R_laser;

  // measurement covariance matrix for radar
  MatrixXd R_radar;

  // measurement matrix for laser
  MatrixXd H_laser;

  // measurement matrix for radar
  MatrixXd Hj;

  /**
   * Precomputed identity matrix same size as the state covariance matrix.
   * This is computed when setting the state covariance matrix P in
        P(const Eigen::MatrixXd value)
   */
  MatrixXd I;

  // acceleration noise x
  float s_ax;

  // acceleration noise y
  float s_ay;
};

#endif /* FusionEKF_H_ */
