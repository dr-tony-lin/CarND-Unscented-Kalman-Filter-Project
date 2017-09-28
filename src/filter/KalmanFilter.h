#ifndef FILTER_KALMAN_FILTER_H_
#define FILTER_KALMAN_FILTER_H_

#include "../sensor/MeasurementPackage.h"
#include "../sensor/SensorType.h"
#include "Eigen/Dense"

class KalmanFilter {
 private:
  // state vector
  Eigen::VectorXd x_;

  // state covariance matrix
  Eigen::MatrixXd P_;
  
 protected:
  // if this is false, lidar measurements will be ignored (except for init)
  bool use_lidar = true;

  // if this is false, radar measurements will be ignored (except for init)
  bool use_radar = true;

 public:
  /**
   * Reset the filter to start another run
   */
  virtual void Reset() {}

  /**
    * Run the Kalman Filter estimation on the given sensor
    * measurement.
    * @param measurement_pack the MeasurementPackage
    * @return true if the estimate was made, false if the initialization was
   * performed.
    */
  virtual bool ProcessMeasurement(
      const MeasurementPackage<SensorType> &measurement_pack) = 0;

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param dt Time between k and k+1 in s
   */
  virtual void Predict(const double dt) = 0;

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param measurement_pack The measurement package
   */
  virtual void Update(
      const MeasurementPackage<SensorType> &measurement_pack) = 0;

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
   * Get the state
   */
  Eigen::VectorXd &x() { return x_; };

  /**
   * Get the state covariance matrix
   */
  Eigen::MatrixXd &P() { return P_; };

 protected:
  /**
   * Constructor
   */
  KalmanFilter(){};

  /**
   * Destructor
   */
  virtual ~KalmanFilter(){};
};
#endif /* KALMAN_FILTER_H_ */
