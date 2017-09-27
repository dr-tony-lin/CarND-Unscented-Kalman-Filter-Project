#ifndef SENSOR_MEASUREMENT_PACKAGE_H_
#define SENSOR_MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

/**
 * This class represents a sensor measurement package containing
 * the timestamp, the sensor type, and the measurements
 */
template<typename T> class MeasurementPackage {
private:
  long long timestamp_;
  T sensor_type_;
  Eigen::VectorXd measurements_;

public:
  /**
   * Construct a new measurement package
   * @param timestamp the timestamp when the measurement was obtained
   * @param sensor_type type of the sensor
   * @param measurements the measurements
   */
  MeasurementPackage(long long timestamp, T sensor_type, Eigen::VectorXd measurements) {
    timestamp_ = timestamp;
    sensor_type_ = sensor_type;
    measurements_ = measurements;
  };

  /**
   * Get the timestamp
   */
  long long timestamp() const {
    return timestamp_;
  };

  /**
   * Get the sensor type
   */
  T sensor_type() const {
    return sensor_type_;
  };

  /**
   * Get the measurements
   */
  Eigen::VectorXd measurements() const {
    return measurements_;
  };
};

#endif /* MEASUREMENT_PACKAGE_H_ */
