#ifndef FILTER_UTILS_H_
#define FILTER_UTILS_H_
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

//
// @file
// @brief This file contains utility functions for EKF implementation
//
namespace utils {
/**
 * The smallest number to be considered as 0
 */
const float EPSLION = 0.00000001;

/**
 * 2&pi;
 */
const float PI_2 = 2 * M_PI;

/**
 * Helper function to convert radar measurement to P, V state
 * @param data radar measurement
 */
extern VectorXd RadarToPV(const VectorXd& data);

/**
 * Helper function to convert radar measurement to CTRV
 * @param data radar measurement
 */
extern VectorXd RadarToCTRV(const  VectorXd& data);
/**
 * Helper function to convert filter state to Radar state
 * @param pv the state vector
 */
extern VectorXd PVToRadar(const VectorXd& pv);

/**
 * Convert vectors of CTRV states to radar measurements
 * @param ctrv the matrix containing the ctrv vectors
 */
extern MatrixXd CTRVToRadar(const MatrixXd& ctrv);

/**
 * Helper function to normalize an angle so it fall in the range of [PI, -PI)
 * This is important for computing the angular difference for radar sensor data
 * Without the alignment, invalid state update might occur when crossing x or y
 * axes
 * @param angle the angle to normalize
 */
extern float NormalizeAngle(const float angle);
}
#endif /* TOOLS_H_ */
