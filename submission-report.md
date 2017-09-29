# Extended Kalman Filter Project

[video1]: ./video1.mp4
[image1]: ./nis-05-065.png
[image2]: ./nis-01-01.png
[image3]: ./nis-005-005.png
[image3]: ./nis-8-2.png
[image3]: ./nis-16-6.png

## Content of the Submission and Usage
This submission includes the following c++ files:
* main.cpp: the main function that communicate with the simulator and drive the estimation process using UKF.
* filter/UKF.h, filter/UKF.cpp: contains UKF implementation
* filter/EKF.h, filter/EKF.cpp: contains EKF implementation
* filter/KalmanFilter.h: define Kalman filter abstraction
* filter/utils.h, filter/utils.cpp]: contains some utility functions used by UKF
* performance/RMSEEvaluator.h: A template class for memory-efficient evaluation of RMSE
* sensor/SensorType.h: defines SensorType enum
* sensor/MeasurementPackage.h defines sensor measurement package structure

### Usage
The program has been tuned to run with the optimized parameters without any additional command line option.
However, it can be launched as:

    ./kalman [-stda number] [-stdyawdd number] [-nolidar|-nolaser] [-noradar] [-nis filename] [-ekf]

It takes the following command line options:

* -stda: specifies the acceleration noise, e.g. -stda 0.5, the default is 0.5
* -stdyawdd: specifies the yaw acceleration noise (0 to  2&#960;), e.g. -stdyawdd 0.65, the default is 0.65
* -nolidar, -nolaser: turns off laser measurements
* -noradar: turns off radar signals
* -nis: turns on NIS (Normalized Innovation Squared) and specifies the file to store NIS. NIS is stored with comma separated values. Each NIS record contains the sequence, the min, the max, and the mean in the corresponding period of 10 estimates.
* -ekf: uses Exteded Kalman Filters instead of Unscented Kalman Filters

The program will listen on port 4567 for an incoming simulator connection. Only one simulator should be connected at anytime, though the program does not prohibit it. To start a new simulator, terminate the existing one first, then start a new one.

To start the simulator:

#### On Windows

    term2_sim9.exe path_to_obj_pose-laser-radar-synthetic-input.txt

#### Other platforms:

    ./term2_sim9 path_to_obj_pose-laser-radar-synthetic-input.txt

#### Run multiple simulations
*UKF* allows one to run multiple simulations, whether from the same simulator or different simulators, one at a time without having to restart it. This allows the RMSE to be evaluated for a much bigger set of estimates.

#### Build
For Windows, Bash on Ubuntu on Windows should be used. Please follow the *Tips for Setting Up Your Environment in Extended Kalman Filter project"  on how to setup and configure the build environment.

To build the program, invoke the following commands on the bash terminal:
```
mkdir build
cd build
cmake ..
make
```

The program can be built to output more information for disgnosis purpose by defining **VERBOSE_OUT** macro.

#### Build API Documentation
The documentation for functions, classes and methods are included in the header files in Doxygen format. To generate Api documentation with the included doxygen.cfg:

1. Make sure doxygen is installed
2. cd to the root folder of this project
3. Issue the following command:

    doxygen doxygen.cfg

## The Implementation

### The source code structure
The src folder contains three folders for the source files:
. filter: contains KalmanFilter.h, UKF.[h,cpp], and utils.[h,cpp] files which implement Unscented Kalman Filter.
. performance: contains RMSEEvaluator.h which provides RMSE evaluation functionality.
. sensor: contains source code for defining sensor measurement.

##### File names
In this project, I made a class to have its own .h, and .cpp files. More specifically, a class source file contains exactly one class, and has the same name as the class it contains. On the other hand, 

#### The libs folder
The libs folder contains Eigen and json.hpp required by the program.

## KalmanFilter class
This class define the baseline Kalman filter functionalities including the **ProcessMeasurement()**, **Predict()**, and **Update()** methods that need to be provided by a subclass.

It's data members include the state, and the state covariance matrices. Refer to the [API documentation](api/html/classKalmanFilter.html).

## UKF class
This class extends KalmanFilter class and implements an Unscented Kalman Filter.

The API documentation for UKF can be found [here](api/html/classUKF.html).

### The Unscented Kalman Filter Process
The process is implemented in method *ProcessMeasurement()*.

### Initialization
The UKF is initialized upon receiving the first measurement. 

#### CTRV Initialization
If the first measurement is from radar. The state is computed from the radar's measurement using *utils::RadarToCTRV()* function.

If the first measurement is from lidar, the measured position is used to populate the CTRV state: P<sub>x</sub> , P<sub>y</sub> , and &#936;.

#### State Covariance Initialization
The covariance is initialized as a 5x5 identity matrix.

The *ProcessMeasurement()* method returns immediately after the initialized is completed.

#### Skipping Measurements
Since we can turn off either the lidar or radar measurements, the *ProcessMeasurement()* method will return immediately when a skipped measurement is received. In this case, the current state will remain the same.

### Prediction
After initialization, a prediction is performed by *Predict()* method. It involves the following operations:

#### Creating Sigma Points
The *CreateSigmaPoints()* is called first to create 15 sigma points

#### Predict The New CTRV State Using the Sigma Points
After the sigma points are created, *PredictWithSigmaPoints()* method is invoked with the delta time to predict the current sigma points.

Then *ComputeMeanOfSigmaPoints()* method is invoked to compute the CTRV from the means of the predicted sigma points.

### Update
After the prediction is completed, the *Update()* method is invoked to update the prediction with the new sensor measurements. The method performs the following operations:

#### Predict The New Measurements using Sigma Points
The same sigma points computed during the prediction stage are used to predict the new measurement. This is done by invoking the *PredictMeasurementFromSigmPoints()* method.

For radar measurements, the method will convert the sigma points to the radar measurement space using *utils::PVToRadar()* function. Then it computes the means of the sigma points to obtain radar measurement prediction and the measurement covariance matrix (a 3x3 matrix). 

The *utils::PVToRadar()* function computes the bearing angle using *atan2(P<sub>x</sub> , P<sub>y</sub>)*, could we use CTRV's &#936; element directly? No, we couldn't, as CTRV's &#936; models the yaw angle, the direction the car is turning, namely *atan2(dP<sub>y</sub>, dP<sub>x</sub>)*. While the radar's bearing is the angle between the car and the radar, which is *atan2(P<sub>y</sub> , P<sub>x</sub>)*.

For lidar measurements, the method will uses the sigma points directly for the lidar measurement space, then it computes the means of the sigma points to obtain the lidar measurement prediction and the measurement covariance matrix (a 2x2 matrix).

#### Computing The Cross Corelation Matrix, the Kalman Gain
After the measurement prediction is made, the cross correlation matrix between the differences of the sigma points with respect to the predicted CTRV state and measurements is computed. Together with the measurement covariance matrix, we can compute the Kalman gain matrix.

#### Update the CTRV State and the State Covariance Matrix
Finally, the predicted CRTV state is updated using the Kalman gain, and the difference between the measurements and the predicted measurements.

Then, the state covariance matrix is updated using the Kalman gain matrix and the measurement covariance matrix.

### Angular correction for radar estimates
For correct estimate of angular movements, angles, and angular differences between predictions and measurements needs to be normalized to [ &#960;, -&#960; ). This is also applied to the &#968; element of the CRTV vector.

### Vectorization
Could the speed of the filter be improved by vectorization? To answer this question, I have created a git branch, vectorization, that contains a more vectorized version of the filter.
For example, the original code, and the vectorized version are as follows:

**The original code:**
```
void UKF::PredictWithSigmaPoints(const double dt) {
  Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);
  for (int i = 0; i < 2 * n_aug + 1; i++) {
    VectorXd col = Xsig_aug.col(i);
    if (fabs(col(4)) < EPSLION) { // avoid division by zero
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
}
```

**The vectorized version**
```
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
}
```

To build the vectorized version, the cmake file is modified to have build with -mavx -msse -mfma flags.

Unfortunately, the result of vectorization did not improve the performance, it actually caused 8-10% of performance drop! This has been observed with Bash on Windows (g++), and Mac (AppleClang).

It is still unclear to me why vectorization does not improve the performance. The possible reasons could include:

1. The data is not large enough to take the advantage of vectorization.
2. Extra data preparation required for the vectorization might have degraded the performance.

## RMSEEvaluator class
The RMSE evaluation implemented in the starter code is not efficient as it keeps all past measurements and estimates in a list, and evaluate RMSE for every new measurement on the entire list.

The purpose of this class is to implement a more efficient RMSE mechanism that does not need to store past measurements and estimates. It maintains current sum of the square errors, and total number of past estimates. RMSE for a new estimate can be simply obtained by taking the square root of the mean of square error.

## EKF class
It implements an Extended Kalman Filter refactored from the Extended Kalman Filter project. This is used for comparison purposes.

## Result
The RMSE performance of the Unscented Kalman filter is optimized when the acceleration deviation is at 0.5 and the yaw acceleration deviation is at 0.65. The following table shows the RMSE for running dataset 1 only, dataset 2 only, and dataset 1 followed by dataset 2:

|            | Dataset 1| Dataset 2| Dataset 1, 2|
|:----------:|:--------:|:--------:|:-----------:|
| X    		 |   0.0639 |   0.0629 |    0.0634   |
| Y          |   0.0858 |   0.0582 |    0.0733   |
| Vx    	 |   0.2288 |   0.3004 |    0.2670   |
| Vy         |   0.2963 |   0.2797 |    0.2882   |

The execution of the third scenario is recorded in [this video](video1.mp4).

### Effects of Acceleration Noise
The following table shows how the X, and Y RMSE of the Unscented Kalman filter varies with the process noise: the acceleration noise, and the yaw acceleration noise. 

|         |       0.05    |       0.5     |      0.65     |      1        |       2       |       4       | Std YawDD|
|:-------:|:-------------:|:-------------:|:-------------:|:-------------:|:-------------:|:-------------:|:--------:|
|**0.05** | 0.2147, 0.2222| 0.1069, 0.142 |  0.107, 0.1423| 0.1073, 0.1429| 0.1076, 0.1437| 0.1071, 0.1438|          | 
|**0.4**  | 0.2036, 0.1954| 0.0631, 0.0743| 0.0626, 0.0744| 0.0623, 0.0751| 0.0627, 0.0769| 0.0627, 0.0769|          | 
|**0.5**  | 0.2047, 0.1983| 0.0640, 0.0733|**0.0634, 0.0733**| 0.0631, 0.0739| 0.0634, 0.757 | 0.0640, 0.0778|          |
|**0.6**  | 0.2058, 0.2010|  0.065, 0.0729| 0.0643, 0.0729| 0.0640, 0.0735| 0.0642, 0.0752|  0.081, 0.0904|          |
|**8**    | 0.2215, 0.2316| 0.0811, 0.0865| 0.0808, 0.0862| 0.0814, 0.0872| 0.0821, 0.0893| 0.0832, 0.0929|          |
|**16**   | 0.2263, 0.2318| 0.0833, 0.0898|  0.083, 0.0896| 0.0834, 0.0903| 0.0851, 0.0933| 0.0872,  0.098|          |
|**Std A**|               |               |               |               |               |               |          |

It shows that the best RMSE performance is ontained when the acceleration deviation is around 0.5, and the yaw acceleration deviation is around 0.65.

### Normalized Innovation Squared (NIS) Evaluation
The NIS results is shown in the following table:

| Std A, Std Yawdd |       NIS Chart           |
|:----------------:|:-------------------------:|
| 0.05,   0.05     | [Chart](./nis-005-005.png)|
|  0.1,    0.1     | [Chart](./nis-01-01.png)  |
|  0.5,   0.65     | [Chart](./nis-05-065.png) |
|    8,      2     | [Chart](./nis-8-2.png)    |
|   16,      6     | [Chart](./nis-16-6.png)   |

The charts show when the noise level is reduced, NIS moves up and the variances also goes up, and when the noise level is increased, the NIS moves down.

### Comparison With Extended Kalman Filter
My experiments shows that Unscented Kalman filters out-performs Extended Kalman filters. This can be seen from the following table:

|            |   EKF    | %improved|
|:----------:|:--------:|:--------:|
| X    		 |   0.0817 |  +22%    |
| Y          |   0.0852 |  +14%    |
| Vx    	 |   0.3773 |  +29%    |
| Vy         |   0.4254 |  +32%    |

#### Sensors and Performance
The following table compares the X, and Y RMSE against different combinations of sensors. The **UKF performs best with both lidar and radar sensors**. On the other hand, the Unscented Kalman filter performs poorly with only one sensor.

|                 | Std A=0.5, Std YawDD=0.65 |
|:---------------:|:-------------------------:|
| Radar and Lidar | 0.0642, 0.0736            | 
| Radar only      | 0.198, 0.233              |
| Lidar only      | 0.351 0.357               |
