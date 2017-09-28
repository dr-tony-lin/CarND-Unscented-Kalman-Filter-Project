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
* filter/KalmanFilter.h: define Kalman filter abstraction
* filter/utils.h, filter/utils.cpp]: contains some utility functions used by UKF
* performance/RMSEEvaluator.h: A template class for memory-efficient evaluation of RMSE
* sensor/SensorType.h: defines SensorType enum
* sensor/MeasurementPackage.h defines sensor measurement package structure

### Usage
To start the program:

    ./UKF [-stda number] [-stdyawd number] [-nolidar|-nolaser] [-noradar] [-nis filename]

It takes the following command line options:

* -stda: specifies the acceleration noise, e.g. -stda 0.5, the default is 0.5
* -stdyawd: specifies the yaw derivative noise (in &#960;), e.g. -stdyawd 0.65, the default is 0.65
* -nolidar, -nolaser: turns off laser measurements
* -noradar: turns off radar signals
* -nis: turns on NIS (Normalized Innovation Squared) and specifies the file to store NIS. NIS is stored with comma separated values. Each NIS record contains the sequence, the min, the max, and the mean in the corresponding period of 10 estimates. 

The program will listen on port 4567 for an incoming simulator connection. Only one simulator should be connected at anytime, though the program does not prohibit it. To start a new simulator, terminate the existing one first, then start a new one.

To start the simulator:

#### On Windows

    term2_sim9.exe path_to_obj_pose-laser-radar-synthetic-input.txt

#### Other platforms:

    ./term2_sim9 path_to_obj_pose-laser-radar-synthetic-input.txt

#### Run multiple simulations
*UKF* allows one to run multiple simulations, whether from the same simulator or different simulators, one at a time without having to restart it. This allows the RMSE to be evaluated for a much bigger set of estimates.

#### Build UKF
For Windows, Bash on Ubuntu on Windows should be used. Please follow the *Tips for Setting Up Your Environment in Extended Kalman Filter project"  on how to setup and configure the build environment.

To build UKF, invoke the following commands on the bash terminal:
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
. filter: contains KalmanFilter.h, UKF.[h,cpp], and utils.[h,cpp] files which implement the Unscented Kalman Filter.
. performance: contains RMSEEvaluator.h which provides RMSE evaluation functionality.
. sensor: contains source code for defining sensor measurement.

##### File names
In this project, I made a class to have its own .h, and .cpp files. More specifically, a class source file contains exactly one class, and has the same name as the class it contains. On the other hand, 

#### The libs folder
The libs folder contains Eigen and json.hpp required by the program.

### KalmanFilter class
This class define the baseline Kalman filter functionalities including the ProcessMeasurement, Predict, Update functions.

It's data members include the state, and state transition matrix, the measurement matrix, and the covariance matrices. Getter and sett. [KalmanFilter](api/html/classKalmanFilter.html).

### UKF class
This class implement the unscented Kalman filter functionalities. It extends KalmanFilter class and implements *ProcessMeasurement()*, *Predict()*, *Update()* methods.

The API documentation for UKF can be found [here](api/html/classUKF.html).

#### Angular correction for radar estimates

The radar measurement's bearing angle is expected to lie between [ &#960;, -&#960; ). This has to be enforced for radar measurements even though they are expected to fall into this range, but some exceptions are seen in the supplied data file for this project.

In addition to the bearing angle, the angular difference between predictions and measurements needs to aligned to be within this range which is also the minimum angle to move from one angle to the other.

Failing to align the angles correctly will severely degrade the accuracy of the estimation.

### The Unscented Kalman Filter Process
The process is implemented in method *ProcessMeasurement()*.

#### Initialization
The UKF is initialized upon receiving the first measurement. 

##### Initialization of CTRV
If the first measurement is from radar. The state is computed from the radar's measurement using *utils::RadarToCTRV()* function.

If the first measurement is from lidar, the measured position is used to populate the CTRV state: P<sub>x</sub> , P<sub>y</sub> , and &#936; 

##### Initialization of State Covariance
The covariance is initialized as a 5x5 identity matrix.

The *ProcessMeasurement()* method returns immediately after the initialized is completed.

##### Skipping Measurements
Since one can turn off either the lidar or radar measurements, the *ProcessMeasurement()* method will return immediately when a skipped measurement is received.

#### Prediction
After initialization, a prediction is performed by *Predict()* method.

##### Creating Sigma Points
The *CreateSigmaPoints()* is called first to create 15 sigma points

##### Predict The New CTRV State Using the Sigma Points
After the sigma points are created, *PredictWithSigmaPoints()* method is invoked with the delta time to predict the current sigma points.

Then *ComputeMeanOfSigmaPoints()* method is invoked to compute the CTRV from the means of the predicted sigma points.

#### Update
After the prediction is completed, the *Update()* method is invoked to update the prediction with the new sensor measurements.

##### Predict The New Measurements using Sigma Points
The same sigma points computed during the prediction stage are used to predict the new measurement. This is done by invoking the *PredictMeasurementFromSigmPoints()* method.

For radar measurements, it will convert the sigma points to the radar measurements first by using *utils::PVToRadar()* method, then compute the means of the sigma points to obtain the radar prediction and the measurement covariance matrix (a 3x3 matrix). It however, does not use *utils::CRTVToRadar()* method which could save a arctangent operation, as this tends resulting in bas results.

For lidar measurements, it will uses the sigma points directly for the lidar measurements, then it computes the means of the sigma points to obtain the lidar prediction and the measurement covariance matrix (a 2x2 matrix). It however, does not use *utils::CRTVToRadar()* method which could save a arctangent operation, as this tends resulting in bas results.

##### Computing The Cross Corelation Matrix, the Kalman Gain

#### Update the CTRV State and the State Covariance Matrix

### RMSEEvaluator class
The RMSE evaluation implemented in the starter code is not efficient as it keeps all past measurements and estimates in a list, and evaluate RMSE for every new measurement on the entire list.

The purpose of this class is to implement a more efficient RMSE mechanism that does not need to store past measurements and estimates. It maintains current sum of the square errors, and total number of past estimates. RMSE for a new estimate can be simply obtained by taking the square root of the mean of square error.

## Result
The RMSE performance of the implementation is averaged as follows after running dataset 1 followed by dataset 2:

|            |   RMSE   |
|:----------:|:--------:|
| X    		 |   0.0642 |
| Y          |   0.0736 |
| Vx    	 |   0.333  |
| Vy         |   0.298  |

Refer to [this video](video1.mp4).

### Discussion and Extra Experiments
The Kalman filter produces estimates effectively by dealing with uncertainity due to sensor noise and random external factors using a weighted average between current state estimation and the new measurement. The weight are derived from the covariance matrices. Intuitively, when the state covariance is smaller than the measurement covariance, the predicted state is trusted more, and will have a bigger weight, and when the state covariance is larger than the measurement covariance, the predicted is trusted less, and will have a smaller weight.

Despite the assumptions made for UKF, will changing condifence on prediction improves the RMSE performance? To answer this question, I run through a set of experiments that use different level of acceleration noise.  

|            |       0.05      |       0.5      |      0.65     |      1         |       2        |       4        |   Std YawDD  |
|:----------:|:--------------:|:--------------:|:-------------:|:--------------:|:--------------:|:--------------:|
| 0.4    	 |                | 0.0639, 0.0745 | 0.0634, 0.0747|                |                |                |              | 
| 0.5        | 0.0648, 0.0735 | 0.0642, 0.0736 | 0.0639, 0.0742| 0.0642, 0.0759 | 0.0648, 0.78   |               |
| 0.6        | 0.0657, 0.0732 | 0.0657, 0.0732 |               |                |                |               |
| 8          |                | 0.0813, 0.0864 | 0.0817, 0.087 | 0.0827, 0.0894 | 0.0837, 0.0931 |               |
| 16         |                | 0.0835, 0.0898 | 0.84, 0.0905  | 0.0857, 0.0935 | 0.0877, 0.0981 |               |
| Std A      |                |                |               |                |                |               |

Acceleration deviation around 0.5, and yaw change rate deviation around 0.65 yields the best RMSE perfoamance. The RMSE perform decreases slowly when increasing the noise. 

The following table compares the RMSE (px, and py) against different combinations of sensors. The UKF performs best with both lidar and radar, and wrost with only lidar.

|                 | Std A=0.5, Std YawDD=0.65 |
|:---------------:|:-------------------------:|
| Radar and Lidar | 0.0642, 0.0736            | 
| Radar only      | 0.197, 0.236              |
| Lidar only      | 0.376 0.359               |

