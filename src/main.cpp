#include <math.h>
#include <uWS/uWS.h>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include "filter/UKF.h"
#include "json.hpp"
#include "filter/utils.h"
#include "performance/RMSEEvaluator.h"
#include "sensor/MeasurementPackage.h"
#include "sensor/SensorType.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("]");
  if (found_null != std::string::npos) {
    return "";
  } else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

// Timestamp of the previous measurement
static long long previous_timestamp = 0;

// Flag for turning NIS generation of/off
static bool generate_nis = false;

// nis output stream
static ofstream nis_out;

// Total number of NIS computed
static int nis_count = 0;

// NIS output frequency
static const int nis_interval = 10;

// Max and min of NIS value in output internal
static float nis_max = -1000000, nis_min = 1000000;

// NIS sum in interval
static float nis_sum = 0;

void exitHandler(void) {
  if (nis_out.is_open()) nis_out.close();
}

int main(int argc, char *argv[]) {
  uWS::Hub h;

  // Create a Kalman Filter instance
  UKF kalman;

  // RMSE evaluator
  RMSEEvaluator<Eigen::ArrayXd> evaluator;

  // NIS file name
  std::string nis_file = "nis.csv";

  // Flags for lidar and radar
  bool lidar = true, radar = true;

  // Standard deviation for acceleration and yaw change rate
  double std_a = 0.5, std_yawd = 0.65;

  // Process command line options
  for (int i = 1; i < argc; i++) {
    if (std::string((argv[i])) == "-nis") { // Set NIS file name
      nis_file = std::string(argv[++i]);
      generate_nis = true;
    } else if (std::string((argv[i])) == "-nolidar") { // Turn off lidar
      lidar = false;
      kalman.UseLidar(false);
    } else if (std::string((argv[i])) == "-nolaser") { // Turn off lidar
      lidar = false;
      kalman.UseLidar(false);
    } else if (std::string((argv[i])) == "-noradar") { // Turn off radar
      radar = false;
      kalman.UseRadar(false);
    } else if (std::string((argv[i])) == "-stda") { // set std_a
      if (sscanf(argv[++i], "%lf", &std_a) != 1) {
        std::cerr << "Invalid standard deviation for acceleration: " << argv[i] << std::endl;
        exit(-1);
      }
    } else if (std::string((argv[i])) == "-stdyawd") { // set std_yawd
      if (sscanf(argv[++i], "%lf", &std_yawd) != 1) {
        std::cerr << "Invalid standard deviation for yaw rate: " << argv[i] << std::endl;
        exit(-1);
      }
    } else {
      std::cerr << "Unknown option: " << argv[i] << std::endl;
      exit(-1);
    }
  }
  if (!(lidar || radar)) { // Need one of the sensor to be enabled
    std::cerr << "Both lidar and radar are disabled, exit now!" << std::endl;
    exit(-1);
  }

  kalman.StdA(std_a);
  kalman.StdYawD(std_yawd);

  // Print out options
  std::cout << "NIS: " << (generate_nis? ("On, file: " + nis_file): "Off") << std::endl;
  std::cout << "Radar: " << (radar? "On": "Off") << std::endl;
  std::cout << "Lidar: " << (lidar? "On": "Off") << std::endl;
  std::cout << "Standard deviation - acceleration: " << std_a << std::endl;
  std::cout << "Standard deviation - yaw rate: " << std_yawd << std::endl;

  // Open NIS file
  if (generate_nis) {
    nis_out.open(nis_file, ios::out | ios::trunc);
    atexit(exitHandler);
  }

  // Handle requests
  h.onMessage([&kalman, &evaluator](
      uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
      uWS::OpCode opCode) {
      // "42" at the start of the message means there's a websocket message
      // event.
      // The 4 signifies a websocket message
      // The 2 signifies a websocket event

      if (length && length > 2 && data[0] == '4' && data[1] == '2') {
        auto s = hasData(std::string(data));
        if (s != "") {
          auto j = json::parse(s);

          std::string event = j[0].get<std::string>();

          if (event == "telemetry") {
            // j[1] is the data JSON object

            string sensor_measurment = j[1]["sensor_measurement"];

            istringstream iss(sensor_measurment);
            long long timestamp;
            SensorType type;
            Eigen::VectorXd measurements;

            // reads first element from the current line
            string sensor_type;
            iss >> sensor_type;

            if (sensor_type.compare("L") == 0) {
              type = SensorType::LASER;
              measurements = VectorXd(2);
              float px;
              float py;
              iss >> px;
              iss >> py;
              measurements << px, py;
              iss >> timestamp;
              timestamp = timestamp;
            } else if (sensor_type.compare("R") == 0) {
              type = SensorType::RADAR;
              measurements = VectorXd(3);
              float ro;
              float theta;
              float ro_dot;
              iss >> ro;
              iss >> theta;
              iss >> ro_dot;
              measurements << ro, theta, ro_dot;
              iss >> timestamp;
              timestamp = timestamp;
            }

            // We want to be able to test different data set without restarting
            // the program
            // However, we will not reset the RMSE data so we could have more
            // samples for a
            // better RMSE
            if (timestamp < previous_timestamp ||
                timestamp - previous_timestamp > 1e8) {
              cout << "Restarting ..." << endl;
              kalman.Reset();
            } else {
              cout << "Process: " << timestamp
                   << ": dt = " << (timestamp - previous_timestamp) / 1000000.0
                   << ", " << (type == SensorType::RADAR ? "R,  " : "L,  ")
                   << measurements.transpose() << endl;
            }

            // Update the timestamp for restarting purpose
            previous_timestamp = timestamp;

            // Create the measurement package to pass to Kalman filter
            MeasurementPackage<SensorType> meas_package(timestamp, type,
                                                        measurements);
            float x_gt;
            float y_gt;
            float vx_gt;
            float vy_gt;
            iss >> x_gt;
            iss >> y_gt;
            iss >> vx_gt;
            iss >> vy_gt;
            VectorXd gt_values(4);
            gt_values(0) = x_gt;
            gt_values(1) = y_gt;
            gt_values(2) = vx_gt;
            gt_values(3) = vy_gt;

            // Call ProcessMeasurment(meas_package) for Kalman filter
            bool processed = kalman.ProcessMeasurement(meas_package);

            if (processed && generate_nis) {
              float nis_value = kalman.ComputeNIS();
              nis_sum += nis_value;
              if (nis_max < nis_value) {
                nis_max = nis_value;
              }
              if (nis_min > nis_value) {
                nis_min = nis_value;
              }
              if (++nis_count % nis_interval == 0) {
                nis_out << nis_count << "," << nis_min << "," << nis_max << "," << (nis_sum / nis_interval) << std::endl;
                nis_out.flush();
                nis_max = -1000000;
                nis_min = 1000000;
                nis_sum = 0.0;
              }
            }

            // Push the current Kalman filter's estimate to the RMSE evaluator
            VectorXd estimate = kalman.x();

            double p_x = estimate(0);
            double p_y = estimate(1);
            double v1 = estimate(2) * cos(estimate(3));
            double v2 = estimate(2) * sin(estimate(3));

            VectorXd est(4);
            est << p_x, p_y, v1, v2;

#ifdef VERBOSE_OUT
            std::cout << "UKF x = " << kalman.x().transpose() << std::endl;
            std::cout << "UKF P = " << kalman.P() << std::endl;
            std::cout << "Estimate = " << est << std::endl;
#endif
            evaluator.Add(est.array(), gt_values.array());

            Eigen::ArrayXd RMSE = evaluator.Evaluate();

            // Send RMSE back to the simulator
            json msgJson;
            msgJson["estimate_x"] = p_x;
            msgJson["estimate_y"] = p_y;
            msgJson["rmse_x"] = RMSE(0);
            msgJson["rmse_y"] = RMSE(1);
            msgJson["rmse_vx"] = RMSE(2);
            msgJson["rmse_vy"] = RMSE(3);
            auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";
            // std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            std::cout << "RMSE: " << RMSE.transpose() << std::endl;
          }
        } else {
          std::string msg = "42[\"manual\",{}]";
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
      const std::string s = "<h1>Hello world!</h1>";
      if (req.getUrl().valueLength == 1) {
        res->end(s.data(), s.length());
      } else {
        // i guess this should be done more gracefully?
        res->end(nullptr, 0);
      }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
      std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
      // ws.close(); // this causes segmentation fault (on Windows at least)

      std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
      std::cout << "Listening to port " << port << std::endl;
  } else {
      std::cerr << "Failed to listen to port" << std::endl;
      return -1;
  }
  h.run();
  }
