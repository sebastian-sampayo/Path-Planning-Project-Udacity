#include <iostream>
#include <vector>

#include "../Eigen-3.3/Eigen/Core"
#include "../Eigen-3.3/Eigen/QR"
#include "../json.hpp"

#include "../logger.h"
#include "../sensor_data.h"
#include "../road.h"

#include "stub.h"

// for convenience
using namespace std;
using json = nlohmann::json;

int main()
{
  SET_LOG_LEVEL(logDEBUG4);
  LOG(logINFO) << "----- Test Road -----";
  
  Road road;
  EnvironmentSensorData environment_data;

  // Convert Sensor fusion data from json to SensorData class:
  LOG(logINFO) << "Get environment sensor data";
  for (const auto& sensed_vehicle : stub_environment_data)
  {
    EnvironmentSensorData::SensedVehicleData data;
    data.id = sensed_vehicle[0];
    data.x = sensed_vehicle[1];
    data.y= sensed_vehicle[2];
    data.vx = sensed_vehicle[3];
    data.vy = sensed_vehicle[4];
    data.s = sensed_vehicle[5];
    data.d = sensed_vehicle[6];
    environment_data.sensed_vehicle_list.push_back(data);
  }
  
  LOG(logINFO) << "Populate Traffic";
  road.PopulateTraffic(environment_data);
  
  // Print
  LOG(logINFO) << "Print Traffic";
  road.LogVehicles();
  
  // Second chunk of sensor data
  EnvironmentSensorData environment_data2;
  
  // Convert Sensor fusion data from json to SensorData class:
  LOG(logINFO) << "Get environment sensor data 2";
  for (const auto& sensed_vehicle : stub_front_vehicle)
  {
    EnvironmentSensorData::SensedVehicleData data;
    data.id = sensed_vehicle[0];
    data.x = sensed_vehicle[1];
    data.y= sensed_vehicle[2];
    data.vx = sensed_vehicle[3];
    data.vy = sensed_vehicle[4];
    data.s = sensed_vehicle[5];
    data.d = sensed_vehicle[6];
    environment_data2.sensed_vehicle_list.push_back(data);
  }
  
  LOG(logINFO) << "Populate Traffic";
  road.PopulateTraffic(environment_data2);
  
  // Print
  LOG(logINFO) << "Print Traffic";
  road.LogVehicles();
  
  // Predicts future road
  Road future_road = road.PredictRoadTraffic(5);
  future_road.ego.Translate(PointFrenet(69, 1));
  future_road.LogVehicles();
  
  cout << "road.IsEgoColliding(): " << road.IsEgoColliding() << endl;
  cout << "future_road.IsEgoColliding(): " << future_road.IsEgoColliding() << endl;
  
  road.PlotVehicles("../img/road_plot.png");
  future_road.PlotVehicles("../img/future_road_plot.png");
  return 0;
}