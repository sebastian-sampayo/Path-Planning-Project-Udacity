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
  SET_LOG_LEVEL(logDEBUG3);
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
  for (auto& vehicle_pair : road.vehicles)
  {
    int id = vehicle_pair.first;
    Vehicle& vehicle = vehicle_pair.second;
    
    double x = vehicle.position.GetX();
    double y = vehicle.position.GetY();
    
    cout << "id: " << id << " | x: " << x << " | y: " << y << endl;
  }
  
  // Second chunk of sensor data
  EnvironmentSensorData environment_data2;
  
  // Convert Sensor fusion data from json to SensorData class:
  LOG(logINFO) << "Get environment sensor data 2";
  for (const auto& sensed_vehicle : stub_environment_data2)
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
  for (auto& vehicle_pair : road.vehicles)
  {
    int id = vehicle_pair.first;
    Vehicle& vehicle = vehicle_pair.second;
    
    double x = vehicle.position.GetX();
    double y = vehicle.position.GetY();
    
    cout << "id: " << id << " | x: " << x << " | y: " << y << endl;
  }
  
  return 0;
}