#ifndef VEHICLE_H
#define VEHICLE_H

#include "point.h"
#include "road.h"
#include "sensor_data.h"

#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>

using namespace std;

// Forward declaration to avoid circular dependency compile errors
class Road;

class Vehicle {
public:

  double lenght = 5; // [m]
  Point position;
  double yaw;
  double speed;
  int lane;
  int lanes_available;
  // double target_speed;
  // int goal_lane;
  // double goal_s;
  // The vehicle can see the road in which it is driving.
  Road* road_ptr;

  // Constructors
  Vehicle();
  Vehicle(const EnvironmentSensorData::SensedVehicleData& data, Road* road);

  // Destructor
  virtual ~Vehicle();
  
  void UpdateSensorData(const EnvironmentSensorData::SensedVehicleData& data);
  // void predict(double deltaT);

};

#endif