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
  double width = 3; // [m]
  Point position;
  double yaw;
  double speed;
  int lane;
  int lanes_available;

  // The vehicle can see the road in which it is driving.
  Road* road_ptr;

  //! Constructors
  Vehicle();
  Vehicle(const EnvironmentSensorData::SensedVehicleData& data, Road* road);

  //! Destructor
  virtual ~Vehicle();
  
  //! Moves vehicle based on kinematic model (assuming constant speed and yaw)
  void Move(double delta_t);
  
  //! Predicts position of vehicle in delta_t seconds (assuming constant speed)
  Point PredictPosition(double delta_t) const;
  
  //! Translates the vehicle to the specified point. Updates yaw based on the new point position
  void Translate(const Point& new_pos);
  
  //! Updates sensor data from the environment
  void UpdateSensorData(const EnvironmentSensorData::SensedVehicleData& data);
};

#endif
