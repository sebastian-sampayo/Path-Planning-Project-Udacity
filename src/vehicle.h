#ifndef VEHICLE_H
#define VEHICLE_H

#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>

#include "kinematic_state.h"
#include "sensor_data.h"

using namespace std;

class Vehicle {
public:

  double lenght = 5;
  KinematicState kinematic_state;
  int lane;
  int lanes_available;
  double target_speed;
  int goal_lane;
  double goal_s;

  /**
  * Constructors
  */
  Vehicle();
  Vehicle(const EnvironmentSensorData::SensedVehicleData& data);
  Vehicle(int lane, double s, double v, double a);

  /**
  * Destructor
  */
  virtual ~Vehicle();
  
  // void predict(double deltaT);

};

#endif