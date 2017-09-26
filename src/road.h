#ifndef ROAD_H
#define ROAD_H

#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <set>
#include <map>
#include <string>
#include <iterator>

#include "sensor_data.h"
#include "vehicle.h"

using namespace std;

struct RoadSpace {
  double s_down;
  double s_up;
  double d_left;
  double d_right;
};

class Road {
public:
  double LANE_WIDTH = 4;
  vector<double> lane_speeds;
  map<int, Vehicle> vehicles;
  Vehicle ego;

  // Default Constructor
  Road() {};
  
  // Constructor with lanes: Builds the specified number of lanes at initialization.
  // @param width Width of each lane
  // @param lane_speeds Speed limit for each lane, starting from the right lane
  Road(double width, vector<double> lane_speeds);
  // Road(int speed_limit, double traffic_density, vector<int> lane_speeds);

  // Destructor
  virtual ~Road();
  
  //! Get number of lanes
  int GetNumberOfLanes() const;
 
  //! Get a vector of ids of the vehicles that are currently in the specified space.
  vector<int> GetVehiclesInSpace(RoadSpace space) const;
  
  //! Check if the specified space on the road is empty (no vehicles)
  bool IsEmptySpace(RoadSpace space) const;
  
  //! Updates the current state of the traffic. If there is a new vehicle in the environmet data,
  // it is added to the vehicles array. If a vehicle in the new data was already in the vehicles array
  // it updates its state.
  void PopulateTraffic(const EnvironmentSensorData& environment_data);
  
  void UpdateEgoKinematics(EgoSensorData& ego_data);
};

#endif
