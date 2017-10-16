#ifndef ROAD_H
#define ROAD_H

#include "sensor_data.h"
#include "vehicle.h"

#include "logger.h"

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

  // Default Constructor (Not implemented yet)
  Road();
  
  // Constructor with lanes: Builds the specified number of lanes at initialization.
  // @param width Width of each lane
  // @param lane_speeds Speed limit for each lane, starting from the right lane
  Road(double width, const vector<double>& lane_speeds);
  // Road(int speed_limit, double traffic_density, vector<int> lane_speeds);

  // Destructor
  virtual ~Road();
  
  //! Get the lane for the specified d-coordinate
  int DToLane(double d) const;
  
  //! Get the d-coordinate for the center of the specified lane
  double GetCenterDByLane(int lane) const;
  
  //! Get number of lanes
  int GetNumberOfLanes() const;
 
  //! Get a vector of ids of the vehicles that are currently in the specified space.
  vector<int> GetVehiclesInSpace(const RoadSpace& space) const;
  
  //! Check if the ego vehicle is colliding with other vehicle
  bool IsEgoColliding(double lenght_offset = 0) const;
  
  //! Check if the specified space on the road is empty (no vehicles)
  bool IsEmptySpace(const RoadSpace& space) const;
  
  //! Check if there is any vehicle in the right hand of the ego vehicle
  bool IsEgoRightSideEmpty() const;
  
  //! Check if there is any vehicle in the left hand of the ego vehicle
  bool IsEgoLeftSideEmpty() const;
  
  //! Print vehicles in stdout when debug log mode is active
  void LogVehicles(TLogLevel log_level = logDEBUG4) const;
  
  //! Plot vehicles and save to an image
  void PlotVehicles(const char* filename) const;
  
  //! Updates the current state of the traffic. If there is a new vehicle in the environmet data,
  // it is added to the vehicles array. If a vehicle in the new data was already in the vehicles array
  // it updates its state.
  void PopulateTraffic(const EnvironmentSensorData& environment_data);
  
  //! Predicts the state of the road in the future. (does not predicts ego motion, only traffic)
  Road PredictRoadTraffic(double t) const;
  
  void UpdateEgoKinematics(const EgoSensorData& ego_data);
};

#endif
