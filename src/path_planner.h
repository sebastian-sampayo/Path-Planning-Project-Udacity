#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <iostream>
#include <vector>
#include "json.hpp"

#include "behavior.h"
#include "road.h"
#include "sensor_data.h"
#include "trajectory.h"
#include "trajectory_strategy.h"


// for convenience
using namespace std;
using json = nlohmann::json;

class PathPlanner {
public:

  // Program Parameters
  double LANE_WIDTH = 4;
  int NUM_LANES = 3;
  int SPEED_LIMITS[3] = {49, 49, 49}; // [mph]

  // The sensor_fusion variable contains all the information about the cars on the right-hand side of the road.
  // The data format for each car is: [ id, x, y, vx, vy, s, d]. The id is a unique identifier for that car. The x, y values are in global map coordinates, and the vx, vy values are the velocity components, also in reference to the global map. Finally s and d are the Frenet coordinates for that car.
  // EnvironmentSensorData environment_data;
  
  Road road;
  
  Behavior behavior;

  // Output of the Path Planner
  // Trajectory next_path;

  // Constructor and destructor
  PathPlanner();
  virtual ~PathPlanner();

  // Methods
  //! Process the inputs and generate the output trajectory
  Trajectory Generate();

  // // Test methods
  // //! Generates a Straight Line
  // void GenerateStraightLine();

  // //! Generates a Circle path
  // void GenerateCircle();

  void SetEgoData(const EgoSensorData& data);
  void SetEnvironmentData(const EnvironmentSensorData& data);
  void SetPointsAlreadyPassed(int n);
  void SetPreviousPath(const json& previous_path_x, const json& previous_path_y);
  void SetPreviousEndPoint(double end_path_s, double end_path_d);
};

#endif
