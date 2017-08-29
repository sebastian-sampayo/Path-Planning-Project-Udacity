#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <iostream>
#include <vector>
#include "json.hpp"

#include "sensor_data.h"

using namespace std;

// for convenience
using json = nlohmann::json;

class PathPlanner {
public:

  // Program Parameters
  static constexpr double LANE_WIDTH = 4;
  static constexpr int NUM_LANES = 3;
  static constexpr int SPEED_LIMITS[] = {10, 20, 30};

  // Inputs of the Path Planner
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;

  json previous_path_x;
  json previous_path_y;

  double end_path_s;
  double end_path_d;

  // The sensor_fusion variable contains all the information about the cars on the right-hand side of the road.
  // The data format for each car is: [ id, x, y, vx, vy, s, d]. The id is a unique identifier for that car. The x, y values are in global map coordinates, and the vx, vy values are the velocity components, also in reference to the global map. Finally s and d are the Frenet coordinates for that car.
  SensorData sensor_fusion;

  // Output of the Path Planner
  vector<double> next_x_vals;
  vector<double> next_y_vals;

  // Methods
  //! Process the inputs and generate the output trajectory
  void GeneratePath();

  // Test methods
  //! Generates a Straight Line
  void GenerateStraightLine();
  
  //! Generates a Circle path
  void GenerateCircle();

  // Constructor and destructor
  PathPlanner() {};
  virtual ~PathPlanner() {};
};

#endif