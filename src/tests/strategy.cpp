#include <iostream>
#include <thread>
#include <vector>
#include <fstream>

#include "../Eigen-3.3/Eigen/Core"
#include "../Eigen-3.3/Eigen/QR"
#include "../json.hpp"

#include "../logger.h"
#include "../map.h"
#include "../path_planner.h"
#include "../trajectory.h"
#include "../utils.h"

#include "stub.h"

// for convenience
using namespace std;
using json = nlohmann::json;

int main()
{
  SET_LOG_LEVEL(logALL);
  LOG(logINFO) << "----- Test Strategy -----";
  
  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  
  PathPlanner path_planner;
  
  LOG(logINFO) << "Get ego vehicle sensor data";
  path_planner.ego_data.x = stub_ego_data["x"];
  path_planner.ego_data.y = stub_ego_data["y"];
  path_planner.ego_data.s = stub_ego_data["s"];
  path_planner.ego_data.d = stub_ego_data["d"];
  path_planner.ego_data.speed = stub_ego_data["speed"];
  path_planner.ego_data.yaw = stub_ego_data["yaw"];
  
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
    path_planner.environment_data.sensed_vehicle_list.push_back(data);
  }

  LOG(logINFO) << "Generate next path";
  Trajectory next_path = path_planner.Generate();

  vector<double> next_x_values;
  vector<double> next_y_values;

  LOG(logINFO) << "Setting next_values ";
  next_x_values = next_path.GetXvalues();
  next_y_values = next_path.GetYvalues();
  
  LOG(logDEBUG2) << "next_path: " << endl << next_path << endl;

  return 0;
}