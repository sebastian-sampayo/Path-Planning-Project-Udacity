#include <iostream>
#include <iostream>
#include <list>
#include <map>
#include <math.h>
#include <string>
#include <iterator>

#include "logger.h"
#include "road.h"
#include "vehicle.h"

using namespace std;

// ----------------------------------------------------------------------------
// PUBLIC
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Initializes Road
Road::Road(double width, vector<int> lane_speeds)
{
  LOG(logDEBUG4) << "Road::Road()";

  LANE_WIDTH = width;

  for (const int speed_limit : lane_speeds)
  {
    Lane lane;
    lane.speed_limit = speed_limit;
    lanes.push_back(lane);
  }
}

// ----------------------------------------------------------------------------
Road::~Road() {}

// ----------------------------------------------------------------------------
void Road::UpdateEgoKinematics(EgoSensorData data)
{
  ego.position = PointCartesian(data.x, data.y);
  // ego.position.SetFrenet(data.s, data.d); // Ignore Frenet. I trust Cartesian more!
  ego.yaw = data.yaw;
  ego.speed = data.speed;
}

// ----------------------------------------------------------------------------
void Road::PopulateTraffic(EnvironmentSensorData& environment_data)
{
  // For each vehicle on the current road (this->vehicles), check if it is still in the environment.
  //   if it is, update it with the sensed vehicle data.
  //   else, erase it.
  // Then, for each sensed vehicle in the environment data, check if it is already in the current road (this->vehicles)
  //    if it is not, add it
  
  LOG(logDEBUG3) << "Road::PopulateTraffic() - Fill a map with the environment data";
  // First fill a map with the environment vehicles in order to search easily.
  map<int, EnvironmentSensorData::SensedVehicleData> environment_sensed_vehicles;
  
  for (auto& sensed_vehicle : environment_data.sensed_vehicle_list)
  {
    auto sensed_vehicle_pair = make_pair(sensed_vehicle.id, sensed_vehicle);
    environment_sensed_vehicles.insert(sensed_vehicle_pair);
  }
  
  LOG(logDEBUG3) << "Road::PopulateTraffic() - Update/Erase vehicles on the current road";
  // First create a list with the vehicles that will be erased
  list<int> old_vehicles;
  
  // Update/Erase vehicles on the current road
  for (auto& vehicle_pair : vehicles)
  {
    const int id = vehicle_pair.first;
    Vehicle& vehicle = vehicle_pair.second;
    const auto it = environment_sensed_vehicles.find(id);
    const bool still_in_environment = (it != environment_sensed_vehicles.end()); // found
    
    if (still_in_environment)
    {
      // Update it
      vehicle.UpdateSensorData(it->second);
    }
    else
    {
      // Mark it for erase
      old_vehicles.push_back(id);
    }
  }
  
  // Erase old vehicles
  for (int id : old_vehicles)
  {
    vehicles.erase(id);
  }
  
  LOG(logDEBUG3) << "Road::PopulateTraffic() - Add new vehicles in the environment onto the road";
  for (const auto& sensed_vehicle : environment_sensed_vehicles)
  {
    const int id = sensed_vehicle.first;
    const auto it = vehicles.find(id);
    const bool is_on_the_road = (it != vehicles.end()); // found
    
    if (!is_on_the_road)
    {
      // Add it
      auto vehicle_pair = make_pair(id, Vehicle(sensed_vehicle.second, this));
      vehicles.insert(vehicle_pair);
    }
  }
}