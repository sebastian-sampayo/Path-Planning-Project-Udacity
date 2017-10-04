#include "road.h"
#include "vehicle.h"

#include "logger.h"

#include <iostream>
#include <iostream>
#include <list>
#include <map>
#include <math.h>
#include <string>
#include <iterator>

using namespace std;

// ----------------------------------------------------------------------------
// PUBLIC
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
Road::Road()
{
  // Not implemented yet
  LOG(logERROR) << "Road::Road() - Not implemented yet!";
}

// ----------------------------------------------------------------------------
// Initializes Road
Road::Road(double width, vector<double> lane_speeds)
{
  LOG(logDEBUG4) << "Road::Road(width, speeds)";

  LANE_WIDTH = width;
  this->lane_speeds = lane_speeds;
}

// ----------------------------------------------------------------------------
Road::~Road() {}

// ----------------------------------------------------------------------------
double Road::GetCenterDByLane(int lane) const
{
  // Note: "lane" starts (is 0) at the left most lane on the road.
  // LANE_WIDTH/2.0 + double(lane) * (LANE_WIDTH)
  return LANE_WIDTH * (0.5 + double(lane));
}

// ----------------------------------------------------------------------------
int Road::GetNumberOfLanes() const
{
  return lane_speeds.size();
}

// ----------------------------------------------------------------------------
vector<int> Road::GetVehiclesInSpace(RoadSpace space) const
{
  vector<int> ids;
  
  for (auto& vehicle_pair : vehicles)
  {
    const int id = vehicle_pair.first;
    const Vehicle& vehicle = vehicle_pair.second;
    const double s = vehicle.position.GetS();
    const double d = vehicle.position.GetD();
    
    if (space.s_down < s && s < space.s_up && space.d_left < d && d < space.d_right)
    {
      ids.push_back(id);
    }
  }
  
  return ids;
}
// ----------------------------------------------------------------------------
bool Road::IsEmptySpace(RoadSpace space) const
{
  return GetVehiclesInSpace(space).empty();
}

// ----------------------------------------------------------------------------
void Road::PopulateTraffic(const EnvironmentSensorData& environment_data)
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

// ----------------------------------------------------------------------------
void Road::UpdateEgoKinematics(EgoSensorData& data)
{
  ego.position = PointCartesian(data.x, data.y);
  // ego.position.SetFrenet(data.s, data.d); // Ignore Frenet. I trust Cartesian more!
  ego.yaw = data.yaw;
  ego.speed = data.speed;
  
  // TODO: Update ego lane
  // lane = { 0 if ego_d is in [0, LANE_WIDTH)
  //          1 if ego_d is in [LANE_WIDTH, LANE_WIDTH*2)
  //          2 if ego_d is in [LANE_WIDTH*2, LANE_WIDTH*3)
  const double ego_d = ego.position.GetD();
  ego.lane = int(ego_d / LANE_WIDTH);
}
