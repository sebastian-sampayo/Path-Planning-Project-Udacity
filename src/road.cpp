#include "road.h"
#include "vehicle.h"

#include "logger.h"
// #include "matplotlibcpp.h"
#include "utils.h"

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
Road::Road(double width, const vector<double>& lane_speeds)
{
  LOG(logDEBUG4) << "Road::Road(width, speeds)";

  LANE_WIDTH = width;
  this->lane_speeds = lane_speeds;
}

// ----------------------------------------------------------------------------
Road::~Road() {}

// ----------------------------------------------------------------------------
int Road::DToLane(double d) const
{
  return int(d / LANE_WIDTH);
}

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
vector<int> Road::GetVehiclesInSpace(const RoadSpace& space) const
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
bool Road::IsEgoColliding(double lenght_offset) const
{
  // This algorithm considers that each vehicle is an ellipse with major axis
  // equals to lenght/2 and minor axis equals LANE_WIDTH/2
  bool collision = false;
  
  for (auto& vehicle_pair : vehicles)
  {
    Vehicle vehicle = vehicle_pair.second;
    
    // Calculates the distance between the vehicle and the ego vehicle
    const double dist_x = vehicle.position.GetX() - ego.position.GetX();
    const double dist_y = vehicle.position.GetY() - ego.position.GetY();
    const double dist = Magnitude(dist_x, dist_y);
    
    // Calculates the angle of the distance vector
    const double delta_ego = atan2(dist_y, dist_x);
    const double theta_ego = delta_ego - ego.yaw;
    const double delta_vehicle = delta_ego + pi();
    const double theta_vehicle = delta_vehicle - vehicle.yaw;
    
    // Calculates border point of the ego vehicle using ellipse equation
    const double a = vehicle.lenght / 2.0 + lenght_offset; // = half_lenght
    // const double b = LANE_WIDTH / 2.0 * 0.9;
    const double b = vehicle.width / 2.0;
    const double d_ego = Magnitude(a * cos(theta_ego), b * sin(theta_ego));
    
    // Calculates border point of the other vehicle using ellipse equation
    const double d_vehicle = Magnitude(a * cos(theta_vehicle), b * sin(theta_vehicle));
    
    // Detect collision
    collision |= (dist < d_ego + d_vehicle);
    
    LOG(logDEBUG4) << "Road::IsEgoColliding() - Vehicles debug: \n"
        << "     id: " << vehicle_pair.first << " | dist: " << dist << endl
        << "  d_ego: " << d_ego << " | d_vehicle: " << d_vehicle << endl
        << "    ego: " << ego.position << endl
        << "vehicle: " << vehicle.position;
    
    if (collision) 
    {
      LOG(logDEBUG2) << "Road::IsEgoColliding() - Collision detected! Vehicle Id: " << vehicle_pair.first << endl 
        << "     id: " << vehicle_pair.first << " | dist: " << dist << endl
        << "  d_ego: " << d_ego << " | d_vehicle: " << d_vehicle << " | theta_ego: " << theta_ego << endl
        << "    ego: " << ego.position << endl
        << "vehicle: " << vehicle.position;
      break; // Collision detected, stop searching
    }
  }
  
  return collision;
}

// ----------------------------------------------------------------------------
bool Road::IsEmptySpace(const RoadSpace& space) const
{
  return GetVehiclesInSpace(space).empty();
}

// ----------------------------------------------------------------------------
bool Road::IsEgoRightSideEmpty() const
{
  // If the ego is in the right most lane, space is not empty
  if (ego.lane == 2) return false;
  
  RoadSpace space;
  
  // const double safe_distance = ego.lenght * 2;
  const double safe_time = 1;
  const double safe_distance = safe_time * ego.speed;
  space.s_down = ego.position.GetS() - safe_distance;
  space.s_up = space.s_down + safe_distance;
  space.d_right = (ego.lane + 1) * LANE_WIDTH;
  space.d_left = space.d_right + LANE_WIDTH;
  
  return IsEmptySpace(space);
}

// ----------------------------------------------------------------------------
bool Road::IsEgoLeftSideEmpty() const
{
  // If the ego is in the left most lane, space is not empty
  if (ego.lane == 0) return false;
  
  RoadSpace space;
  
  // const double safe_distance = ego.lenght * 2;
  const double safe_time = 1;
  const double safe_distance = safe_time * ego.speed;
  space.s_down = ego.position.GetS() - safe_distance;
  space.s_up = space.s_down + safe_distance;
  space.d_right = ego.lane * LANE_WIDTH;
  space.d_left = space.d_right - LANE_WIDTH;
  
  return IsEmptySpace(space);
}
// ----------------------------------------------------------------------------
void Road::LogVehicles(TLogLevel log_level) const
{
  LOG(log_level) << "Road::PrintVehicles() - Vehicles: ";
  
  LOG(log_level) << "ego: " << ego.position << " | yaw: " << ego.yaw << " | speed: " << ego.speed << "m/s";
  
  for (auto& vehicle_pair : vehicles)
  {
    const int id = vehicle_pair.first;
    Vehicle vehicle = vehicle_pair.second;
    
    LOG(log_level) << "id: " << id << " | " << vehicle.position 
      << " | yaw: " << vehicle.yaw << " | speed: " << vehicle.speed << "m/s";
  }
}

// ----------------------------------------------------------------------------
// void Road::PlotVehicles(const char* filename) const
// {
  // namespace plt = matplotlibcpp;
  
  // plt::figure();
  // for (auto& vehicle_pair : vehicles)
  // {
    // Vehicle vehicle = vehicle_pair.second;
    
    // vector<double> s, d;
    // s.push_back(vehicle.position.GetS());
    // d.push_back(vehicle.position.GetD());
    // plt::plot(d, s, "r*");
    // string str("id: ");
    // str += std::to_string(vehicle_pair.first);
    // plt::annotate(str.c_str(), d[0], s[0]);
  // }
  
  // vector<double> s, d;
  // s.push_back(ego.position.GetS());
  // d.push_back(ego.position.GetD());
  // plt::plot(d, s, "g*");
  // string str("ego");
  // plt::annotate(str.c_str(), d[0], s[0]);
  
  // plt::xlabel("d");
  // plt::ylabel("s");
  // plt::title("Vehicles on the road in Frenet coordinates");
  // plt::save(filename);
// }

// ----------------------------------------------------------------------------
void Road::PopulateTraffic(const EnvironmentSensorData& environment_data)
{
  // For each vehicle on the current road (this->vehicles), check if it is still in the environment.
  //   if it is, update it with the sensed vehicle data.
  //   else, erase it.
  // Then, for each sensed vehicle in the environment data, check if it is already in the current road (this->vehicles)
  //    if it is not, add it
  
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
  
  for (const auto& sensed_vehicle : environment_sensed_vehicles)
  {
    const int id = sensed_vehicle.first;
    const auto it = vehicles.find(id);
    const bool is_on_the_road = (it != vehicles.end()); // found
    
    if (!is_on_the_road)
    {
      LOG(logDEBUG3) << "Road::PopulateTraffic() - Add new vehicles in the environment onto the road";
      // Add it
      auto vehicle_pair = make_pair(id, Vehicle(sensed_vehicle.second, this));
      vehicles.insert(vehicle_pair);
    }
  }
}

// ----------------------------------------------------------------------------
Road Road::PredictRoadTraffic(double t) const
{
  Road future_road = *this;
  
  for (auto& vehicle_pair : future_road.vehicles)
  {
    Vehicle& vehicle = vehicle_pair.second;
    vehicle.Move(t);
  }
  
  return future_road;
}

// ----------------------------------------------------------------------------
void Road::UpdateEgoKinematics(const EgoSensorData& data)
{
  const double MPH2MPS = 0.44704;
  ego.position = PointCartesian(data.x, data.y);
  // ego.position.SetFrenet(data.s, data.d); // Ignore Frenet. I trust Cartesian more!
  ego.yaw = deg2rad(data.yaw); // Sensor data is in degrees, convert it to radians
  ego.speed = data.speed * MPH2MPS; // Sensor data is in MPH, convert it to m/s
  
  // TODO: Update ego lane
  // lane = { 0 if ego_d is in [0, LANE_WIDTH)
  //          1 if ego_d is in [LANE_WIDTH, LANE_WIDTH*2)
  //          2 if ego_d is in [LANE_WIDTH*2, LANE_WIDTH*3)
  const double ego_d = ego.position.GetD();
  ego.lane = DToLane(ego_d);
}
