#include "logger.h"
#include "road.h"
#include "trajectory.h"
#include "trajectory_cost.h"
#include "utils.h"
#include "vehicle.h"

#include <map>
#include <iostream>

using namespace std;

// ----------------------------------------------------------------------------
// PUBLIC
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
TrajectoryCost::TrajectoryCost()
{
  // Not implemented yet!
  LOG(logERROR) << "TrajectoryCost::TrajectoryCost() - Not implemented yet!";
}

// ----------------------------------------------------------------------------
TrajectoryCost::TrajectoryCost(Road* road)
  : road_ptr(road)
{
  LOG(logDEBUG4) << "TrajectoryCost::TrajectoryCost(Road*)";
  
  // TODO: Move this to a configuration file

  // Cost Functions
  functions[CostFunctions::MAX_JERK] = &TrajectoryCost::MaxJerk;
  functions[CostFunctions::DETECT_COLLISION] = &TrajectoryCost::DetectCollision;
  
  // Weights
  weights[CostFunctions::MAX_JERK] = 1;
  weights[CostFunctions::DETECT_COLLISION] = 1;
}

// ----------------------------------------------------------------------------
TrajectoryCost::~TrajectoryCost()
{
}

// ----------------------------------------------------------------------------
double TrajectoryCost::CalculateCost(Trajectory trajectory)
{
  this->trajectory = trajectory;

  double cost = 0;
  
  for (auto it : functions)
  {
    double weight = weights[it.first]; // TODO: use find and check for valid key
    cost += weight * (this->*(it.second))();
  }

  return cost;
}

// ----------------------------------------------------------------------------
double TrajectoryCost::MaxJerk()
{
  return 0;
}

// ----------------------------------------------------------------------------
double TrajectoryCost::DetectCollision()
{
  Road& road = *road_ptr; // alias
  
  bool collision = false;
  double cost = 0;
  
  const double T_simulator = 0.02; // TODO: Move to a configuration file
  double collision_margin_percentage = 0.1;
  
  // For each point in the trajectory 
  //    For each vehicle on the road
  //        calculates the distance between the point and the vehicle
  //        if distance < 2 VEHICLE_RADIUS => collision!
  double t = 0; // time
  
  for (Point point : trajectory)
  {
    double x = point.GetX();
    double y = point.GetY();
    
    for (auto& vehicle_pair : road.vehicles)
    {
      Vehicle vehicle = vehicle_pair.second;
      
      // Predict the vehicle position in time t
      Point future_pos = vehicle.PredictPosition(t);
      double vehicle_x = future_pos.GetX();
      double vehicle_y = future_pos.GetY();
      
      double dist = distance(x, y, vehicle_x, vehicle_y);
      
      collision |= (dist < vehicle.lenght * collision_margin_percentage);
      
      if (collision) 
      {
        LOG(logDEBUG3) << "TrajectoryCost::DetectCollision() - Collision detected! Vehicle Id: " << vehicle_pair.first;
        break; // Collision detected, stop searching
      }
    }
    if (collision) break; // Collision detected, stop searching
    
    t += T_simulator;
  }
  
  cost = (collision ? 1 : 0);
  
  return cost;
}