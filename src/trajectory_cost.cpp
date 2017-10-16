#include "map.h"
#include "road.h"
#include "trajectory.h"
#include "trajectory_cost.h"
#include "vehicle.h"

#include "logger.h"
#include "utils.h"

#include <iostream>
#include <map>

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
  // functions[CostFunctions::CENTERED] = &TrajectoryCost::Centered;
  functions[CostFunctions::DETECT_COLLISION] = &TrajectoryCost::DetectCollision;
  functions[CostFunctions::EMPTY_SPACE] = &TrajectoryCost::EmptySpace;
  functions[CostFunctions::LANE_PREFERENCE] = &TrajectoryCost::LanePreference;
  functions[CostFunctions::MAX_ACCEL] = &TrajectoryCost::MaxAcceleration;
  functions[CostFunctions::MAX_JERK] = &TrajectoryCost::MaxJerk;
  functions[CostFunctions::MAX_SPEED] = &TrajectoryCost::MaxSpeed;
  functions[CostFunctions::SPEED] = &TrajectoryCost::Speed;
  
  // Weights
  // weights[CostFunctions::CENTERED] = .5;
  weights[CostFunctions::DETECT_COLLISION] = 1000;
  weights[CostFunctions::EMPTY_SPACE] = 1;
  weights[CostFunctions::LANE_PREFERENCE] = 0.001;
  weights[CostFunctions::MAX_ACCEL] = 1;
  weights[CostFunctions::MAX_JERK] = 1;
  weights[CostFunctions::MAX_SPEED] = 10;
  weights[CostFunctions::SPEED] = 0.5;
}

// ----------------------------------------------------------------------------
TrajectoryCost::~TrajectoryCost()
{
}

// ----------------------------------------------------------------------------
double TrajectoryCost::CalculateCost(const Trajectory& trajectory)
{
  Timer timer;
  
  this->trajectory = trajectory;

  double cost = 0;
  
  for (auto it : functions)
  {
    double weight = weights[it.first]; // TODO: use find and check for valid key
    cost += weight * (this->*(it.second))();
    if (cost >= MAX_COST) break;
  }
  
  const double elapsed_time = timer.GetElapsedMiliSeconds();
  LOG(logDEBUG3) << "TrajectoryCost::CalculateCost - elapsed_time: " << elapsed_time << "ms";

  return cost;
}

// ----------------------------------------------------------------------------
double TrajectoryCost::Centered()
{
  Road& road = *road_ptr; // alias
  
  double cost;
  double t = 0; // time
  const double T_simulator = 0.02; // TODO: Move to a configuration file
  
  for (const Point& point : trajectory)
  {
    const double d = point.GetD();
    const double lane = road.DToLane(d);
    const double d_lane_centered = road.GetCenterDByLane(lane);
    const double offset = d - d_lane_centered;
    cost += fabs(offset) * T_simulator;
  }
  
  LOG(logDEBUG3) << "TrajectoryCost::Centered() - Cost = Avg Offset: " << cost;
  
  return cost;
}

// ----------------------------------------------------------------------------
double TrajectoryCost::DetectCollision()
{
  Road& road = *road_ptr; // alias
  
  bool collision = false;
  double cost = 0;
  const double lenght_offset = road.ego.lenght/8.0;
  
  const double T_simulator = 0.02; // TODO: Move to a configuration file
  double t = 0; // time
  
  for (const Point& point : trajectory)
  {  
    // Predict the vehicles position in time t
    Road future_road = road.PredictRoadTraffic(t);
    future_road.ego.Translate(point);
    
    collision = future_road.IsEgoColliding(lenght_offset);
  
    if (collision) 
    {
      LOG(logDEBUG2) << "TrajectoryCost::DetectCollision() - Future collision detected! t: " << t;
      break; // Collision detected, stop searching
    }
    
    t += T_simulator;
  }
  
  // Binary output
  const double collision_cost = (collision ? 1 : 0);
  
  // Penalize the time of collision as well. If the collision is further away may be we can avoid it in a future step.
  // const double max_T = trajectory.size() * T_simulator;
  // const double time_cost = 1 - t/max_T;
  
  // cost = 0.7 * collision_cost + 0.3 * time_cost;
  
  cost = collision_cost;
  
  return cost;
}

// ----------------------------------------------------------------------------
double TrajectoryCost::EmptySpace()
{
  // BUG: We should compare the end point with the future state of the road, not with the current snapshot of the road.
  // Assuming that the time elapsed between each point is constant = T_simulator,
  // then the time at end_point will be trajectory.size()*T_simulator
  const double MAX_DISTANCE = 100;
  const double T_simulator = 0.02; // TODO: Move this constant to a common configuration file
  
  Road& current_road = *road_ptr; // alias
  Road future_road = current_road.PredictRoadTraffic(trajectory.size()*T_simulator);
  
  Point end_point = trajectory.back();
  int lane = future_road.DToLane(end_point.GetD());
  
  LOG(logDEBUG3) << "TrajectoryCost::EmptySpace() - trajectory end_point: \n" << end_point;
  
  RoadSpace space_ahead;
  space_ahead.s_down = end_point.GetS();
  space_ahead.s_up = space_ahead.s_down + MAX_DISTANCE;
  space_ahead.d_left = future_road.GetCenterDByLane(lane) - future_road.LANE_WIDTH/2.0;
  space_ahead.d_right = space_ahead.d_left + future_road.LANE_WIDTH;
  
  vector<int> ids = future_road.GetVehiclesInSpace(space_ahead);
  
  double min_dist = MAX_DISTANCE;
  for (int id : ids)
  {
    LOG(logDEBUG3) << "TrajectoryCost::EmptySpace() - Vehicle ahead id: " << id << endl
      << end_point << endl
      << future_road.vehicles[id].position << endl;
      
    double vehicle_x = future_road.vehicles[id].position.GetX();
    double vehicle_y = future_road.vehicles[id].position.GetY();
    double dist = distance(end_point.GetX(), end_point.GetY(), vehicle_x, vehicle_y);
    
    if (dist < min_dist)
    {
      min_dist = dist;
    }
  }
  
  // double cost = max(0.0, 1 - min_dist / MAX_DISTANCE);
  double cost = exp(-min_dist/10);
  
  LOG(logDEBUG3) << "TrajectoryCost::EmptySpace() - Cost = " << cost << " | min_dist: " << min_dist;
  
  return cost;
}

// ----------------------------------------------------------------------------
double TrajectoryCost::LanePreference()
{
  Road& road = *road_ptr; // alias
  
  const Point& last = trajectory.back();
  const int lane = road.DToLane(last.GetD());
  const double cost = (lane == 1 ? 0 : 1);
    
  return cost;
}

// ----------------------------------------------------------------------------
double TrajectoryCost::MaxAcceleration()
{
  const double T_simulator = 0.02;
  
  // Get the derivative of the trajectory
  Trajectory speed_trajectory = trajectory.GetDerivative(T_simulator);
  Trajectory accel_trajectory = speed_trajectory.GetDerivative(T_simulator);
  
  double cost = 0;
  int t = 0;
  
  for (const Point& p : accel_trajectory)
  {
    
    if (p.GetS() > MAX_ACCEL || p.GetD() > MAX_ACCEL)
    {
      LOG(logDEBUG3) << "TrajectoryCost::MaxAcceleration() - Too much acceleration! | accel_s = " << p.GetS() << "m/s^2 | accel_d = " << p.GetD() << "m/s^2 | t = " << t;
      const double accel = Magnitude(p.GetX(), p.GetY());
      cost = Logistic(accel/MAX_ACCEL);
      break;
    }
    
    t += T_simulator;
  }
  
  return cost;
}

// ----------------------------------------------------------------------------
double TrajectoryCost::MaxJerk()
{
  const double T_simulator = 0.02;
  
  // Get the derivative of the trajectory
  Trajectory speed_trajectory = trajectory.GetDerivative(T_simulator);
  Trajectory accel_trajectory = speed_trajectory.GetDerivative(T_simulator);
  Trajectory jerk_trajectory = accel_trajectory.GetDerivative(T_simulator);
  
  double cost = 0;
  int t = 0;
  
  for (const Point& p : jerk_trajectory)
  {
    
    if (p.GetS() > MAX_JERK || p.GetD() > MAX_JERK)
    {
      LOG(logDEBUG3) << "TrajectoryCost::MaxJerk() - Too much jerk! | jerk_s = " << p.GetS() << "m/s^3 | jerk_d = " << p.GetD() << "m/s^3 | t = " << t;
      const double jerk = Magnitude(p.GetX(), p.GetY());
      cost = Logistic(jerk/MAX_JERK);
      break;
    }
    
    t += T_simulator;
  }
  
  return cost;
}

// ----------------------------------------------------------------------------
double TrajectoryCost::MaxSpeed()
{
  const double T_simulator = 0.02;
  
  // Get the derivative of the trajectory
  Trajectory speed_trajectory = trajectory.GetDerivative(T_simulator);
  
  double cost = 0;
  int t = 0;
  
  for (const Point& p : speed_trajectory)
  {
    const double speed = Magnitude(p.GetX(), p.GetY());
    
    if (speed > MAX_SPEED || p.GetS() > MAX_SPEED || p.GetD() > MAX_SPEED)
    {
      cost = Logistic(speed/MAX_SPEED);
      LOG(logDEBUG2) << "TrajectoryCost::MaxSpeed() - Too much speed! speed: " << speed << " | speed_s = " << p.GetS() << "m/s^2 | speed_d = " << p.GetD() << "m/s^2 | t = " << t << " | Cost: " << cost;
      break;
    }
    
    t += T_simulator;
  }
  
  return cost;
}

// ----------------------------------------------------------------------------
double TrajectoryCost::Speed()
{
  const double typical_distance = 60;
  Point first = trajectory.front();
  Point last = trajectory.back();
  
  double s_distance = last.GetS() - first.GetS();
  
  if (s_distance < 0)
  {
    s_distance += Map::GetInstance().MAX_S;
  }
  
  double cost = InvLogistic(s_distance / typical_distance);
  
  LOG(logDEBUG3) << "TrajectoryCost::Speed() - s_distance: "  << s_distance << " | Cost: " << cost;
  
  return cost;
}

// ----------------------------------------------------------------------------
double TrajectoryCost::Logistic(double x)
{
  return (2.0 / (1 + exp(-x)) - 1.0);
}

// ----------------------------------------------------------------------------
double TrajectoryCost::InvLogistic(double x)
{
  return exp(-x);
}
