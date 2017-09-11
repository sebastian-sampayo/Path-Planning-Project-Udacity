#include <iostream>
#include <vector>

#include "logger.h"
#include "path_planner.h"
#include "road.h"
#include "utils.h"
#include "trajectory.h"
#include "vehicle.h"

using namespace std;

// Static attributes
constexpr int PathPlanner::SPEED_LIMITS[];

// ----------------------------------------------------------------------------
PathPlanner::PathPlanner()
{
  LOG(logDEBUG4) << "PathPlanner::PathPlanner()";
  // behavior = Behavior();
}

// ----------------------------------------------------------------------------
PathPlanner::~PathPlanner()
{
  LOG(logDEBUG4) << "PathPlanner::~PathPlanner()";
}

// ----------------------------------------------------------------------------
Trajectory PathPlanner::Generate()
{
  behavior.UpdateState(road);
  return behavior.GetTrajectory();
}

// void PathPlanner::GeneratePath()
// {
  // Vehicle ego();
  
  // // Convert from int[] to vector<int>
  // // TODO: encapsulate in a function
  // vector<int> speed_limits;
  // for (int i = 0; i < NUM_LANES; ++i)
  // {
    // speed_limits.push_back(SPEED_LIMITS[i]);
  // }
  // Road road(LANE_WIDTH, speed_limits);
  
// }

// ----------------------------------------------------------------------------
void PathPlanner::SetEgoData(EgoSensorData data)
{
  LOG(logDEBUG3) << "PathPlanner::SetEgoData() - data.s = " << data.s;
  LOG(logDEBUG3) << "PathPlanner::SetEgoData() - data.d = " << data.d;
  LOG(logDEBUG3) << "PathPlanner::SetEgoData() - data.x = " << data.x;
  LOG(logDEBUG3) << "PathPlanner::SetEgoData() - data.y = " << data.y;
  this->ego_data.x = data.x;
  this->ego_data.y = data.y;
  this->ego_data.s = data.s;
  this->ego_data.d = data.d;
  this->ego_data.speed = data.speed;
  this->ego_data.yaw = data.yaw;
  
  // Under the hood also set the start point of the trajectory with this data
  PointCartesian pc(data.x, data.y);
  // PointFrenet pf(data.s, data.d); // Let's ignore the Frenet data, because it is an untrusted conversion from the Cartesian data. It's better if I use my own conversion from Cartesian to Frenet.
  road.ego.position = Point(pc);
  road.ego.yaw = data.yaw;
  road.ego.speed = data.speed;
  
  LOG(logDEBUG3) << "PathPlanner::SetEgoData() - ego position = " << road.ego.position;
}

// ----------------------------------------------------------------------------
void SetEnvironmentData(const EnvironmentSensorData& data)
{
  road.PopulateTraffic(data);
}

// ----------------------------------------------------------------------------
void PathPlanner::SetPointsAlreadyPassed(int n)
{
  behavior.strategy->N_points_passed = n;
}

// ----------------------------------------------------------------------------
void PathPlanner::SetPreviousPath(json previous_path_x, json previous_path_y)
{
  behavior.strategy->previous_path.clear();

  int path_size = previous_path_x.size();
  for (int i = 0; i < path_size; ++i)
  {
    Point p;
    p.SetXY(previous_path_x[i], previous_path_y[i]);
    behavior.strategy->previous_path.push_back(p);
  }
}

// ----------------------------------------------------------------------------
void PathPlanner::SetPreviousEndPoint(double end_path_s, double end_path_d)
{
  behavior.strategy->previous_end_point.SetFrenet(end_path_s, end_path_d);
}