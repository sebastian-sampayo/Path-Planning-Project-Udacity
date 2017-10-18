#include <iostream>
#include <vector>

#include "logger.h"
#include "path_planner.h"
#include "road.h"
#include "utils.h"
#include "trajectory.h"
#include "vehicle.h"

using namespace std;

// ----------------------------------------------------------------------------
PathPlanner::PathPlanner()
  : road(Road(LANE_WIDTH, CArrayToVector<int, double>(SPEED_LIMITS, NUM_LANES))), 
    behavior(Behavior(&road))
{
  LOG(logDEBUG4) << "PathPlanner::PathPlanner()";
  
  const double MPH2MPS = 0.44704; // TODO: Move this to a config file
  
  // Convert speed limits from mph to m/s
  for (double& speed : road.lane_speeds)
  {
    speed *= MPH2MPS;
  }
}

// ----------------------------------------------------------------------------
PathPlanner::~PathPlanner()
{
  LOG(logDEBUG4) << "PathPlanner::~PathPlanner()";
}

// ----------------------------------------------------------------------------
Trajectory PathPlanner::Generate()
{
  behavior.UpdateState();
  return behavior.GetTrajectory();
}

// ----------------------------------------------------------------------------
void PathPlanner::SetEgoData(const EgoSensorData& data)
{
  LOG(logDEBUG4) << "PathPlanner::SetEgoData() - data.s = " << data.s;
  LOG(logDEBUG4) << "PathPlanner::SetEgoData() - data.d = " << data.d;
  LOG(logDEBUG4) << "PathPlanner::SetEgoData() - data.x = " << data.x;
  LOG(logDEBUG4) << "PathPlanner::SetEgoData() - data.y = " << data.y;
  
  // Under the hood also set the start point of the trajectory with this data
  // PointCartesian pc(data.x, data.y);
  // PointFrenet pf(data.s, data.d); // Let's ignore the Frenet data, because it is an untrusted conversion from the Cartesian data. It's better if I use my own conversion from Cartesian to Frenet.
  // road.ego.position = Point(pc);
  // road.ego.yaw = data.yaw;
  // road.ego.speed = data.speed;
  road.UpdateEgoKinematics(data);
  
  LOG(logDEBUG4) << "PathPlanner::SetEgoData() - ego position = " << road.ego.position;
}

// ----------------------------------------------------------------------------
void PathPlanner::SetEnvironmentData(const EnvironmentSensorData& data)
{
  road.PopulateTraffic(data);
}

// ----------------------------------------------------------------------------
void PathPlanner::SetPointsAlreadyPassed(int n)
{
  behavior.strategy->N_points_passed = n;
}

// ----------------------------------------------------------------------------
void PathPlanner::SetPreviousPath(const json& previous_path_x, const json& previous_path_y)
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
