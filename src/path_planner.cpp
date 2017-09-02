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

PathPlanner::PathPlanner()
{
  LOG(logDEBUG4) << "PathPlanner::PathPlanner()";
  // behavior = Behavior();
}

PathPlanner::~PathPlanner()
{
  LOG(logDEBUG4) << "PathPlanner::~PathPlanner()";
}

Trajectory PathPlanner::Generate()
{
  behavior.UpdateState();
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

void PathPlanner::SetPreviousPath(json previous_path_x, json previous_path_y)
{
  int path_size = previous_path_x.size();
  for (int i = 0; i < path_size; ++i)
  {
    Point p;
    p.x = previous_path_x[i];
    p.y = previous_path_y[i];
    behavior.strategy->previous_path.points.push_back(p);
  }
}

void PathPlanner::SetPreviousEndPoint(double end_path_s, double end_path_d)
{
  behavior.strategy->previous_end_point.s = end_path_s;
  behavior.strategy->previous_end_point.d = end_path_d;
}