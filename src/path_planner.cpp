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

void PathPlanner::SetEgoData(EgoSensorData data)
{
  LOG(logDEBUG3) << "PathPlanner::SetEgoData() - data.s = " << data.s;
  this->ego_data.x = data.x;
  this->ego_data.y = data.y;
  this->ego_data.s = data.s;
  this->ego_data.d = data.d;
  this->ego_data.speed = data.speed;
  this->ego_data.yaw = data.yaw;
  
  // Under the hood also set the start point of the trajectory with this data
  road.ego.kinematic_state.position.SetXY(data.x, data.y);
  road.ego.kinematic_state.position.SetFrenet(data.s, data.d);
  road.ego.kinematic_state.position.SetYaw(data.yaw);
  road.ego.kinematic_state.speed = data.speed;
}

void PathPlanner::SetPreviousPath(json previous_path_x, json previous_path_y)
{
  behavior.strategy->previous_path.points.clear();

  int path_size = previous_path_x.size();
  for (int i = 0; i < path_size; ++i)
  {
    Position p;
    p.SetXY(previous_path_x[i], previous_path_y[i]);
    behavior.strategy->previous_path.points.push_back(p);
  }
}

void PathPlanner::SetPreviousEndPoint(double end_path_s, double end_path_d)
{
  behavior.strategy->previous_end_point.SetFrenet(end_path_s, end_path_d);
}