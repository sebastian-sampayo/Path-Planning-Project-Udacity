#include "point.h"
#include "map.h"
#include "spline_strategy.h"
#include "trajectory.h"

#include "logger.h"
#include "spline.h"
#include "utils.h"

#include <vector>

using namespace std;

// ----------------------------------------------------------------------------
// PUBLIC
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
void SplineStrategy::GenerateTrajectory()
{
  // Useful constants
  double start_s = start_point.GetS();
  double start_d = start_point.GetD();
  const double goal_s = goal_point.GetS();
  const double goal_d = goal_point.GetD();
  const double T_simulator = 0.02;
  const double MPH2MS = 0.44704;
  const double safe_distance = 30; // This should depend on the speed. We might think of a safe "time" distance better.
  const double spline_mid_point = 1;
  double prev_size =  previous_path.size();
  N_points_passed = trajectory.size() - prev_size;
  // Calculate how to break up spline points so that we travel at our desired reference velocity
  // reference_speed = delta_s / T_simulator
  // reference_speed * T_simulator = delta_s = distance between trajectory points
  const double delta_s = reference_speed * MPH2MS * T_simulator;
  
  // // Reset the trajectory with the previous path
  // trajectory = previous_path;
  // Remove the few points that the car has already passed.
  trajectory.erase(trajectory.begin(), trajectory.begin()+N_points_passed);
  previous_path = trajectory; // Use the previous trajectory instead of the simulator data, so that we avoid coordinate conversion errors.

  LOG(logDEBUG3) << "SplineStrategy::GenerateTrajectory() - N_points_passed = " << N_points_passed;
  LOG(logDEBUG3) << "SplineStrategy::GenerateTrajectory() - prev_size = " << prev_size;
  LOG(logDEBUG4) << "SplineStrategy::GenerateTrajectory() - previous_path = " << previous_path;
  LOG(logDEBUG3) << "SplineStrategy::GenerateTrajectory() - start_s = " << start_s;
  LOG(logDEBUG3) << "SplineStrategy::GenerateTrajectory() - delta_s = " << delta_s;

  vector<double> waypoints_s;
  vector<double> waypoints_d;
  
  // Start point
  // Set the previous path's last but one point as a waypoint, so the derivative at the start point
  // is the same as that of the previous path's end point.

  // if previous size is almost empty, use the car as starting referencec
  if (prev_size < 2)
  {
    waypoints_s.push_back(Map::GetInstance().CycleS(start_s - spline_mid_point));
    waypoints_d.push_back(start_d);
  }
  else if (previous_path[prev_size-2].GetS() >= previous_path[prev_size-1].GetS())
  {
    LOG(logWARNING) << "SplineStrategy::GenerateTrajectory() - prevS-2 >= prevS-1 ";
    LOG(logDEBUG3) << "SplineStrategy::GenerateTrajectory() - start_s = " << start_s 
      << endl
      << " previous_path[-1] = " << previous_path[prev_size-1]
      << endl
      << " previous_path[-2] = " << previous_path[prev_size-2];
  }
  else // use the previous path's end point as starting reference
  {
    // Redefine reference state as previous path end point
    // ref_x = previous_path_x[prev_size-1];
    // ref_y = previous_path_y[prev_size-1];
    // WHY? Debug this
    LOG(logDEBUG3) << "SplineStrategy::GenerateTrajectory() - start_s = " << start_s 
      << endl
      << " previous_path[-1] = " << previous_path[prev_size-1]
      << endl
      << " previous_path[-2] = " << previous_path[prev_size-2];

    waypoints_s.push_back(previous_path[prev_size-2].GetS());
    waypoints_d.push_back(previous_path[prev_size-2].GetD());
    
    start_s = previous_path[prev_size-1].GetS();
    start_d = previous_path[prev_size-1].GetD();
  }

  waypoints_s.push_back(start_s);
  waypoints_d.push_back(start_d);

  // Goal point
  waypoints_s.push_back(goal_s);
  waypoints_d.push_back(goal_d);

  waypoints_s.push_back(Map::GetInstance().CycleS(goal_s + spline_mid_point));
  // waypoints_s.push_back(Map::GetInstance().CycleS(goal_s + delta_s));
  waypoints_d.push_back(goal_d);

  LOG(logDEBUG3) << "SplineStrategy::GenerateTrajectory() - goal_s = " << goal_s;
  LOG(logDEBUG3) << "SplineStrategy::GenerateTrajectory() - goal_s + spline_mid_point = " << Map::GetInstance().CycleS(goal_s + spline_mid_point);
  LOG(logDEBUG3) << "SplineStrategy::GenerateTrajectory() - goal_d = " << goal_d;

  // Generate a spline
  tk::spline d_spline;
  d_spline.set_points(waypoints_s, waypoints_d);
  
  // DEBUG
  debug_spline = d_spline;

  // const double target_distance = goal_s;
  // N_points = target_distance / delta_s;
  double s = start_s;
  double d = start_d;

  for (int i = 0; i < N_points - prev_size; ++i)
  {
    s += delta_s;
    d = d_spline(s);
    trajectory.push_back(PointFrenet(s, d));
  }

  LOG(logDEBUG4) << "SplineStrategy::GenerateTrajectory() - trajectory = " << trajectory;
}

// ----------------------------------------------------------------------------
SplineStrategy::SplineStrategy()
{
  
}