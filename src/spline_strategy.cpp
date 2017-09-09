#include "point.h"
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
  vector<double> waypoints_s;
  vector<double> waypoints_d;
  
  // Useful constants
  const double start_x = start.position.GetX();
  const double start_y = start.position.GetY();
  const double start_s = start.position.GetS();
  const double start_d = start.position.GetD();
  const double goal_x = goal.GetX();
  const double goal_y = goal.GetY();
  const double T_simulator = 0.02;
  const double MPH2MS = 1/2.24;
  
  // Need 3 points at least!

  // Set starting point
  waypoints_s.push_back(start_s);
  waypoints_d.push_back(start_d);
  
  // Set goal point
  waypoints_s.push_back(goal.GetS());
  waypoints_d.push_back(goal.GetD());
  
  // Generate a spline
  tk::spline d_spline;
  d_spline.set_points(waypoints_s, waypoints_d);
  
  // Calculate how to break up spline points so that we travel at our desired reference velocity
  // reference_speed = distance / (N * T_simulator)
  // reference_speed * T_simulator = distance / N = delta_s = distance between trajectory points
  
  const double target_distance = distance(start_x, start_y, goal_x, goal_y);
  const double delta_s = reference_speed * MPH2MS * T_simulator;
  double s = start_s;
  double d = start_d;
  
  for (int i = 0; i < N_points; ++i)
  {
    trajectory.push_back(PointFrenet(s, d));
    s += delta_s;
    d += d_spline(s);
  }
}

// ----------------------------------------------------------------------------
SplineStrategy::SplineStrategy()
{
  
}