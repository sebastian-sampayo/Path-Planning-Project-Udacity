#include <vector>

#include "logger.h"
#include "straight_line_strategy.h"
#include "trajectory.h"
#include "utils.h"

using namespace std;

// ----------------------------------------------------------------------------
// PUBLIC
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
void StraightLineStrategy::GenerateTrajectory()
{
  GenerateXYTrajectory();
}

// ----------------------------------------------------------------------------
StraightLineStrategy::StraightLineStrategy()
{
  LOG(logDEBUG4) << "StraightLineStrategy::StraightLineStrategy()";
}

// ----------------------------------------------------------------------------
// PRIVATE
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
void StraightLineStrategy::GenerateXYTrajectory()
{
  double car_x = start.position.x;
  double car_y = start.position.y;
  double car_yaw = start.yaw;
  
  // Straight line test
  double dist_inc = 0.5;
  for(int i = 0; i < 50; i++)
  {
    Point p;
    p.x = car_x + (dist_inc*i) * cos(deg2rad(car_yaw));
    p.y = car_y + (dist_inc*i) * sin(deg2rad(car_yaw));
    trajectory.points.push_back(p);
  }
}

// ----------------------------------------------------------------------------
void StraightLineStrategy::GenerateSDTrajectory()
{
  double car_x = start.position.x;
  double car_y = start.position.y;
  double car_yaw = start.yaw;
  
  // Straight line test
  double dist_inc = 0.5;
  for(int i = 0; i < 50; i++)
  {
    double s = 0;
    double d = 0;
    Point p;
    // vector<double> xy = getXY(s, d, );
    // p.x = xy[0];
    // p.y = xy[1];
    trajectory.points.push_back(p);
  }
}