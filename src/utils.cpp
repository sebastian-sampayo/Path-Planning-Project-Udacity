#include <math.h>
#include <vector>

#include "Eigen-3.3/Eigen/Core"

#include "utils.h"

using namespace std;

// ----------------------------------------------------------------------------
double atan3(double y, double x)
{
  double theta = 0;
  const double small_x = fabs(x) < ATAN3_TOL;
  const double small_y = fabs(y) < ATAN3_TOL;
  
  if ( (!small_x && !small_y) || (small_x && small_y) )
  {
    theta = atan2(y, x);
  }
  else if (small_x)
  {
    theta = (y > 0 ? pi()/2.0 : -pi()/2.0);
  }
  else if (small_y)
  {
    theta = (x > 0 ? 0 : -pi());
  }
  else
  {
    // All possible cases are above. This should never be executed
    theta = atan2(y, x);
  }

  return theta;
}

// ----------------------------------------------------------------------------
double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// ----------------------------------------------------------------------------
double Magnitude(double x, double y)
{
  return sqrt(x*x + y*y);
}

// ----------------------------------------------------------------------------
int ClosestWaypoint(double x, double y, const vector<double>& maps_x, const vector<double>& maps_y)
{

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;

}

// ----------------------------------------------------------------------------
int NextWaypoint(double x, double y, double theta, const vector<double>& maps_x, const vector<double>& maps_y)
{

  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2( (map_y-y),(map_x-x) );

  double angle = abs(theta-heading);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  }

  return closestWaypoint;

}
