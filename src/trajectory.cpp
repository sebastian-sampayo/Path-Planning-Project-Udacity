#include <vector>

#include <iostream>
#include "logger.h"
#include "point.h"
#include "trajectory.h"

using namespace std;

// ----------------------------------------------------------------------------
// PUBLIC
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
vector<double> Trajectory::GetXvalues()
{
  vector<double> x_values;
  
  for (Point& point : points)
  {
    x_values.push_back(point.GetX());
  }

  return x_values;
}

// ----------------------------------------------------------------------------
vector<double> Trajectory::GetYvalues()
{
  vector<double> y_values;
  
  for (Point& point : points)
  {
    y_values.push_back(point.GetY());
  }
  
  return y_values;
}

// ----------------------------------------------------------------------------
ostream& operator<<(ostream& os, const Trajectory& t)
{
  // Format:
  // x0, y0
  // x1, y1
  for (const Point& point : t.points)
  {
    os << point.GetX()
      << ", "
      << point.GetY()
      << endl;
  }
  
  return os;
}
