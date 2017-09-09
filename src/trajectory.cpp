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
  
  for (Point& point : *this)
  {
    x_values.push_back(point.GetX());
  }

  return x_values;
}

// ----------------------------------------------------------------------------
vector<double> Trajectory::GetYvalues()
{
  vector<double> y_values;
  
  for (Point& point : *this)
  {
    y_values.push_back(point.GetY());
  }
  
  return y_values;
}

// ----------------------------------------------------------------------------
ostream& operator<<(ostream& os, const Trajectory& t)
{
  // Format:
  // s | d | x | y
  os << "|s\t|d\t|x\t|y" << endl;
  
  for (const Point& point : t)
  {
    os << "|"
      << point.GetS()
      << "\t|"
      << point.GetD()
      << "\t|"
      << point.GetX()
      << "\t|"
      << point.GetY()
      << endl;
  }
  
  return os;
}
