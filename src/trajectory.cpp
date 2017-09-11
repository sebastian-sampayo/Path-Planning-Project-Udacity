#include <vector>

#include <iostream>
#include "logger.h"
#include "point.h"
#include "trajectory.h"

using namespace std;

// ----------------------------------------------------------------------------
// PUBLIC
// ----------------------------------------------------------------------------

// Get values methods
// ----------------------------------------------------------------------------
vector<double> Trajectory::GetXvalues() { return GetValues(&Point::GetX); }
vector<double> Trajectory::GetYvalues() { return GetValues(&Point::GetY); }
vector<double> Trajectory::GetSvalues() { return GetValues(&Point::GetS); }
vector<double> Trajectory::GetDvalues() { return GetValues(&Point::GetD); }

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

// ----------------------------------------------------------------------------
// PRIVATE
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
vector<double> Trajectory::GetValues(double (Point::*getValue)() const)
{
  vector<double> values;
  
  for (Point& point : *this)
  {
    values.push_back((point.*getValue)());
  }

  return values;
}