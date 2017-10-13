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
Trajectory Trajectory::GetDerivative(double delta_t) const
{
  Trajectory derivative;
  
  const int size = this->size();
  
  for (int i = 0; i < size-1; ++i)
  {
    if (i >= size -1) LOG(logERROR) << "Trajectory::GetDerivative() - Index out of range! i: " << i;
    const Point p0 = this->at(i);
    const Point p1 = this->at(i+1);
    
    const double dx = (p1.GetX() - p0.GetX()) / delta_t;
    const double dy = (p1.GetY() - p0.GetY()) / delta_t;
    const double ds = (p1.GetS() - p0.GetS()) / delta_t;
    const double dd = (p1.GetD() - p0.GetD()) / delta_t;
    
    // Fix for stopped :
    Point dp = Point(PointCartesian(dx, dy), PointFrenet(ds, dd)); // Problem here
    
    derivative.push_back(dp);
  }
  
  return derivative;
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