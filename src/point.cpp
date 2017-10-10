/****************************************************************************\
 * Udacity Nanodegree: Self-Driving Car Engineering - December cohort
 * Project: Path Planning
 * Date: 
 * 
 * Author: Sebastián Lucas Sampayo
 * e-mail: sebisampayo@gmail.com
 * file: point.cpp
 * Description: 
 *  This implementation encapsulate the internal representation of the position
 *  so that we can change it in the future without changing the client code.
\****************************************************************************/

#include <iostream>

#include "map.h"
#include "logger.h"
#include "point.h"

// ----------------------------------------------------------------------------
// PUBLIC
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Constructors / Destructors
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
Point::Point() { SetFrenet(0,0); }

// ----------------------------------------------------------------------------
Point::Point(const PointCartesian& p) { SetXY(p.x, p.y); }

// ----------------------------------------------------------------------------
Point::Point(const PointFrenet& p) { SetFrenet(p.s, p.d); }

// ----------------------------------------------------------------------------
Point::Point(const PointCartesian& pc, const PointFrenet& pf)
{
  point_cartesian_.x = pc.x;
  point_cartesian_.y = pc.y;
  point_frenet_.s = pf.s;
  point_frenet_.d = pf.d;
}

// ----------------------------------------------------------------------------
Point::~Point()
{ LOG(logDEBUG5) << "Point::~Point()"; }

// ----------------------------------------------------------------------------
// Getters
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
double Point::GetX() const { return point_cartesian_.x; }
double Point::GetY() const { return point_cartesian_.y; }
double Point::GetS() const { return point_frenet_.s; }
double Point::GetD() const { return point_frenet_.d; }

// ----------------------------------------------------------------------------
// Setters
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
void Point::SetXY(double x, double y)
{
  point_cartesian_ = PointCartesian(x, y);
  
  // Also set the other representation
  point_frenet_ = PointFrenet(point_cartesian_);
}

// ----------------------------------------------------------------------------
void Point::SetFrenet(double s, double d)
{
  point_frenet_ = PointFrenet(s, d);
  
  // Also set the other representation
  point_cartesian_ = PointCartesian(point_frenet_);
}

// ----------------------------------------------------------------------------
// Converters
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Constructor: Convert from Frenet to Cartesian
PointCartesian::PointCartesian(const PointFrenet& p)
{
  Map::GetInstance().ToCartesian(p.s, p.d, x, y);
}

// ----------------------------------------------------------------------------
// Constructor: Convert from Cartesian to Frenet
PointFrenet::PointFrenet(const PointCartesian& p)
{
  Map::GetInstance().ToFrenet(p.x, p.y, s, d);
}

// ----------------------------------------------------------------------------
// Adds a point in Frenet Coordinates
Point& Point::operator+=(const PointFrenet& rhs)
{
  double s = point_frenet_.s + rhs.s;
  double d = point_frenet_.d + rhs.d;
  SetFrenet(s, d);
  
  return *this;
}


// ----------------------------------------------------------------------------
// Adds a point in Frenet Coordinates
Point& Point::operator-=(const PointFrenet& rhs)
{
  double s = point_frenet_.s - rhs.s;
  double d = point_frenet_.d - rhs.d;
  SetFrenet(s, d);
  
  return *this;
}

// ----------------------------------------------------------------------------
// Printers
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
ostream& operator<<(ostream& os, const Point& p)
{
  os << "x: " << p.point_cartesian_.x << " | "
    << "y: " << p.point_cartesian_.y << " | "
    << "s: " << p.point_frenet_.s << " | "
    << "d: " << p.point_frenet_.d << " | ";
    // << endl;

  return os;
}

// ----------------------------------------------------------------------------
ostream& operator<<(ostream& os, const PointCartesian& p)
{
  os << "x: " << p.x << " | "
    << "y: " << p.y << " | ";
    // << endl;

  return os;
}

// ----------------------------------------------------------------------------
ostream& operator<<(ostream& os, const PointFrenet& p)
{
  os << "s: " << p.s << " | "
    << "d: " << p.d << " | ";
    // << endl;

  return os;
}