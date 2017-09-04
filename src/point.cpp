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
#include "point.h"

// ----------------------------------------------------------------------------
// PUBLIC
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
Point::Point() :
  x_(0), y_(0), s_(0), d_(0), 
  valid_cartesian_(false), valid_frenet_(false)
{
}

// ----------------------------------------------------------------------------
Point::~Point()
{
}

// "Get" methods
// ----------------------------------------------------------------------------
double Point::GetX() const
{
  LOG(logDEBUG4) << "Point::GetX()";
  double x = -1;
  if (valid_cartesian_)
  {
    x = x_;
  }
  else if (valid_frenet_)
  {
    double y;
    Map::GetInstance().GetXY(s_, d_, x, y);
  }
  else
  {
    LOG(logERROR) << "Point::GetX() - no valid representation! Was the Point set?";
  }
  
  return x;
}

// ----------------------------------------------------------------------------
double Point::GetY() const
{
  LOG(logDEBUG4) << "Point::GetY()";
  double y = -1;
  if (valid_cartesian_)
  {
    y = y_;
  }
  else if (valid_frenet_)
  {
    double x;
    Map::GetInstance().GetXY(s_, d_, x, y);
  }
  else
  {
    LOG(logERROR) << "Point::GetY() - no valid representation! Was the Point set?";
  }
  
  return y;
}

// ----------------------------------------------------------------------------
double Point::GetS() const
{
  LOG(logDEBUG4) << "Point::GetS()";
  double s = -1;
  if (valid_frenet_)
  {
    s = s_;
  }
  else if (valid_cartesian_)
  {
    // TODO: GetFrenet() independent on theta
  }
  else
  {
    LOG(logERROR) << "Point::GetS() - no valid representation! Was the Point set?";
  }
  
  return s;
}

// ----------------------------------------------------------------------------
double Point::GetD() const
{
  LOG(logDEBUG4) << "Point::GetD()";
  double d = -1;
  if (valid_frenet_)
  {
    d = d_;
  }
  else if (valid_cartesian_)
  {
    // TODO: GetFrenet() independent on theta
  }
  else
  {
    LOG(logERROR) << "Point::GetD() - no valid representation! Was the Point set?";
  }
  
  return d;
}

// "Set" methods
// ----------------------------------------------------------------------------
void Point::SetXY(double x, double y)
{
  LOG(logDEBUG4) << "Point::SetXY()";
  x_ = x;
  y_ = y;
  
  valid_cartesian_ = true;
  
  if (!valid_frenet_)
  {
    // Convert to frenet
    //Map::GetInstance().GetFrenet(x_, y_, yaw_, s_, d_);
    //TODO: GetFrenet
    valid_frenet_ = true;
  }
}

// ----------------------------------------------------------------------------
void Point::SetFrenet(double s, double d)
{
  LOG(logDEBUG4) << "Point::SetFrenet()";
  s_ = s;
  d_ = d;

  valid_frenet_ = true;

  if (!valid_cartesian_)
  {
    // Convert to cartesian
    Map::GetInstance().GetXY(s_, d_, x_, y_);
    valid_cartesian_ = true;
  }
}

// ----------------------------------------------------------------------------
ostream& operator<<(ostream& os, const Point& p)
{
  os << "x: " << p.x_ << " | "
    << "y: " << p.y_ << " | "
    << "s: " << p.s_ << " | "
    << "d: " << p.d_ << " | "
    << endl;

  return os;
}
