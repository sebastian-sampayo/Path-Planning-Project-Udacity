/****************************************************************************\
 * Udacity Nanodegree: Self-Driving Car Engineering - December cohort
 * Project: Path Planning
 * Date: 
 * 
 * Author: Sebastián Lucas Sampayo
 * e-mail: sebisampayo@gmail.com
 * file: position.cpp
 * Description: 
 *  This implementation encapsulate the internal representation of the position
 *  so that we can change it in the future without changing the client code.
\****************************************************************************/

#include <iostream>

#include "map.h"
#include "position.h"

// ----------------------------------------------------------------------------
// PUBLIC
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
Position::Position() :
  x_(0), y_(0), s_(0), d_(0), yaw_(0), 
  valid_cartesian_(false), valid_frenet_(false), valid_yaw_(false)
{
}

// ----------------------------------------------------------------------------
Position::~Position()
{
}

// "Get" methods
// ----------------------------------------------------------------------------
double Position::GetX() const
{
  LOG(logDEBUG4) << "Position::GetX()";
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
    LOG(logERROR) << "Position::GetX() - no valid representation! Was the Position set?";
  }
  
  return x;
}

// ----------------------------------------------------------------------------
double Position::GetY() const
{
  LOG(logDEBUG4) << "Position::GetY()";
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
    LOG(logERROR) << "Position::GetY() - no valid representation! Was the Position set?";
  }
  
  return y;
}

// ----------------------------------------------------------------------------
double Position::GetS() const
{
  LOG(logDEBUG4) << "Position::GetS()";
  double s = -1;
  if (valid_frenet_)
  {
    s = s_;
  }
  else if (valid_cartesian_)
  {
    if (valid_yaw_)
    {
      double d;
      Map::GetInstance().GetFrenet(x_, y_, yaw_, s, d);
    }
    else
    {
      LOG(logERROR) << "Position::GetS() - Trying to calculate Frenet from Cartesian but yaw was not set properly!";
    }
  }
  else
  {
    LOG(logERROR) << "Position::GetS() - no valid representation! Was the Position set?";
  }
  
  return s;
}

// ----------------------------------------------------------------------------
double Position::GetD() const
{
  LOG(logDEBUG4) << "Position::GetD()";
  double d = -1;
  if (valid_frenet_)
  {
    d = d_;
  }
  else if (valid_cartesian_)
  {
    if (valid_yaw_)
    {
      double s;
      Map::GetInstance().GetFrenet(x_, y_, yaw_, s, d);
    }
    else
    {
      LOG(logERROR) << "Position::GetD() - Trying to calculate Frenet from Cartesian but yaw was not set properly!";
    }
  }
  else
  {
    LOG(logERROR) << "Position::GetD() - no valid representation! Was the Position set?";
  }
  
  return d;
}

// ----------------------------------------------------------------------------
double Position::GetYaw() const
{
  return yaw_;
}

// "Set" methods
// ----------------------------------------------------------------------------
void Position::SetXY(double x, double y)
{
  LOG(logDEBUG4) << "Position::SetXY()";
  x_ = x;
  y_ = y;
  
  valid_cartesian_ = true;
  
  if (!valid_frenet_ && valid_yaw_)
  {
    // Convert to frenet
    Map::GetInstance().GetFrenet(x_, y_, yaw_, s_, d_);
    valid_frenet_ = true;
  }
}

// ----------------------------------------------------------------------------
void Position::SetFrenet(double s, double d)
{
  LOG(logDEBUG4) << "Position::SetFrenet()";
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
void Position::SetYaw(double yaw)
{
  yaw_ = yaw;
  valid_yaw_ = true;
}

// ----------------------------------------------------------------------------
ostream& operator<<(ostream& os, const Position& p)
{
  os << "x: " << p.x_ << " | "
    << "y: " << p.y_ << " | "
    << "s: " << p.s_ << " | "
    << "d: " << p.d_ << " | "
    << "yaw: " << p.yaw_ << " | "
    << endl;

  return os;
}