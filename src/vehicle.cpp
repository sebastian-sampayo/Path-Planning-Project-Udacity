#include "point.h"
#include "road.h"
#include "vehicle.h"

#include "logger.h"
#include "utils.h"

#include <iostream>
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>

// ----------------------------------------------------------------------------
// PUBLIC
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Initializes Vehicle
Vehicle::Vehicle()
{
  LOG(logDEBUG4) << "Vehicle::Vehicle()";
  speed = 0;
  yaw = 0;
  
  road_ptr = NULL;
}

// ----------------------------------------------------------------------------
Vehicle::Vehicle(const EnvironmentSensorData::SensedVehicleData& data, Road* road)
{
  this->road_ptr = road;
  UpdateSensorData(data);
}

// ----------------------------------------------------------------------------
Vehicle::~Vehicle() {}

// ----------------------------------------------------------------------------
void Vehicle::Move(double delta_t)
{
  position = PredictPosition(delta_t);
}

// ----------------------------------------------------------------------------
Point Vehicle::PredictPosition(double delta_t) const
{
  const double current_x = this->position.GetX();
  const double current_y = this->position.GetY();
  const double current_vx = speed * cos(yaw);
  const double current_vy = speed * sin(yaw);
  const double future_x = current_x + current_vx * delta_t;
  const double future_y = current_y + current_vy * delta_t;
  
  return PointCartesian(future_x, future_y);
}

// ----------------------------------------------------------------------------
void Vehicle::Translate(Point new_pos)
{
  double dy = new_pos.GetY() - position.GetY();
  double dx = new_pos.GetX() - position.GetX();
  yaw = atan2(dy, dx);
  position = new_pos;
}

// ----------------------------------------------------------------------------
void Vehicle::UpdateSensorData(const EnvironmentSensorData::SensedVehicleData& data)
{
  PointCartesian pc(data.x, data.y);
  // PointFrenet pf(data.s, data.d); // Let's ignore frenet coordinates. I trust cartesian!
  // Point p(pc, pf);
  Point p(pc);
  position = p;
  speed = Magnitude(data.vx, data.vy);
  
  if (data.vx > 0.001)
  {
    yaw = atan2(data.vy, data.vx);
  }
  else
  {
    yaw = 0;
  }
  
  // Update current Lane
  lane = int(p.GetD() / road_ptr->LANE_WIDTH);
}