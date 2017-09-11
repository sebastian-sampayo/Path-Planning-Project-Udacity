#include "point.h"

#include "logger.h"
#include "utils.h"

#include <iostream>
#include "vehicle.h"
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
}

// ----------------------------------------------------------------------------
Vehicle::Vehicle(const EnvironmentSensorData::SensedVehicleData& data)
{
  UpdateKinematics(data);
}

// ----------------------------------------------------------------------------
Vehicle::Vehicle(int lane, double s, double v, double a)
{
  LOG(logDEBUG4) << "Vehicle::Vehicle(int lane, double s, double v, double a)";
}

// ----------------------------------------------------------------------------
Vehicle::~Vehicle() {}

// ----------------------------------------------------------------------------
void Vehicle::UpdateKinematics(const EnvironmentSensorData::SensedVehicleData& data)
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
}
