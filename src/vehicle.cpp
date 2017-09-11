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
  
  // Update current Lane (lane width is kind of hardcoded. It would be better if the vehicle could see the current road, so that he can ask the lane width)
  double d = p.GetD();
  
  if (0 < d && d < 4)
  {
    lane = 0;
  }
  else if (4 < d && d < 8)
  {
    lane = 1;
  }
  else if (8 < d && d < 12)
  {
    lane = 2;
  }
  else 
  {
    lane = -1; // out of lane. may be keep the same logic (as if there were more lanes on the sides)
  }
}
