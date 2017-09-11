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
  
  // Update current Lane
  lane = int(p.GetD() / road_ptr->LANE_WIDTH);
}
