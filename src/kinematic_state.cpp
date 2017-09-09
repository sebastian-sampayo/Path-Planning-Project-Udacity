#include <math.h>

#include "kinematic_state.h"
#include "point.h"
#include "sensor_data.h"
#include "utils.h"

// ----------------------------------------------------------------------------
// PUBLIC
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
KinematicState::KinematicState() :
  position(Point()), speed(0), yaw(0), accel(0)
{}

// ----------------------------------------------------------------------------
KinematicState::KinematicState(const EnvironmentSensorData::SensedVehicleData& data)
{
  PointCartesian pc(data.x, data.y);
  PointFrenet pf(data.s, data.d);
  Point p(pc, pf);
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
  
  // No information for acceleration. Don't set it.
}

// ----------------------------------------------------------------------------
KinematicState::KinematicState(const EgoSensorData& data)
{
  PointCartesian pc(data.x, data.y);
  PointFrenet pf(data.s, data.d);
  Point p(pc, pf);
  position = p;
  yaw = data.yaw;
  speed = data.speed;
}