#include <iostream>
#include "vehicle.h"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>

#include "kinematic_state.h"
#include "logger.h"

// ----------------------------------------------------------------------------
// PUBLIC
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Initializes Vehicle
Vehicle::Vehicle()
{
  LOG(logDEBUG4) << "Vehicle::Vehicle()";
}

// ----------------------------------------------------------------------------
Vehicle::Vehicle(const EnvironmentSensorData::SensedVehicleData& data)
{
  kinematic_state = KinematicState(data);
}

// ----------------------------------------------------------------------------
Vehicle::Vehicle(int lane, double s, double v, double a)
{
  LOG(logDEBUG4) << "Vehicle::Vehicle(int lane, double s, double v, double a)";
}

// ----------------------------------------------------------------------------
Vehicle::~Vehicle() {}
