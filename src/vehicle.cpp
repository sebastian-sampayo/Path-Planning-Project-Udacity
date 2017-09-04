#include <iostream>
#include "vehicle.h"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>

#include "logger.h"

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle()
{
  LOG(logDEBUG4) << "Vehicle::Vehicle()";
}

Vehicle::Vehicle(int lane, double s, double v, double a)
{
  LOG(logDEBUG4) << "Vehicle::Vehicle(int lane, double s, double v, double a)";
}

Vehicle::~Vehicle() {}
