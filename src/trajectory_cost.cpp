#include <map>

#include "logger.h"
#include "road.h"
#include "trajectory.h"
#include "trajectory_cost.h"

using namespace std;

// ----------------------------------------------------------------------------
// PUBLIC
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
TrajectoryCost::TrajectoryCost()
{
  // Not implemented yet!
  LOG(logERROR) << "TrajectoryCost::TrajectoryCost() - Not implemented yet!";
}

// ----------------------------------------------------------------------------
TrajectoryCost::TrajectoryCost(Road* road)
  : road_ptr(road)
{
  LOG(logDEBUG4) << "TrajectoryCost::TrajectoryCost(Road*)";
  
  // TODO: Move this to a configuration file

  // Cost Functions
  functions[CostFunctions::MAX_JERK] = &TrajectoryCost::MaxJerk;
  
  // Weights
  weights[CostFunctions::MAX_JERK] = 1;
}

// ----------------------------------------------------------------------------
TrajectoryCost::~TrajectoryCost()
{
}

// ----------------------------------------------------------------------------
double TrajectoryCost::CalculateCost(Trajectory trajectory)
{
  this->trajectory = trajectory;

  double cost = 0;
  
  for (auto it : functions)
  {
    double weight = weights[it.first]; // TODO: use find and check for valid key
    cost += weight * (this->*(it.second))();
  }

  return cost;
}

// ----------------------------------------------------------------------------
double TrajectoryCost::MaxJerk()
{
  return 1;
}