#ifndef TRAJECTORY_COST_H
#define TRAJECTORY_COST_H

#include "road.h"
#include "trajectory.h"

#include <map>

using namespace std;

class TrajectoryCost {
public:
  //! Constructors
  TrajectoryCost();
  TrajectoryCost(Road* road);
  
  //! Destructor
  ~TrajectoryCost();
  
  //! These are the cost functions available. The constructor must initialize weights and function pointers
  enum class CostFunctions
  {
    MAX_JERK,
    
    NUM_COST_FUNCTIONS
  };
  
  typedef double (TrajectoryCost::*pfCostFunction)();
  
  map<CostFunctions, pfCostFunction> functions;
  map<CostFunctions, double> weights;
  Road* road_ptr;
  
  //! Calculate overall cost for the specified trajectory
  double CalculateCost(Trajectory trajectory);
  
  // Cost functions
  
  //! Penalize maximum jerk
  double MaxJerk();

private:
  Trajectory trajectory;
};

#endif // TRAJECTORY_COST_H