#ifndef TRAJECTORY_COST_H
#define TRAJECTORY_COST_H

#include "road.h"
#include "trajectory.h"

#include <map>

using namespace std;

class TrajectoryCost {
public:
  static constexpr double MAX_JERK = 10; // [m/s^3]
  static constexpr double MAX_ACCEL = 10; // [m/s^2]

  //! Constructors
  TrajectoryCost();
  TrajectoryCost(Road* road);
  
  //! Destructor
  ~TrajectoryCost();
  
  //! These are the cost functions available. The constructor must initialize weights and function pointers
  enum class CostFunctions
  {
    DETECT_COLLISION,
    EMPTY_SPACE,
    MAX_ACCEL,
    MAX_JERK,
    SPEED,
    
    NUM_COST_FUNCTIONS
  };
  
  typedef double (TrajectoryCost::*pfCostFunction)();
  
  map<CostFunctions, pfCostFunction> functions;
  map<CostFunctions, double> weights;
  Road* road_ptr;
  
  //! Calculate overall cost for the specified trajectory
  double CalculateCost(const Trajectory& trajectory);
  
  // Cost functions
  
  //! Binary cost function which penalizes collisions. Return 0 if no collision, else 1
  double DetectCollision();
  
  //! Penalizes trajectories whose end point are closer to a vehicle in the same lane
  double EmptySpace();
  
  //! Penalizes maximum acceleration
  double MaxAcceleration();
  
  //! Penalizes maximum jerk
  double MaxJerk();
  
  //! Penalizes the slowest trajectory
  double Speed();

private:
  Trajectory trajectory;
  
  //! A helper function that returns a value between 0 and 1 for x in the 
  // range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].
  // Useful for cost functions.
  double Logistic(double x);
  
  //! A helper function that returns a value between 0 and 1 for x in the
  // range [infinity, 0]
  double InvLogistic(double x);
};

#endif // TRAJECTORY_COST_H
