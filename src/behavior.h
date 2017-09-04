#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <vector>
#include <set>

#include "road.h"
#include "trajectory.h"
#include "trajectory_strategy.h"

using namespace std;

class Behavior {
public:
  enum BehaviorState {
    KEEP_LANE,
    PREPARE_CHANGE_LANE_LEFT,
    PREPARE_CHANGE_LANE_RIGHT,
    CHANGE_LANE_LEFT,
    CHANGE_LANE_RIGHT,
    
    NUM_BEHAVIOR_STATES
  };
  
  BehaviorState state;
  vector<set<BehaviorState>> state_transitions;
  TrajectoryStrategy* strategy;
  
  // Constructor and destructor
  Behavior();
  virtual ~Behavior();
  
  //! Get the generated trajectory
  Trajectory GetTrajectory();
  
  void UpdateState(Road& road);
  
private:
  void SetPossibleTransitions();
};

#endif