#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <set>
#include <utility>      // std::pair, std::make_pair
#include <vector>

#include "road.h"
#include "trajectory.h"
#include "trajectory_strategy.h"
#include "trajectory_cost.h"

using namespace std;

class Behavior {
public:
  enum class BehaviorState {
    KEEP_LANE,
    PREPARE_CHANGE_LANE_LEFT, // Not used for the moment
    PREPARE_CHANGE_LANE_RIGHT, // Not used for the moment
    CHANGE_LANE_LEFT,
    CHANGE_LANE_RIGHT,
    
    NUM_BEHAVIOR_STATES
  };
  
  BehaviorState state;
  map<BehaviorState, set<BehaviorState> > state_transitions;
  TrajectoryStrategy* strategy; // TODO: <Refactor> Rename to generator
  TrajectoryCost cost;
  
  // The behavior model can see the road in which it is being applied.
  Road* road_ptr;
  
  // Constructor and destructor
  Behavior();  // Not implemented yet
  Behavior(Road*);
  virtual ~Behavior();
  
  //! Get the desired d-coordinate based on the current lane and the behavior state
  double GetDDesired(int lane, BehaviorState state) const;
  
  //! Get the generated trajectory
  const Trajectory& GetTrajectory();
  void UpdateState();
  
  //! Print state to stdout
  friend ostream& operator<<(ostream& os, const BehaviorState state);
  
private:
  void SetDDesired();
  void SetPossibleTransitions();
  
  // A double-key type definition
  typedef pair<int, BehaviorState> LaneStateKey;
  
  // A double-key map to store the desired d-coordinate for each combination of lane-state
  map<LaneStateKey, double> d_desired_by_lane_and_state;
  
  //! Store the best trajectory temporary
  Trajectory best_trajectory_;
};

#endif
