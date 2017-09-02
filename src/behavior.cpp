#include <vector>

#include "behavior.h"
#include "logger.h"
#include "trajectory.h"
#include "trajectory_strategy.h"
#include "straight_line_strategy.h"

using namespace std;

// ----------------------------------------------------------------------------
// PUBLIC
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
Behavior::Behavior()
{
  LOG(logDEBUG4) << "Behavior::Behavior()";
  // Init state
  state = KEEP_LANE;
  
  // Set allowed state transitions:
  SetPossibleTransitions();
  
  // TODO: this should be set by the client code
  strategy = new StraightLineStrategy();
}

// ----------------------------------------------------------------------------
Behavior::~Behavior()
{
  LOG(logDEBUG4) << "Behavior::~Behavior()";
  delete strategy;
}

// ----------------------------------------------------------------------------
Trajectory Behavior::GetTrajectory()
{
  return strategy->trajectory;
}

// ----------------------------------------------------------------------------
void Behavior::UpdateState()
{
  strategy->GenerateTrajectory();
}

// ----------------------------------------------------------------------------
// PRIVATE
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
void Behavior::SetPossibleTransitions()
{
  LOG(logDEBUG4) << "Behavior::SetPossibleTransitions()";
  
  // Init the vector with each state
  for (int i = 0; i < NUM_BEHAVIOR_STATES; ++i)
  {
    set<BehaviorState> aux_set;
    state_transitions.push_back(aux_set);
  }

  // Keep Lane transitions allowed
  state_transitions[KEEP_LANE].insert(KEEP_LANE);
  state_transitions[KEEP_LANE].insert(PREPARE_CHANGE_LANE_LEFT);
  state_transitions[KEEP_LANE].insert(PREPARE_CHANGE_LANE_RIGHT);

  // ...
}