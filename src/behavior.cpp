#include <vector>

#include "behavior.h"
#include "logger.h"
#include "road.h"
#include "trajectory.h"
#include "trajectory_strategy.h"
#include "straight_line_strategy.h"
#include "walkthrough_strategy.h"

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
  // strategy = new StraightLineStrategy();
  strategy = new WalkthroughStrategy();
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
void Behavior::UpdateState(Road& road)
{
  // Set starting point with the input road (add Road in the input)
  LOG(logDEBUG3) << "Behavior::UpdateState()";
  strategy->start = road.ego.kinematic_state;
  LOG(logDEBUG4) << "Behavior::UpdateState() - road.ego.kinematic_state.position = \n" 
    << road.ego.kinematic_state.position;
  LOG(logDEBUG4) << "Behavior::UpdateState() - strategy->start.position = \n"
    << strategy->start.position;
  
  LOG(logDEBUG3) << "Behavior::UpdateState() - calling GenerateTrajectory()";
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