#include <vector>

#include "behavior.h"
#include "logger.h"
#include "road.h"
#include "trajectory.h"
#include "trajectory_strategy.h"
#include "straight_line_strategy.h"
#include "spline_strategy.h"
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
  state = BehaviorState::KEEP_LANE;
  
  // Set allowed state transitions:
  SetPossibleTransitions();
  
  // strategy = new StraightLineStrategy();
  // strategy = new WalkthroughStrategy();
  strategy = new SplineStrategy();
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
  // Road& road = *road_ptr; // alias
  // Set starting point with the input road (add Road in the input)
  LOG(logDEBUG3) << "Behavior::UpdateState()";
  strategy->start_point = road.ego.position;
  strategy->start_yaw = road.ego.yaw;
  LOG(logDEBUG3) << "Behavior::UpdateState() - road.ego.position = \n" 
    << road.ego.position;
  LOG(logDEBUG3) << "Behavior::UpdateState() - strategy->start.position = \n"
    << strategy->start_point;
  
  int N_lanes = road.GetNumberOfLanes();
  double d_desired[N_lanes];
  
  for (int lane = 0; lane < N_lanes; ++lane)
  {
    d_desired[lane] = road.LANE_WIDTH/2.0 + double(lane) * (road.LANE_WIDTH);
  }
  
  // ---- DEBUG for lane change with splines -----
  // Set goal
  // strategy->goal_point = PointFrenet(strategy->start_point.GetS() + 30, strategy->start_point.GetD());
  
  // Define the space ahead
  const double ego_s = road.ego.position.GetS();
  const double ego_d = road.ego.position.GetD();
  RoadSpace space_ahead;
  space_ahead.s_down = ego_s;
  space_ahead.s_up = space_ahead.s_down + 30;
  space_ahead.d_left = ego_d - 2;
  space_ahead.d_right = space_ahead.d_left + 8;
  double goal_s = ego_s + 30;
  double goal_d = ego_d;
  
  if (!road.IsEmptySpace(space_ahead))
  {
    LOG(logDEBUG2) << "Behavior::UpdateState() - Vehicle detected ahead!! Trying to change lane...";
    LOG(logDEBUG2) << "Behavior::UpdateState() - Current lane: " << road.ego.lane;
  //   // change lane (ego.lane available)
  //   // int goal_lane = road.ego.lanes_available[0];
  //   int goal_lane = 0;
  //   strategy->goal_point = PointFrenet(ego_s + 30, d_desired[goal_lane]);
    goal_d = (road.ego.lane == 0) ? 6 : 2;
  }
  
  strategy->goal_point = Point(PointFrenet(goal_s, goal_d));
  
  LOG(logDEBUG3) << "Behavior::UpdateState() - strategy->goal_point = \n"
    << strategy->goal_point;
  
  LOG(logDEBUG3) << "Behavior::UpdateState() - calling GenerateTrajectory()";
  strategy->GenerateTrajectory();
  // -----------------------------------------------
  
  // For each possible state, perturb the goal point associated with it, 
  //   generate the trajectory
  //   calculate the cost associated with it
  //   keep track of the best one (state, trajectory and cost)
  for (BehaviorState possible_state : state_transitions[state])
  {
    //
  }
}

// ----------------------------------------------------------------------------
// PRIVATE
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
void Behavior::SetPossibleTransitions()
{
  LOG(logDEBUG4) << "Behavior::SetPossibleTransitions()";
  
  // Init the vector with each state
  for (int i = 0; i < BehaviorState::NUM_BEHAVIOR_STATES; ++i)
  {
    set<BehaviorState> aux_set;
    state_transitions.push_back(aux_set);
  }

  // Keep Lane transitions allowed
  state_transitions[BehaviorState::KEEP_LANE].insert(BehaviorState::KEEP_LANE);
  state_transitions[BehaviorState::KEEP_LANE].insert(BehaviorState::PREPARE_CHANGE_LANE_LEFT);
  state_transitions[BehaviorState::KEEP_LANE].insert(BehaviorState::PREPARE_CHANGE_LANE_RIGHT);

  // ...
}
