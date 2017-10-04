#include <utility>      // std::pair, std::make_pair
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
  // Not implemented yet!
  LOG(logERROR) << "Behavior::Behavior() - Not implemented yet!";
  
}

// ----------------------------------------------------------------------------
Behavior::Behavior(Road* road)
{
  LOG(logDEBUG4) << "Behavior::Behavior(Road*)";
  
  // Set road pointer
  this->road_ptr = road;
  
  // Init state
  state = BehaviorState::KEEP_LANE;
  
  // Set allowed state transitions:
  SetPossibleTransitions();
  
  // Set d-coordinate desired for each combination of lane and state
  SetDDesired();
  
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
double Behavior::GetDDesired(int lane, BehaviorState state) const
{
  const double INVALID_D = -1; // TODO: Move somewhere else
  double d_desired = INVALID_D;
  
  LaneStateKey doubleKey(lane, state);
  auto it = d_desired_by_lane_and_state.find(doubleKey);
  
  if (it != d_desired_by_lane_and_state.end())
  {
    d_desired = it->second;
  }
  
  return d_desired;
}

// ----------------------------------------------------------------------------
Trajectory Behavior::GetTrajectory()
{
  return strategy->trajectory;
}

// ----------------------------------------------------------------------------
void Behavior::UpdateState()
{
  LOG(logDEBUG3) << "Behavior::UpdateState()";

  Road& road = *road_ptr; // alias

  // Set starting point with the input road (add Road in the input)
  strategy->start_point = road.ego.position;
  strategy->start_yaw = road.ego.yaw;

  LOG(logDEBUG3) << "Behavior::UpdateState() - road.ego.position = \n" 
    << road.ego.position;
  LOG(logDEBUG3) << "Behavior::UpdateState() - strategy->start.position = \n"
    << strategy->start_point;
  
  // int N_lanes = road.GetNumberOfLanes();
  // double d_desired[N_lanes];
  
  // for (int lane = 0; lane < N_lanes; ++lane)
  // {
    // d_desired[lane] = road.LANE_WIDTH/2.0 + double(lane) * (road.LANE_WIDTH);
  // }
  
  // ---- DEBUG for lane change with splines -----
  // Set goal
  // strategy->goal_point = PointFrenet(strategy->start_point.GetS() + 30, strategy->start_point.GetD());
  
  // Define the space ahead
  const double ego_s = road.ego.position.GetS();
  const double ego_d = road.ego.position.GetD();
  // RoadSpace space_ahead;
  // space_ahead.s_down = ego_s;
  // space_ahead.s_up = space_ahead.s_down + 30;
  // space_ahead.d_left = ego_d - 2;
  // space_ahead.d_right = space_ahead.d_left + 8;
  double goal_s = ego_s + 30;
  double goal_d = ego_d;
  
  // if (!road.IsEmptySpace(space_ahead))
  // {
    // LOG(logDEBUG2) << "Behavior::UpdateState() - Vehicle detected ahead!! Trying to change lane...";
    // LOG(logDEBUG2) << "Behavior::UpdateState() - Current lane: " << road.ego.lane;
  // //   // change lane (ego.lane available)
  // //   // int goal_lane = road.ego.lanes_available[0];
  // //   int goal_lane = 0;
  // //   strategy->goal_point = PointFrenet(ego_s + 30, d_desired[goal_lane]);
    // goal_d = (road.ego.lane == 0) ? 6 : 2;
  // }
  // -----------------------------------------------
  
  LOG(logDEBUG3) << "Behavior::UpdateState() - Current state = " << int(state);
  
  // For each possible state, perturb the goal point associated with it, 
  //   generate the trajectory
  //   calculate the cost associated with it
  //   keep track of the best one (state, trajectory and cost)
  for (BehaviorState next_possible_state : state_transitions[state])
  {
    LOG(logDEBUG3) << "Behavior::UpdateState() - Next possible state = " << int(next_possible_state);
    goal_d = GetDDesired(road.ego.lane, next_possible_state);

    // Don't get off the road!
    if (goal_d < 0 || goal_d > (road.GetNumberOfLanes()*road.LANE_WIDTH))
    {
      // this state is invalid
      continue;
    }

    // TODO: perturb goal
    strategy->goal_point = Point(PointFrenet(goal_s, goal_d));

    LOG(logDEBUG3) << "Behavior::UpdateState() - strategy->goal_point = \n"
      << strategy->goal_point;
    
    LOG(logDEBUG3) << "Behavior::UpdateState() - calling GenerateTrajectory()";
    strategy->GenerateTrajectory();
  }
}

// ----------------------------------------------------------------------------
// PRIVATE
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
void Behavior::SetDDesired()
{
  Road& road = *road_ptr; // alias
  
  // TODO: This is sort of hardcoded for 3 lanes. It should be generic.
  
  // Useful constants
  const double PCL_OFFSET = road.LANE_WIDTH/4.0;
  const double d_left = road.GetCenterDByLane(0);
  const double d_center = road.GetCenterDByLane(1);
  const double d_right = road.GetCenterDByLane(2);
  
  // Keys
  LaneStateKey left_KL(0, BehaviorState::KEEP_LANE);
  LaneStateKey left_PCLR(0, BehaviorState::PREPARE_CHANGE_LANE_RIGHT);
  LaneStateKey left_CLR(0, BehaviorState::CHANGE_LANE_RIGHT);
  
  LaneStateKey center_KL(1, BehaviorState::KEEP_LANE);
  LaneStateKey center_PCLL(1, BehaviorState::PREPARE_CHANGE_LANE_LEFT);
  LaneStateKey center_CLL(1, BehaviorState::CHANGE_LANE_LEFT);
  LaneStateKey center_PCLR(1, BehaviorState::PREPARE_CHANGE_LANE_RIGHT);
  LaneStateKey center_CLR(1, BehaviorState::CHANGE_LANE_RIGHT);
  
  LaneStateKey right_KL(2, BehaviorState::KEEP_LANE);
  LaneStateKey right_PCLL(2, BehaviorState::PREPARE_CHANGE_LANE_LEFT);
  LaneStateKey right_CLL(2, BehaviorState::CHANGE_LANE_LEFT);
  
  // Values
  d_desired_by_lane_and_state[left_KL]   = d_left;
  d_desired_by_lane_and_state[left_PCLR] = d_left + PCL_OFFSET;
  d_desired_by_lane_and_state[left_CLR]  = d_center;
  
  d_desired_by_lane_and_state[center_KL]   = d_center;
  d_desired_by_lane_and_state[center_PCLL] = d_center - PCL_OFFSET;
  d_desired_by_lane_and_state[center_CLL]  = d_left;
  d_desired_by_lane_and_state[center_PCLR] = d_center + PCL_OFFSET;
  d_desired_by_lane_and_state[center_CLR]  = d_right;
  
  d_desired_by_lane_and_state[right_KL]   = d_right;
  d_desired_by_lane_and_state[right_PCLL] = d_right - PCL_OFFSET;
  d_desired_by_lane_and_state[right_CLL]  = d_center;
}

// ----------------------------------------------------------------------------
void Behavior::SetPossibleTransitions()
{
  LOG(logDEBUG4) << "Behavior::SetPossibleTransitions()";

  // Keep Lane transitions allowed
  state_transitions[BehaviorState::KEEP_LANE].insert(BehaviorState::KEEP_LANE);
  state_transitions[BehaviorState::KEEP_LANE].insert(BehaviorState::PREPARE_CHANGE_LANE_LEFT);
  state_transitions[BehaviorState::KEEP_LANE].insert(BehaviorState::PREPARE_CHANGE_LANE_RIGHT);
  state_transitions[BehaviorState::PREPARE_CHANGE_LANE_LEFT].insert(BehaviorState::KEEP_LANE);
  state_transitions[BehaviorState::PREPARE_CHANGE_LANE_LEFT].insert(BehaviorState::PREPARE_CHANGE_LANE_LEFT);
  state_transitions[BehaviorState::PREPARE_CHANGE_LANE_LEFT].insert(BehaviorState::CHANGE_LANE_LEFT);
  state_transitions[BehaviorState::PREPARE_CHANGE_LANE_RIGHT].insert(BehaviorState::KEEP_LANE);
  state_transitions[BehaviorState::PREPARE_CHANGE_LANE_RIGHT].insert(BehaviorState::PREPARE_CHANGE_LANE_RIGHT);
  state_transitions[BehaviorState::PREPARE_CHANGE_LANE_RIGHT].insert(BehaviorState::CHANGE_LANE_RIGHT);
  state_transitions[BehaviorState::CHANGE_LANE_LEFT].insert(BehaviorState::KEEP_LANE);
  state_transitions[BehaviorState::CHANGE_LANE_LEFT].insert(BehaviorState::CHANGE_LANE_LEFT);
  state_transitions[BehaviorState::CHANGE_LANE_RIGHT].insert(BehaviorState::KEEP_LANE);
  state_transitions[BehaviorState::CHANGE_LANE_RIGHT].insert(BehaviorState::CHANGE_LANE_RIGHT);
}


