#include "behavior.h"
#include "map.h"
#include "road.h"
#include "trajectory.h"
#include "trajectory_strategy.h"
#include "straight_line_strategy.h"
#include "spline_strategy.h"
#include "walkthrough_strategy.h"

#include "logger.h"
#include "utils.h"

#include <math.h>
#include <utility>      // std::pair, std::make_pair
#include <vector>

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
  : road_ptr(road), cost(road)
{
  LOG(logDEBUG4) << "Behavior::Behavior(Road*)";

  // Init state
  state = BehaviorState::KEEP_LANE;
  
  // Set allowed state transitions:
  SetPossibleTransitions();
  
  // Set d-coordinate desired for each combination of lane and state
  SetDDesired();
  
  // strategy = new StraightLineStrategy();
  // strategy = new WalkthroughStrategy();
  strategy = new SplineStrategy();
  strategy->reference_speed = 0;
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
  return best_trajectory_;
}

// ----------------------------------------------------------------------------
void Behavior::UpdateState()
{
  Timer timer;
  
  const double MPH2MPS = 0.44704; // TODO: Move this to a config file
  LOG(logDEBUG3) << "------ Behavior::UpdateState() -------";

  Road& road = *road_ptr; // alias

  // Set starting point with the input road (add Road in the input)
  strategy->start_point = road.ego.position;
  strategy->start_yaw = road.ego.yaw;

  LOG(logDEBUG3) << "Behavior::UpdateState() - road.ego.position = \n" 
    << road.ego.position;
  LOG(logDEBUG3) << "Behavior::UpdateState() - strategy->start.position = \n"
    << strategy->start_point;
  
  // Define the space ahead
  // const double safe_distance = 30; // This should depend on the speed. We might think of a safe "time" distance better.
  const double safe_time = 2; // seconds
  const double safe_distance = 30; //safe_time * strategy->reference_speed; // meters
  const double perturbed_s_range = road.ego.lenght * 2;
  
  const double ego_s = road.ego.position.GetS();
  const double ego_d = road.ego.position.GetD();
  
  RoadSpace space_ahead;
  space_ahead.s_down = ego_s;
  space_ahead.s_up = space_ahead.s_down + safe_distance;
  space_ahead.d_left = ego_d - 2;
  space_ahead.d_right = space_ahead.d_left + 4;
  double goal_s = ego_s + 60; // TODO: design carefully goal_s
  double goal_d = ego_d;
  
  LOG(logDEBUG3) << "Behavior::UpdateState() - Current state = " << state;
  road.LogVehicles(logDEBUG3);
  
  double max_speed = road.lane_speeds[road.ego.lane];
  const double speed_increment = .224 * MPH2MPS; // [m/s]
  
  // Vehicle ahead detection - slow down
  const bool free_space_ahead = road.IsEmptySpace(space_ahead);
  if (!free_space_ahead)
  {
    // Match front vehicle's speed using a proportional control
    auto id_vector = road.GetVehiclesInSpace(space_ahead);
    
    // Front vehicle speed tracking
    // strategy->reference_speed -= 0.05*(strategy->reference_speed - road.vehicles[id_vector[0]].speed);
    const double desired_speed = road.vehicles[id_vector[0]].speed;
    const double error_speed = (desired_speed - road.ego.speed);
    // const double error_speed_sign = (error_speed > 0 ? 1.0 : 0.0);
    // strategy->reference_speed += error_speed_sign * speed_increment;
    const double kv = 0.02;
    strategy->reference_speed += kv * error_speed;
    LOG(logDEBUG2) << "Behavior::UpdateState() - kv * error_speed: " << kv * error_speed;
    
    // Front vehicle distance tracking
    const double desired_front_vehicle_distance = 15; // This value should be lower than space_ahead.s_up, otherwise it could crash with the front vehicle
    double front_vehicle_distance = road.vehicles[id_vector[0]].position.GetS() - road.ego.position.GetS();
    if (front_vehicle_distance < 0) front_vehicle_distance += Map::GetInstance().MAX_S;
    const double error_position = desired_front_vehicle_distance - front_vehicle_distance;
    // const double error_position_sign = (error_position > 0 ? 1.0 : 0.0);
    // strategy->reference_speed += error_position_sign * speed_increment;
    const double kp = 3e-3;
    strategy->reference_speed -= kp * error_position; // Notice the minus sign is because the control force (ref_speed) is not proportional to the controlled variable (position)
    LOG(logDEBUG2) << "Behavior::UpdateState() - kp * error_position: " << kp * error_position;
    
      // TODO: Try this: 
      // const double kv = 0.05;
      // const double desired_speed = road.vehicles[id_vector[0]].speed;
      // strategy->reference_speed += kv*(desired_speed - road.ego.speed);
      // TODO: To track position, try this:
      // const double kp = 0.05;
      // const double desired_front_vehicle_distance = 15; // This value should be lower than space_ahead.s_up, otherwise it could crash with the front vehicle
      // const double front_vehicle_distance = road.vehicles[id_vector[0]].position.GetS() - road.ego.position.GetS();
      // if (front_vehicle_distance < 0) front_vehicle_distance += Map::GetInstance().MAX_S;
      // double position_error = desired_front_vehicle_distance - front_vehicle_distance;
      // strategy->reference_speed += kp*(position_error);
          
    LOG(logDEBUG2) << "Behavior::UpdateState() - Vehicle detected ahead!! Slowing down..."
      << " strategy->reference_speed: " << strategy->reference_speed;
    // TODO: try to make the transitions smoother (why not use a PID?)
  }
  else if (strategy->reference_speed < max_speed)
  {
    strategy->reference_speed += speed_increment;
  }
  else
  {
    strategy->reference_speed -= speed_increment;
  }
  
  if (strategy->reference_speed < speed_increment || road.ego.speed < speed_increment)
  {
    LOG(logWARNING) << "Behavior::UpdateState() - Vehicle stopped!";
  }
  
  // For each possible state, perturb the goal point associated with it, 
  //   generate the trajectory
  //   calculate the cost associated with it
  //   keep track of the best one (state, trajectory and cost)
  double min_cost = TrajectoryCost::MAX_COST;
  const double min_cost_tol = 1e-4;
  Trajectory best_trajectory;
  BehaviorState best_state = BehaviorState::KEEP_LANE;
  TrajectoryStrategy* best_strategy = new SplineStrategy();
  
  for (BehaviorState next_possible_state : state_transitions[state])
  {
    LOG(logDEBUG3) << "Behavior::UpdateState() - Next possible state = " << next_possible_state;
    
    // If there are no vehicles ahead just keep lane, don't even generate trajectories for other states.
    if (free_space_ahead && next_possible_state != BehaviorState::KEEP_LANE)
    {
      LOG(logDEBUG2) << "Behavior::UpdateState() - Free space ahead! Keep lane";
      continue;
    }
    
    goal_d = GetDDesired(road.ego.lane, next_possible_state);

    // Don't get off the road!
    // TODO: move this condition to a helper function IsGoalDValid(d)
    if (goal_d < 0 || goal_d > (road.GetNumberOfLanes()*road.LANE_WIDTH))
    {
      // this state is invalid
      LOG(logDEBUG2) << "Behavior::UpdateState() - Goal out of the road, skip state";
      continue;
    }
    
    // Manually avoid changing lanes if there is a vehicle on the side
    if ((next_possible_state == BehaviorState::CHANGE_LANE_LEFT &&  !road.IsEgoLeftSideEmpty()) ||
        (next_possible_state == BehaviorState::CHANGE_LANE_RIGHT && !road.IsEgoRightSideEmpty()))
    {
      LOG(logDEBUG2) << "Behavior::UpdateState() - Avoiding changing lane: Vehicle on the side";
      continue;
    }

    // TODO: perturb goal
    // for (each perturbed goal)
    double best_speed = strategy->reference_speed;
    int N_speed_steps = 1; //min(5, int((max_speed - strategy->reference_speed) / speed_increment));
    
    if (N_speed_steps == 0 || strategy->reference_speed >= max_speed)
    {
      N_speed_steps = 1;
    }
    
    if (N_speed_steps == 0) LOG(logERROR) << "Behavior::UpdateState() - N_speed_steps = 0 !!!!!";

    const int N_s_steps = 1;
    
    // Store strategy temporary as we are going to stomp on it
    TrajectoryStrategy* strategy_copy = new SplineStrategy();
    *strategy_copy = *strategy;
    
    for (int j = 0; j < N_s_steps; ++j)
    {
      // Warning, this loop could lead to high acceleration and jerk transitions. 
      // May be we could limit it to just 3 values: -speed_inc, 0, +speed_inc
      for (int i = 0; i < N_speed_steps; ++i)
      {
        LOG(logDEBUG5) << "Behavior::UpdateState() - strategy->trajectory = " << strategy->trajectory;
        
        strategy->reference_speed += i*speed_increment;
        
        if (strategy->reference_speed >= max_speed)
        {
          strategy->reference_speed -= (i+1)*speed_increment;
        }
        
        // Range centered in goal_s
        // const double perturbed_goal_s = goal_s - perturbed_s_range/2.0 + j * perturbed_s_range / N_s_steps;
        // Range beginning in goal_s
        double perturbed_goal_s = goal_s + j * perturbed_s_range / N_s_steps;
        strategy->goal_point = Point(PointFrenet(perturbed_goal_s, goal_d));

        LOG(logDEBUG3) << "Behavior::UpdateState() - strategy->goal_point = \n"
          << strategy->goal_point;
        LOG(logDEBUG3) << "Behavior::UpdateState() - strategy->reference_speed = "
          << strategy->reference_speed;
        LOG(logDEBUG3) << "Behavior::UpdateState() - calling GenerateTrajectory()";
        strategy->GenerateTrajectory();
        
        double temp_cost = cost.CalculateCost(strategy->trajectory);
        LOG(logDEBUG3) << "Behavior::UpdateState() - temp_cost = " << temp_cost;
        
        // Update best values, only if the new cost is really smaller than the current min_cost (there might be an erratic behavior because of very small numeric errors)
        if (temp_cost < min_cost - min_cost_tol)
        {
          LOG(logDEBUG3) << "Behavior::UpdateState() - Best cost updated! | prev min_cost: " << min_cost << " | new temp_cost: " << temp_cost;
          min_cost = temp_cost;
          best_trajectory = strategy->trajectory;
          *best_strategy = *strategy;
          best_state = next_possible_state;
          best_speed = strategy->reference_speed;
        }
        
        // Reset strategy with the copy
        *strategy = *strategy_copy;
      }
    }
    
    delete strategy_copy;
  }
  
  // Update object state and best trajectory
  *strategy = *best_strategy;
  delete best_strategy;
  best_trajectory_ = best_trajectory;
  state = best_state;
  LOG(logDEBUG3) << "Behavior::UpdateState() - best_state = " << best_state;
  LOG(logDEBUG4) << "Behavior::UpdateState() - best_trajectory = " << best_trajectory;
  
  // If the best cost is too high, slow down
  // if (min_cost >= TrajectoryCost::MAX_COST) 
  // {
    // LOG(logDEBUG2) << "Behavior::UpdateState() - Cost too high! Slow down!";
    // strategy->reference_speed -= speed_increment;
  // }
  
  const double elapsed_time = timer.GetElapsedMiliSeconds();
  LOG(logDEBUG2) << "Behavior::UpdateState() - elapsed_time = " << elapsed_time << "ms";
}

// ----------------------------------------------------------------------------
ostream& operator<<(ostream& os, const Behavior::BehaviorState state)
{
  map<Behavior::BehaviorState, const char*> state_str;
  
  state_str[Behavior::BehaviorState::KEEP_LANE] = "KEEP_LANE";
  state_str[Behavior::BehaviorState::PREPARE_CHANGE_LANE_LEFT] = "PREPARE_CHANGE_LANE_LEFT";
  state_str[Behavior::BehaviorState::PREPARE_CHANGE_LANE_RIGHT] = "PREPARE_CHANGE_LANE_RIGHT";
  state_str[Behavior::BehaviorState::CHANGE_LANE_LEFT] = "CHANGE_LANE_LEFT";
  state_str[Behavior::BehaviorState::CHANGE_LANE_RIGHT] = "CHANGE_LANE_RIGHT";
  
  os << state_str[state]; // TODO: use find() and check for invalid state
  
  return os;
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
  
  // No mid state
  state_transitions[BehaviorState::KEEP_LANE].insert(BehaviorState::CHANGE_LANE_LEFT);
  state_transitions[BehaviorState::KEEP_LANE].insert(BehaviorState::CHANGE_LANE_RIGHT);
  
  // PREPARE_CHANGE_LANE mid state
  // state_transitions[BehaviorState::KEEP_LANE].insert(BehaviorState::PREPARE_CHANGE_LANE_LEFT);
  // state_transitions[BehaviorState::KEEP_LANE].insert(BehaviorState::PREPARE_CHANGE_LANE_RIGHT);
  // state_transitions[BehaviorState::PREPARE_CHANGE_LANE_LEFT].insert(BehaviorState::KEEP_LANE);
  // state_transitions[BehaviorState::PREPARE_CHANGE_LANE_LEFT].insert(BehaviorState::PREPARE_CHANGE_LANE_LEFT);
  // state_transitions[BehaviorState::PREPARE_CHANGE_LANE_LEFT].insert(BehaviorState::CHANGE_LANE_LEFT);
  // state_transitions[BehaviorState::PREPARE_CHANGE_LANE_RIGHT].insert(BehaviorState::KEEP_LANE);
  // state_transitions[BehaviorState::PREPARE_CHANGE_LANE_RIGHT].insert(BehaviorState::PREPARE_CHANGE_LANE_RIGHT);
  // state_transitions[BehaviorState::PREPARE_CHANGE_LANE_RIGHT].insert(BehaviorState::CHANGE_LANE_RIGHT);
  
  
  state_transitions[BehaviorState::CHANGE_LANE_LEFT].insert(BehaviorState::CHANGE_LANE_LEFT);
  state_transitions[BehaviorState::CHANGE_LANE_LEFT].insert(BehaviorState::KEEP_LANE);
  state_transitions[BehaviorState::CHANGE_LANE_RIGHT].insert(BehaviorState::CHANGE_LANE_RIGHT);
  state_transitions[BehaviorState::CHANGE_LANE_RIGHT].insert(BehaviorState::KEEP_LANE);
}


