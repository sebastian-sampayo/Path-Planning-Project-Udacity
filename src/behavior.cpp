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
  
  // Init trajectory generator strategy
  // strategy = new StraightLineStrategy();
  // strategy = new WalkthroughStrategy();
  strategy = new SplineStrategy();
  strategy->reference_speed = 0;
  strategy->reference_accel = 5;
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
const Trajectory& Behavior::GetTrajectory()
{
  return best_trajectory_;
}

// ----------------------------------------------------------------------------
void Behavior::UpdateState()
{
  Timer timer;
  
  const double MPH2MPS = 0.44704; // TODO: Move this to a config file
  const double T_simulator = 0.02; // TODO: Move this constant to a common configuration file
  
  LOG(logDEBUG3) << "------ Behavior::UpdateState() -------";
  
  // ---- Hack! -----
  // Update the reference_speed with whatever the speed is at the point where we are going to add points
  const double N_points_passed = strategy->trajectory.size() - strategy->previous_path.size();
  const double N_future = strategy->N_points - strategy->N_end_points_removed; // - N_points_passed;
  LOG(logDEBUG2) << "Behavior::UpdateState() - N_future: " << N_future;
  if (strategy->trajectory.size() > N_future)
  {
    Trajectory speed_trajectory = strategy->trajectory.GetDerivative(T_simulator);
    Point p = speed_trajectory[N_future-1];
    const double speed = Magnitude(p.GetX(), p.GetY());
    strategy->reference_speed = speed;
  }
  // -------------------------

  Road& road = *road_ptr; // alias

  // Set starting point with the ego position
  strategy->start_point = road.ego.position;
  strategy->start_yaw = road.ego.yaw;

  LOG(logDEBUG3) << "Behavior::UpdateState() - road.ego.position: "<< road.ego.position
    << " | strategy->start.position: " << strategy->start_point
    << " | strategy->reference_speed: " << strategy->reference_speed;

  strategy->reference_accel = 0;
  const double prev_ref_speed = strategy->reference_speed;

  // Define the space ahead. If nobody is there, the ego will keep lane
  const double safe_time = 1; // seconds
  const double target_range = 30;
  const double perturbed_s_range = road.ego.lenght * 4;
  
  const double ego_s = road.ego.position.GetS();
  const double ego_d = road.ego.position.GetD();
  
  RoadSpace space_ahead;
  space_ahead.s_down = ego_s;
  space_ahead.s_up = space_ahead.s_down + 100;
  space_ahead.d_left = road.ego.lane * road.LANE_WIDTH;
  space_ahead.d_right = space_ahead.d_left + road.LANE_WIDTH;
  double goal_s = ego_s + 60;
  double goal_d = road.GetCenterDByLane(road.ego.lane); // Center of the lane
  
  LOG(logINFO) << "Behavior::UpdateState() - Current state = " << state;
  road.LogVehicles(logDEBUG3);
  
  const double max_speed = road.lane_speeds[road.ego.lane];
  const double speed_increment = .224 * MPH2MPS; // [m/s]
  
  if (road.ego.speed >= max_speed)
    LOG(logWARNING) << "Behavior::UpdateState() - Speed too high!! ego.speed: " << road.ego.speed << "m/s | " << road.ego.speed / MPH2MPS << "mph";
  
  // Flag to hit the emergency brake in case a vehicle in front of us is going really slow
  bool emergency_brake = false;
  const double emergency_diff_speed = road.ego.speed/2;
  const double  emergency_dist = road.ego.lenght * 2;
  
  // Vehicle ahead detection - slow down
  const bool free_space_ahead = road.IsEmptySpace(space_ahead);
  if (!free_space_ahead)
  {
    // Match front vehicle's speed using a proportional control
    vector<int> id_vector = road.GetVehiclesInSpace(space_ahead);
    
    // Find the closest vehicle //TODO: Move the whole "space ahead vehicle detection" to a method inside road or vehicle.
    double min_dist_ahead = space_ahead.s_up - space_ahead.s_down;
    int id_closest = -1;
    
    for (int id: id_vector)
    {
      const Vehicle& vehicle = road.vehicles[id];
      const double dist = vehicle.position.GetS() - road.ego.position.GetS();
      if (dist < 0) LOG(logERROR) << "Behavior::UpdateState() - ERROR: distance < 0: " << dist;
      if (dist < min_dist_ahead)
      {
        min_dist_ahead = dist;
        id_closest = id;
      }
    }
    
    const double front_vehicle_speed = road.vehicles[id_closest].speed;
    const double diff_speed = front_vehicle_speed - road.ego.speed;
    emergency_brake = ( (diff_speed < 0 && -diff_speed > emergency_diff_speed) || (min_dist_ahead < emergency_dist) ) && road.ego.speed > max_speed/5.0;

    LOG(logINFO) << "Behavior::UpdateState() - min_dist_ahead: " << min_dist_ahead << " | id_closest: " << id_closest;
    
    // Track front vehicle only if it is within a minor range, using a Proportional controller
    if (min_dist_ahead < target_range)
    {
      if (id_closest == -1) LOG(logERROR) << "Behavior::UpdateState() - id_closest = -1 !! - closest vehicle ahead not found!";
      
      // Front vehicle speed tracking
      const double desired_speed = front_vehicle_speed;
      const double error_speed = (desired_speed - road.ego.speed);
      const double kv = 1e-3; //0.02;
      strategy->reference_speed += kv * error_speed;
      
      // Front vehicle distance tracking
      // This value should be lower than space_ahead.s_up, otherwise it could crash with the front vehicle
      // This value means that the desired distance is 50 points behind. This allows the generated trajectory to add points before the vehicle
      // const double desired_front_vehicle_distance = 50*road.ego.speed*T_simulator;
      // This way, the distance is defined in terms of time. It is always a distance such that there are "safe_time" seconds between both vehicles.
      const double desired_front_vehicle_distance = safe_time*road.ego.speed;
      double front_vehicle_distance = road.vehicles[id_closest].position.GetS() - road.ego.position.GetS();
      if (front_vehicle_distance < 0) front_vehicle_distance += Map::GetInstance().MAX_S;
      const double error_position = desired_front_vehicle_distance - front_vehicle_distance;
      // const double error_position_sign = (error_position > 0 ? 1.0 : 0.0);
      // strategy->reference_speed += error_position_sign * speed_increment;
      const double kp = 3e-3; //3e-3 for space ahead = 30 // 0.9e-3 for space ahead 100, target range 30, desired_dist = speed
      strategy->reference_speed -= kp * error_position; // Notice the minus sign is because the control force (ref_speed) is not proportional to the controlled variable (position)
      
      LOG(logDEBUG1) << "Behavior::UpdateState() - kv * error_speed: " << kv * error_speed << " | kp * error_position: " << kp * error_position;
      
      LOG(logINFO) << "Behavior::UpdateState() - Vehicle detected ahead!! Slowing down..."
        << " strategy->reference_speed: " << strategy->reference_speed;
      // TODO: try to make the transitions smoother (why not use a PID?)
    }
    else if (road.ego.speed < max_speed - speed_increment && strategy->reference_speed < max_speed - speed_increment)
    {
      const double error_speed = max_speed - road.ego.speed;
      const double kv = speed_increment / max_speed * 2;
      
      strategy->reference_speed += max(speed_increment, kv*error_speed);
      // strategy->reference_speed += speed_increment;
    }
    else
    {
      strategy->reference_speed -= speed_increment;
    }
  }
  else if (road.ego.speed < max_speed - speed_increment && strategy->reference_speed < max_speed - speed_increment)
  {
    const double error_speed = max_speed - road.ego.speed;
    const double kv = speed_increment / max_speed * 2;
    
    strategy->reference_speed += min(speed_increment, kv*error_speed);
      
    // If the ego vehicle is going to slow, speed up quickly
    if (road.ego.speed < max_speed/3)
    {
      strategy->reference_speed = max(road.ego.speed, strategy->reference_speed);
      strategy->reference_accel = 8;
    }
  }
  else
  {
    strategy->reference_speed -= speed_increment;
  }
  
  if (strategy->reference_speed < speed_increment/2.0 || road.ego.speed < speed_increment/2.0)
  {
    LOG(logWARNING) << "Behavior::UpdateState() - Vehicle stopped! | speed: " << road.ego.speed << " | prev trajectory: " << strategy->trajectory;
  }
  
  // ------------------- Analyze each state -----------------------
  // For each possible state, perturb the goal point associated with it, 
  //   generate the trajectory
  //   calculate the cost associated with it
  //   keep track of the best one (state, trajectory and cost)
  double min_cost = cost.MAX_COST + 1; // This value will always be greater than the trajectory cost
  const double min_cost_tol = 1e-4;
  Trajectory best_trajectory;
  BehaviorState best_state = BehaviorState::KEEP_LANE;
  TrajectoryStrategy* best_strategy = new SplineStrategy();
  int best_i = 0;
  int best_j = 0;
  bool can_change_lane = true;
  bool keep_lane_was_skipped = false;
  const double prev_goal_d = strategy->goal_point.GetD();
  int prev_goal_lane = road.DToLane(prev_goal_d);
  
  for (BehaviorState next_possible_state : state_transitions[state])
  {
    LOG(logDEBUG2) << "Behavior::UpdateState() - Next possible state = " << next_possible_state;
    
    // If there are no vehicles ahead just keep lane, don't even generate trajectories for other states. Unless we were trying to change lanes.
    if (free_space_ahead && next_possible_state != BehaviorState::KEEP_LANE && state == BehaviorState::KEEP_LANE)
    {
      LOG(logDEBUG2) << "Behavior::UpdateState() - Free space ahead! Keep lane";
      continue;
    }
    
    goal_d = GetDDesired(road.ego.lane, next_possible_state);
    
    // Hack for s = 4000, d = 10 going out of lane
    if (goal_s > 4000 && road.DToLane(goal_d) > 1)
    {
      goal_d -= road.ego.width/8.0;
    }
    
    // Manually avoid changing lanes if there is a vehicle on the side
    if ((next_possible_state == BehaviorState::CHANGE_LANE_LEFT &&  !road.IsEgoLeftSideEmpty()) ||
        (next_possible_state == BehaviorState::CHANGE_LANE_RIGHT && !road.IsEgoRightSideEmpty()))
    {
      LOG(logDEBUG2) << "Behavior::UpdateState() - Avoiding changing lane: Vehicle on the side";
      road.LogVehicles(logDEBUG2);
      can_change_lane = false;
      
      // If KEEP_LANE was skipped in the previous cycle, 
      if (keep_lane_was_skipped)
      {
        goal_d = GetDDesired(road.ego.lane, BehaviorState::KEEP_LANE);
      }
      else
        continue;
    }
    
    // Don't get back to KEEP_LANE if we were trying to change lanes and we haven't made it yet
    if (state != BehaviorState::KEEP_LANE && next_possible_state == BehaviorState::KEEP_LANE && prev_goal_lane != road.ego.lane && prev_goal_d != -1 && can_change_lane)
    {
      LOG(logDEBUG2) << "Behavior::UpdateState() - Trying to change lane. Skipping KEEP_LANE state. Ego pos: " << road.ego.position << " | Ego lane: " << road.ego.lane  << " | prev_goal_lane: " << prev_goal_lane << " | prev_goal_d: " << prev_goal_d;
      keep_lane_was_skipped = true;
      continue;
    }

    // Don't get off the road!
    // TODO: move this condition to a helper function IsGoalDValid(d)
    if (goal_d < 0 || goal_d > (road.GetNumberOfLanes()*road.LANE_WIDTH))
    {
      // this state is invalid
      LOG(logDEBUG2) << "Behavior::UpdateState() - Goal out of the road, skip state";
      continue;
    }
    
    if (goal_d - road.ego.width/2.0 < 0)
    {
      goal_d += road.ego.width/8.0;
    }
    else if (goal_d + road.ego.width/2.0 > (road.GetNumberOfLanes()*road.LANE_WIDTH))
    {
      goal_d -= road.ego.width/8.0;
    }
    

    // ------------------- Perturb goal ------------------
    const int N_s_steps = 2;
    int N_accel_steps = 5; // Must be odd, so that we get accel = 0 as a candidate
    const double accel_range = 8;
    const double accel_inc = accel_range/(N_accel_steps-1);
    
    // If the ego vehicle is stopped, don't iterate acceleration values
    if (road.ego.speed < max_speed/5)
    {
      N_accel_steps = 1;
    }
    
    // Store strategy temporary as we are going to stomp on it
    TrajectoryStrategy* strategy_copy = new SplineStrategy();
    *strategy_copy = *strategy;
    
    for (int j = 0; j < N_s_steps; ++j)
    {
      // Warning, this loop could lead to high acceleration and jerk transitions. 
      // May be we could limit it to just 3 values: -speed_inc, 0, +speed_inc
      for (int i = 0; i < N_accel_steps; ++i)
      {
        LOG(logDEBUG5) << "Behavior::UpdateState() - strategy->trajectory = " << strategy->trajectory;
        
        const double accel_i = -accel_range/2.0 + i*accel_inc;
        const double future_speed = road.ego.speed + N_future * accel_i * T_simulator / 2;
        
        if (road.ego.speed > max_speed/5 && N_accel_steps != 1 && future_speed < max_speed*0.98)
        {
          strategy->reference_accel = accel_i;
          LOG(logDEBUG3) << "Behavior::UpdateState() - Acceleration incremented. road.ego.speed: " << road.ego.speed << " | max_speed/5: " << max_speed/5 << " | future_speed: " << future_speed;
        }
        else
        {
          LOG(logDEBUG3) << "Behavior::UpdateState() - Acceleration not incremented. road.ego.speed: " << road.ego.speed << " | max_speed/5: " << max_speed/5 << " | future_speed: " << future_speed;
        }
        
        if (emergency_brake)
        {
          LOG(logWARNING) << "Behavior::UpdateState() - Activating emergency brake!!!";
          strategy->reference_accel -= 10;
          strategy->reference_speed -= speed_increment;
        }
        
        // Range centered in goal_s
        // double perturbed_goal_s = goal_s - perturbed_s_range/2.0 + j * perturbed_s_range / N_s_steps;
        // Range beginning in goal_s
        double perturbed_goal_s = goal_s - j * perturbed_s_range / N_s_steps;
        if (perturbed_goal_s - ego_s < 0) perturbed_goal_s += Map::GetInstance().MAX_S;
        strategy->goal_point = Point(PointFrenet(perturbed_goal_s, goal_d));

        LOG(logDEBUG2) << "Behavior::UpdateState() - Perturbed goal iteration: \n"
          << "i: " << i << ", j: " << j
          << " | goal_point: " << strategy->goal_point
          << " | reference_speed: " << strategy->reference_speed
          << " | reference_accel: " << strategy->reference_accel
          << " | future_speed: " << future_speed
          << " | max_speed: " << max_speed;
        LOG(logDEBUG3) << "Behavior::UpdateState() - calling GenerateTrajectory()";
        strategy->GenerateTrajectory();
        
        double temp_cost = cost.CalculateCost(strategy->trajectory);
        LOG(logDEBUG2) << "Behavior::UpdateState() - temp_cost = " << temp_cost;
        
        // Update best values, only if the new cost is really smaller than the current min_cost (there might be an erratic behavior because of very small numeric errors)
        if (temp_cost < min_cost - min_cost_tol)
        {
          LOG(logDEBUG3) << "Behavior::UpdateState() - Best cost updated! | prev min_cost: " << min_cost << " | new temp_cost: " << temp_cost;
          min_cost = temp_cost;
          best_trajectory = strategy->trajectory;
          *best_strategy = *strategy;
          best_state = next_possible_state;
          best_i = i;
          best_j = j;
        }
        
        // Reset strategy with the copy
        *strategy = *strategy_copy;
      }
    }
    
    delete strategy_copy;
  }
  
  // Update object state and best trajectory, only if we have found a better trajectory than 
  *strategy = *best_strategy;
  delete best_strategy;
  best_trajectory_ = best_trajectory;
  state = best_state;
  LOG(logINFO) << "Behavior::UpdateState() - best (i,j): (" << best_i << ", " << best_j << ")"
    << " | best_state: " << best_state << " | min_cost: " << min_cost;
  LOG(logINFO) << "Behavior::UpdateState() - Best iteration: \n"
        << " | goal_point: " << strategy->goal_point
        << " | reference_speed: " << strategy->reference_speed
        << " | reference_accel: " << strategy->reference_accel;
  LOG(logDEBUG4) << "Behavior::UpdateState() - best_trajectory = " << best_trajectory;
  
  if (best_trajectory.size() == 0 || strategy->trajectory.size() ==  0)
  {
    LOG(logERROR) << "Behavior::UpdateState() - best_trajectory.size() = 0 !!! ================= \n" << best_trajectory.size();
    LOG(logERROR) << "Behavior::UpdateState() - strategy->trajectory.size() = 0 !!! ================= \n" << strategy->trajectory.size();
    assert(false);
  }
  
  // If the best cost is too high, slow down
  // if (min_cost >= TrajectoryCost::MAX_COST && road.ego.speed > max_speed/2) 
  // {
    // LOG(logDEBUG2) << "Behavior::UpdateState() - Cost too high! Slow down!";
    // strategy->reference_speed -= speed_increment;
  // }
  
  // // Check if there is a speed limit violation
  // Trajectory speed_trajectory = best_trajectory_.GetDerivative(T_simulator);
  // double n = 0;
  // for (Point p : speed_trajectory)
  // {
    // const double speed = Magnitude(p.GetX(), p.GetY());
    
    // // if (speed > max_speed || fabs(p.GetX()) > max_speed || fabs(p.GetY()) > max_speed || fabs(p.GetS()) > max_speed || fabs(p.GetD()) > max_speed)
    // if (speed > max_speed)
    // {
      // LOG(logWARNING) << "Behavior::UpdateState() - Speed violation at n: " << n << " | v: " << p << " |v|: " << speed << endl
        // << "reference_speed: " << strategy->reference_speed
        // << " | reference_accel: " << strategy->reference_accel
        // << " | prev_ref_speed: " << prev_ref_speed
        // << " | current speed inc: " << strategy->reference_speed - prev_ref_speed
        // // << " | speed_increment: " << speed_increment
        // << " | best_state: " << best_state << endl
        // << "trajectory[" << n << "]: " << strategy->trajectory[n] << endl
        // << "trajectory[" << n+1 << "]: " << strategy->trajectory[n+1] << endl
        // << "Current Ego: speed: " << road.ego.speed
        // << " | Ego pos: " << road.ego.position << endl;
        // if (n < 50)
        // {
          // // LOG(logWARNING) << "speed_trajectory: " << speed_trajectory;
          // // assert(false);
        // }
    // }
    
    // n++;
  // }
  
  // // Check if there is an accel limit violation
  // Trajectory accel_trajectory = speed_trajectory.GetDerivative(T_simulator);
  // n = 0;
  // for (Point p : accel_trajectory)
  // {
    // const double accel = Magnitude(p.GetX(), p.GetY());
    
    // // if (speed > max_speed || fabs(p.GetX()) > max_speed || fabs(p.GetY()) > max_speed || fabs(p.GetS()) > max_speed || fabs(p.GetD()) > max_speed)
    // if (accel > 10)
    // {
      // LOG(logWARNING) << "Behavior::UpdateState() - Accel violation at n: " << n << " | a: " << p << " |a|: " << accel << endl
        // << "reference_speed: " << strategy->reference_speed
        // << " | reference_accel: " << strategy->reference_accel
        // << " | prev_ref_speed: " << prev_ref_speed
        // << " | current speed inc: " << strategy->reference_speed - prev_ref_speed
        // // << " | speed_increment: " << speed_increment
        // << " | best_state: " << best_state << endl
        // << "trajectory[" << n << "]: " << strategy->trajectory[n] << endl
        // << "trajectory[" << n+1 << "]: " << strategy->trajectory[n+1] << endl
        // << "trajectory[" << n+2 << "]: " << strategy->trajectory[n+2] << endl
        // << "speed_trajectory[" << n << "]: " << speed_trajectory[n] << endl
        // << "speed_trajectory[" << n+1 << "]: " << speed_trajectory[n+1] << endl
        // << "Current Ego: speed: " << road.ego.speed
        // << " | Ego pos: " << road.ego.position << endl;
        // // << "speed_trajectory: " << speed_trajectory;
        // // if (n < 50)
          // // assert(false);
    // }
    
    // n++;
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


