#include "point.h"
#include "map.h"
#include "spline_strategy.h"
#include "trajectory.h"

#include "logger.h"
#include "spline.h"
#include "utils.h"

#include <assert.h>
#include <vector>

using namespace std;

// ----------------------------------------------------------------------------
// PUBLIC
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
void SplineStrategy::GenerateTrajectory()
{
  // GenerateSDTrajectory();
  GenerateXYTrajectory();
}

// ----------------------------------------------------------------------------
void SplineStrategy::GenerateSDTrajectory()
{
  // Useful constants
  double start_s = start_point.GetS();
  double start_d = start_point.GetD();
  double goal_s = goal_point.GetS();
  const double goal_d = goal_point.GetD();
  const double T_simulator = 0.02; // TODO: Move to a configuration file
  const double spline_mid_point = 1;
  double prev_size =  previous_path.size();
  N_points_passed = trajectory.size() - prev_size;
  
  // If the ego hasn't passed any point since the last update:
  // This happens when the previous planner loop lasted less than T_simulator (check this)
  // So, to overcome the problem of passing again the same trajectory (that would lead to a stop), lets fake N_points_passed to 1, so we move on by 1 point
  if (N_points_passed == 0)
  {
    LOG(logWARNING) << "SplineStrategy::GenerateTrajectory() - N_points_passed = " << N_points_passed;
    // << "! | Don't generate!";
    LOG(logWARNING) << "SplineStrategy::GenerateTrajectory() - prev_size = " << prev_size;
    LOG(logWARNING) << "SplineStrategy::GenerateTrajectory() - trajectory.size() = " << trajectory.size();
    
    if (trajectory.size() > 0)
    {
      prev_size -= 1;
      N_points_passed = 1;
      // return; // early return
    }
  }

  // Calculate how to break up spline points so that we travel at our desired reference velocity
  // reference_speed = delta_s / T_simulator
  // reference_speed * T_simulator = delta_s = distance between trajectory points
  double delta_s = reference_speed * T_simulator;
  
  LOG(logDEBUG2) << "SplineStrategy::GenerateTrajectory() - N_points_passed: " << N_points_passed
    << " | prev_size: " << prev_size
    << " | trajectory.size(): " << trajectory.size()
    << " | delta_s: " << delta_s;
    
  if (N_points_passed > 0)
  {
    LOG(logDEBUG3) << "SplineStrategy::GenerateTrajectory() - Closest trajectory points: \n"
      << "trajectory[N_passed-1]: " << trajectory[N_points_passed-1] << endl
      << "trajectory[N_passed]: " << trajectory[N_points_passed] << endl
      << "trajectory[N_passed+1]: " << trajectory[N_points_passed+1] << endl
      << "trajectory[N_passed+2]: " << trajectory[N_points_passed+2] << endl
      << "trajectory[N_passed+3]: " << trajectory[N_points_passed+3] << endl
      << "trajectory[N_passed+4]: " << trajectory[N_points_passed+4] << endl
      << "trajectory[N_passed+5]: " << trajectory[N_points_passed+5];
  }

  // // Reset the trajectory with the previous path
  // trajectory = previous_path;
  // Remove the few points that the car has already passed.
  trajectory.erase(trajectory.begin(), trajectory.begin()+N_points_passed);
  previous_path = trajectory; // Use the previous trajectory instead of the simulator data, so that we avoid coordinate conversion errors.
  prev_size =  previous_path.size();
  
  // Remove some points at the end so the trajectory generated is more flexible
  // const int N_end_points_removed = 100;//0; //int(trajectory.size() * 3.0/4.0); // 85 // Moved to a class attribute
  
  if (N_end_points_removed < trajectory.size())
  {
    trajectory.erase(trajectory.end() - N_end_points_removed, trajectory.end());
    previous_path = trajectory; // Use the previous trajectory instead of the simulator data, so that we avoid coordinate conversion errors.
    prev_size =  previous_path.size();
  }

  LOG(logDEBUG4) << "SplineStrategy::GenerateTrajectory() - prev_size = " << prev_size;
  LOG(logDEBUG4) << "SplineStrategy::GenerateTrajectory() - previous_path = " << previous_path;
  LOG(logDEBUG4) << "SplineStrategy::GenerateTrajectory() - start_s = " << start_s;
  LOG(logDEBUG4) << "SplineStrategy::GenerateTrajectory() - delta_s = " << delta_s;

  vector<double> waypoints_s;
  vector<double> waypoints_d;
  
  // Start point
  // Set the previous path's last but one point as a waypoint, so the derivative at the start point
  // is the same as that of the previous path's end point.
  
  int prev_idx = prev_size - 2;
  
  // if previous size is almost empty, use the car as starting reference
  if (prev_size < 2)
  {
    waypoints_s.push_back(Map::GetInstance().CycleS(start_s - spline_mid_point));
    // waypoints_s.push_back(start_s - spline_mid_point);
    waypoints_d.push_back(start_d);
  }
  else if (previous_path[prev_idx].GetS() >= previous_path[prev_idx+1].GetS())
  {
    LOG(logERROR) << "SplineStrategy::GenerateTrajectory() - prevS-2 >= prevS-1 ";
    LOG(logERROR) << "SplineStrategy::GenerateTrajectory() - start_s = " << start_s << endl
      << " prev_idx = " << prev_idx << endl
      << " previous_path[prev_idx+1] = " << previous_path[prev_idx+1] << endl
      << " previous_path[prev_idx] = " << previous_path[prev_idx];
    // This should not happen!!    
    assert(false);
  }
  else // use the previous path's end point as starting reference
  {
    LOG(logDEBUG4) << "SplineStrategy::GenerateTrajectory() - start_s = " << start_s << endl
      << " prev_idx = " << prev_idx << endl
      << " previous_path[prev_idx+1] = " << previous_path[prev_idx+1] << endl
      << " previous_path[prev_idx] = " << previous_path[prev_idx];

    const double prev_s = previous_path[prev_idx].GetS();
    waypoints_s.push_back(prev_s);
    waypoints_d.push_back(previous_path[prev_idx].GetD());
    
    start_s = previous_path[prev_idx+1].GetS();
    start_d = previous_path[prev_idx+1].GetD();
    
    // Prevent from decreasing values at the end of the circuit
    if (start_s - prev_s < 0) start_s += Map::GetInstance().MAX_S;
  }

  waypoints_s.push_back(start_s);
  waypoints_d.push_back(start_d);

  // Goal point
  // Prevent from decreasing values at the end of the circuit
  if (goal_s - start_s < 0) goal_s += Map::GetInstance().MAX_S;
  waypoints_s.push_back(goal_s);
  waypoints_d.push_back(goal_d);

  double goal_s_bis = goal_s + spline_mid_point;
  // Prevent from decreasing values at the end of the circuit
  if (goal_s_bis - goal_s < 0) goal_s_bis += Map::GetInstance().MAX_S;
  waypoints_s.push_back(goal_s_bis);
  waypoints_d.push_back(goal_d);

  LOG(logDEBUG4) << "SplineStrategy::GenerateTrajectory() - goal_s = " << goal_s;
  LOG(logDEBUG4) << "SplineStrategy::GenerateTrajectory() - goal_s + spline_mid_point = " << goal_s_bis;
  LOG(logDEBUG4) << "SplineStrategy::GenerateTrajectory() - goal_d = " << goal_d;

  // Generate a spline
  tk::spline d_spline;
  d_spline.set_points(waypoints_s, waypoints_d);
  
  // DEBUG
  debug_spline = d_spline;

  // const double target_distance = goal_s;
  // N_points = target_distance / delta_s;
  double s = start_s;
  double d = start_d;
  
  const double max_speed = 49*0.44704; // 21.90496
  const double max_accel = 10;
  const double insane_accel = 50;
  const bool SPEED_HACK_ON = true;
  const bool ACCEL_HACK_ON = true;
  const int N_tries = 10;
  const bool HACK2 = true;

  for (int i = 0; i < N_points - prev_size; ++i)
  {
    Point p;
    Point prev_p;
    Point prev_prev_p;
    const double t_size = trajectory.size();
    Trajectory hack_trajectory;
    
    const double delta_s_inc = reference_accel * T_simulator * T_simulator / 2;
    
    if (t_size > 1)
    {
      prev_p = trajectory.back();
      prev_prev_p = *(trajectory.end()-2);
      // LOG(logDEBUG3) << "SplineStrategy::GenerateTrajectory() - prev_p: " << prev_p << " | prev_prev_p: " << prev_prev_p << " | trajectory: " << trajectory;
      // assert(false);
      hack_trajectory.push_back(trajectory[t_size-2]);
      hack_trajectory.push_back(trajectory[t_size-1]);
    }
    
    if (HACK2)
    {
      // Add delta_s in cartesian and then convert to frenet
      delta_s += delta_s_inc;
      s += delta_s;      
      d = d_spline(s);
      p = PointFrenet(s, d);
      
      if (t_size > 1)
      {
        hack_trajectory.push_back(p);
        const Trajectory speed_hack_trajectory = hack_trajectory.GetDerivative(T_simulator);
        const Point prev_v = speed_hack_trajectory[0];
        const Point v = speed_hack_trajectory[1];
        const double yaw = atan3(v.GetY(), v.GetX());
        const double next_x = prev_p.GetX() + cos(yaw) * delta_s;
        const double next_y = prev_p.GetY() + sin(yaw) * delta_s;
        hack_trajectory.pop_back();
        p = PointCartesian(next_x, next_y);
        s = p.GetS();
      }
      
      // Seems that speed is not invariant between Frenet and Cartesian system.
      // Hack to make impossible to exceed speed limit
      // delta_s = min(delta_s, 48*0.44704*T_simulator);
      bool substract_aux = true;
      
      for (int j = 0; j < N_tries; ++j)
      {
        d = d_spline(s);
        p = PointFrenet(s, d);
        
        if (t_size < 2) break;
        
        // double p_speed = Magnitude((p.GetX() - prev_p.GetX()), (p.GetY() - prev_p.GetY())) / T_simulator;
        hack_trajectory.push_back(p);
        const Trajectory speed_hack_trajectory = hack_trajectory.GetDerivative(T_simulator);
        const Point prev_v = speed_hack_trajectory[0];
        const Point v = speed_hack_trajectory[1];
        const double p_speed = Magnitude(v.GetX(), v.GetY());
        hack_trajectory.pop_back();
        
        if (p_speed > max_speed && SPEED_HACK_ON) // 21.90496
        {
          double aux = (delta_s_inc > 0 ? delta_s_inc*(j+1) : (delta_s/(N_tries-2)));
          
          LOG(logDEBUG1) << "SplineStrategy::GenerateTrajectory() - Reducing delta_s: " << delta_s << " | aux: " << aux << " | delta_s_inc: " << delta_s_inc << " | i: " << i <<  " | j: " << j << " | p_speed: " << p_speed << " | s: " << s << " | r_speed: " << reference_speed << " | r_accel: " << reference_accel << " | goal_s: " << goal_s << " | goal_d: " << goal_d;
          
          s -= aux;
          delta_s -= aux;
          
          s = Map::GetInstance().CycleS(s);
          if (s < 0) s += Map::GetInstance().MAX_S;
          // Turn acceleration off
          reference_accel = 0;
        }
        else
        {
          break;
        }
      }
    }
    else
    {
      if (reference_accel == 0 && delta_s_inc > 0) LOG(logERROR) << "SplineStrategy::GenerateTrajectory() - delta_s_inc: " << delta_s_inc;
      if (delta_s + delta_s_inc > 0 && i > 0)
        delta_s += delta_s_inc;
      
      s += delta_s;

      // Seems that speed is not invariant between Frenet and Cartesian system.
      // Hack to make impossible to exceed speed limit
      // delta_s = min(delta_s, 48*0.44704*T_simulator);
      bool substract_aux = true;
      
      for (int j = 0; j < N_tries; ++j)
      {
        d = d_spline(s);
        p = PointFrenet(s, d);
        
        if (t_size < 2) break;
        
        // double p_speed = Magnitude((p.GetX() - prev_p.GetX()), (p.GetY() - prev_p.GetY())) / T_simulator;
        hack_trajectory.push_back(p);
        const Trajectory speed_hack_trajectory = hack_trajectory.GetDerivative(T_simulator);
        const Point prev_v = speed_hack_trajectory[0];
        const Point v = speed_hack_trajectory[1];
        const double p_speed = Magnitude(v.GetX(), v.GetY());
        hack_trajectory.pop_back();
        
        if (p_speed > max_speed && SPEED_HACK_ON) // 21.90496
        {
          double aux = (delta_s_inc > 0 ? delta_s_inc*(j+1) : (delta_s/(N_tries-2)));
          
          LOG(logDEBUG1) << "SplineStrategy::GenerateTrajectory() - Reducing delta_s: " << delta_s << " | aux: " << aux << " | delta_s_inc: " << delta_s_inc << " | i: " << i <<  " | j: " << j << " | p_speed: " << p_speed << " | s: " << s << " | r_speed: " << reference_speed << " | r_accel: " << reference_accel << " | goal_s: " << goal_s << " | goal_d: " << goal_d;
          
          s -= aux;
          delta_s -= aux;
          
          s = Map::GetInstance().CycleS(s);
          if (s < 0) s += Map::GetInstance().MAX_S;
          // Turn acceleration off
          reference_accel = 0;
        }
        else
        {
          break;
        }
      }
      
      for (int j = 0; j < N_tries; ++j)
      {
        if (t_size < 2) break;
        
        d = d_spline(s);
        p = PointFrenet(s, d);

        hack_trajectory.push_back(p);
        const Trajectory speed_hack_trajectory = hack_trajectory.GetDerivative(T_simulator);
        const Point prev_v = speed_hack_trajectory[0];
        const Point v = speed_hack_trajectory[1];
        const double p_speed = Magnitude(v.GetX(), v.GetY());
        const Trajectory accel_hack_trajectory = speed_hack_trajectory.GetDerivative(T_simulator);
        const Point a = accel_hack_trajectory[0];
        const double p_accel = Magnitude(a.GetX(), a.GetY());
        hack_trajectory.pop_back();
        
        if (p_accel > max_accel && ACCEL_HACK_ON)
        {
          double aux = (delta_s_inc > 0 ? delta_s_inc*(j+1) : (delta_s/(N_tries-2)));
          
          if (j == 0)
          {
            substract_aux = a.GetS() > 0;
          }
          
          if (j == 0 && p_accel > insane_accel)
          {
            // aux *= (p_accel - max_accel)/insane_accel;
            // aux = delta_s / 3;
            const double dx = prev_p.GetX() - prev_prev_p.GetX();
            const double dy = prev_p.GetY() - prev_prev_p.GetY();
            // const double prev_yaw = atan3(v.GetY(), v.GetX());
            const double next_x = prev_p.GetX() + dx + delta_s_inc;
            const double next_y = prev_p.GetY() + dy + delta_s_inc;
            Point next_p = PointCartesian(next_x, next_y);
            aux = (substract_aux ? -1 : 1) * (next_p.GetS() - s);
          }
          
          if (s < 140) // (j == 0 || j == N_tries-1) && 
          {
            LOG(logWARNING) << "SplineStrategy::GT()-High Accel! p_accel: " << p_accel << "| delta_s: " << delta_s << " | delta_s_inc: " << delta_s_inc << " | aux: " << aux << " | i: " << i  <<  " | j: " << j <<  " | p_speed: " << p_speed << " | s: " << s << " | r_speed: " << reference_speed << " | r_accel: " << reference_accel << " | goal_s: " << goal_s << " | goal_d: " << goal_d
              << endl << "a: " << a << "v: " << v << " | prev_v: " << prev_v
              << endl << "p: " << p << " | prev_p: " << prev_p << " | prev_prev_p: " << prev_prev_p;
          }
          assert(prev_prev_p.GetS() != 0 && prev_prev_p.GetD() != 0);
          // assert(false);

          
          // if (max_accel - p_accel < 30)
          // {
            // // I think we should never be here
            // aux *= 0.5;
          // }
          
          if (substract_aux)
          {
            s -= aux;
            delta_s -= aux;
          }
          else
          {
            s += aux;
            delta_s += aux;
          }
          
          s = Map::GetInstance().CycleS(s);
          if (s < 0) s += Map::GetInstance().MAX_S;
          // Turn acceleration off
          reference_accel = 0;
        }
        else
        {
          break;
        }
      }
    }

    trajectory.push_back(p);
    if (N_points_passed == 0 && prev_size > 0) LOG(logERROR) << "SplineStrategy::GenerateTrajectory() - N_points_passed = 0 | prev_size = " << prev_size << " ! This shouldn't be executed!";
  }

  LOG(logDEBUG4) << "SplineStrategy::GenerateTrajectory() - trajectory = " << trajectory;
}

// ----------------------------------------------------------------------------
void SplineStrategy::GenerateXYTrajectory()
{
  // Useful constants
  double start_s = start_point.GetS();
  double start_d = start_point.GetD();
  double goal_s = goal_point.GetS();
  const double goal_d = goal_point.GetD();
  const double T_simulator = 0.02; // TODO: Move to a configuration file
  const double spline_mid_point = 1;
  double prev_size =  previous_path.size();
  N_points_passed = trajectory.size() - prev_size;
  // Clear previous trajectory
  trajectory.clear();
  
  double ref_vel = reference_speed;
  LOG(logDEBUG3) << "WalkthroughStrategy::GenerateTrajectory() - prev_size = " << prev_size;
  
  if (N_end_points_removed < previous_path.size())
  {
    previous_path.erase(previous_path.end() - N_end_points_removed, previous_path.end());
    // previous_path = trajectory; // Use the previous trajectory instead of the simulator data, so that we avoid coordinate conversion errors.
    prev_size =  previous_path.size();
  }
  
  vector<double> previous_path_x(previous_path.GetXvalues());
  vector<double> previous_path_y(previous_path.GetYvalues());
  
  // Create a list of widely spaced (x, y) waypoints, evenly spaced at 30m
  // later we will interpolate these waypoints with a spline and fill it in
  // with more points that control sp...
  vector<double> ptsx;
  vector<double> ptsy;
  
  // reference x, y, yaw states
  // either we will reference the starting point as where the car is or at the previous path end point
  double car_s = start_point.GetS();
  double car_x = start_point.GetX();
  double car_y = start_point.GetY();
  double car_yaw = start_yaw;
  double ref_x = car_x;
  double ref_y = car_y;
  double ref_yaw = car_yaw;
  
  // if previous size is almost empty, use the car as starting referencec
  if (prev_size < 2)
  {
    // Use two points that make the path tangent to the car
    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);
    
    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);
    
    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);
  }
  // use the previous path's end point as starting reference
  else
  {
    // Redefine reference state as previous path end point
    ref_x = previous_path_x[prev_size-1];
    ref_y = previous_path_y[prev_size-1];
    
    double ref_x_prev = previous_path_x[prev_size-2];
    double ref_y_prev = previous_path_y[prev_size-2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
    
    // Use two points that make the path tangent to the previous path's end point
    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);
    
    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }
  
  // In Frenet add evenly 30m spaced points ahead of the starting reference
  // int lane = 1;
  Point next_wp0, next_wp1, next_wp2;
  next_wp0.SetFrenet(goal_s, goal_d);
  next_wp1.SetFrenet(goal_s+30, goal_d);
  next_wp2.SetFrenet(goal_s+60, goal_d);
  
  ptsx.push_back(next_wp0.GetX());
  ptsx.push_back(next_wp1.GetX());
  ptsx.push_back(next_wp2.GetX());
  
  ptsy.push_back(next_wp0.GetY());
  ptsy.push_back(next_wp1.GetY());
  ptsy.push_back(next_wp2.GetY());
  
  for (int i = 0; i < ptsx.size(); ++i)
  {
    // siht car reference angle to 0 degrees
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;
    
    ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
    LOG(logDEBUG4) << "WalkthroughStrategy::GenerateTrajectory() - ptsx[" << i << "] = " << ptsx[i];
    if (prev_size > 1)
      LOG(logDEBUG4) << "WalkthroughStrategy::GenerateTrajectory() - previous_path_x[" << prev_size-1-i << "] = " << previous_path_x[prev_size-1-i];
  }
  
  // create a spline
  tk::spline s;
  
  // set (x,y) points to the spline
  s.set_points(ptsx, ptsy);
  
  // Start with all of the previous path points from the last time
  for(int i = 0; i < previous_path.size(); i++)
  {
    Point p;
    p.SetXY(previous_path_x[i], previous_path_y[i]);
    trajectory.push_back(p);
  }
  
  // Calculate how to break up spline points so that we travel at our desired reference velocity
  double target_x = 30.0;
  double target_y = s(target_x);
  double target_dist = distance(target_x, target_y, 0, 0);
  
  double x_add_on  = 0;
  
  // Fill up the rest of our path planner after filling it with previous points,
  // here we will always output 50 points
  for (int i = 1; i <= N_points - previous_path_x.size(); ++i)
  {
    double N = target_dist / (0.02 * ref_vel);
    double x_point = x_add_on + target_x/N + i * reference_accel * T_simulator * T_simulator / 2;
    double y_point = s(x_point);
    
    x_add_on  = x_point;
    
    double x_ref = x_point;
    double y_ref = y_point;
    
    // rotate back to normal after rotating it earlier
    x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
    
    x_point += ref_x;
    y_point += ref_y;
    
    Point p;
    p.SetXY(x_point, y_point);
    trajectory.push_back(p);
  }
}

// ----------------------------------------------------------------------------
SplineStrategy::SplineStrategy()
{
  
}
