#include <vector>

#include "logger.h"
#include "map.h"
#include "point.h"
#include "straight_line_strategy.h"
#include "trajectory.h"
#include "utils.h"

using namespace std;

// ----------------------------------------------------------------------------
// PUBLIC
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
void StraightLineStrategy::GenerateTrajectory()
{
  // GenerateXYTrajectory();
  GenerateSDTrajectory();
}

// ----------------------------------------------------------------------------
StraightLineStrategy::StraightLineStrategy()
{
  LOG(logDEBUG3) << "StraightLineStrategy::StraightLineStrategy()";
}

// ----------------------------------------------------------------------------
// PRIVATE
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
void StraightLineStrategy::GenerateXYTrajectory()
{
  // Clear previous trajectory
  trajectory.clear();
  
  double car_x = start_point.GetX();
  double car_y = start_point.GetY();
  double car_yaw = start_yaw;
  
  // Straight line test
  double dist_inc = 0.5;
  for(int i = 0; i < 50; i++)
  {
    Point p;
    double x = car_x + (dist_inc*i) * cos(deg2rad(car_yaw));
    double y = car_y + (dist_inc*i) * sin(deg2rad(car_yaw));
    p.SetXY(x, y);
    trajectory.push_back(p);
  }
}

// ----------------------------------------------------------------------------
void StraightLineStrategy::GenerateSDTrajectory()
{
  // Clear previous trajectory
  trajectory.clear();
  
  double car_s = start_point.GetS();
  LOG(logDEBUG3) << "StraightLineStrategy::GenerateSDTrajectory() - car_s = " << car_s;
  
  // Straight line test
  double dist_inc = 0.2;
  for(int i = 0; i < 50; i++)
  {
    double s = car_s + double(i+1) * dist_inc;
    double d = 2 + 4;
    Point p;
    p.SetFrenet(s, d);
    trajectory.push_back(p);
    LOG(logDEBUG4) << "StraightLineStrategy::GenerateSDTrajectory() - p : " << p;
  }
}

// ----------------------------------------------------------------------------
void StraightLineStrategy::GenerateSDTrajectory2()
{
  // // Clear previous trajectory
  // trajectory.clear();
  
  // double ref_vel = 49.5;
  
  // vector<double> previous_path_x(previous_path.GetXvalues());
  // vector<double> previous_path_y(previous_path.GetYvalues());
  
  // double prev_size =  previous_path.size();
  // // Create a list of widely spaced (x, y) waypoints, evenly spaced at 30m
  // // later we will interpolate these waypoints with a spline and fill it in
  // // with more points that control sp...
  // vector<double> ptsx;
  // vector<double> ptsy;
  
  // // reference x, y, yaw states
  // // either we will reference the starting point as where the car is or at the previous path end point
  // double car_s = start_point.GetS();
  // double car_x = start_point.GetX();
  // double car_y = start_point.GetY();
  // double car_yaw = start_yaw;
  // double ref_x = car_x;
  // double ref_y = car_y;
  // double ref_yaw = deg2rad(car_yaw);
  
  // const double goal_d = goal_point.GetD();
  // const double T_simulator = 0.02; // TODO: Move to a configuration file
  // const double spline_mid_point = 1;

  // double delta_s = reference_speed * T_simulator;
    
  // N_points_passed = trajectory.size() - prev_size;
  
  // vector<double> waypoints_s;
  // vector<double> waypoints_d;
  
  // // if previous size is almost empty, use the car as starting referencec
  // if (prev_size < 2)
  // {
    // waypoints_s.push_back(Map::GetInstance().CycleS(start_s - spline_mid_point));
    // waypoints_d.push_back(start_d);
    
    // waypoints_s.push_back(start_s);
    // waypoints_d.push_back(start_d);
  // }
  // // use the previous path's end point as starting reference
  // else
  // {
    // const double prev_s = previous_path[prev_size-1].GetS();
    // waypoints_s.push_back(prev_s);
    // waypoints_d.push_back(previous_path[prev_size-1].GetD());
    
    // waypoints_s.push_back(previous_path[prev_idx+1].GetS());
    // waypoints_d.push_back(previous_path[prev_idx+1].GetD());
    
    
    // // Redefine reference state as previous path end point
    // ref_x = previous_path_x[prev_size-1];
    // ref_y = previous_path_y[prev_size-1];
    
    // double ref_x_prev = previous_path_x[prev_size-2];
    // double ref_y_prev = previous_path_y[prev_size-2];
    // ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
    
    // // Use two points that make the path tangent to the previous path's end point
    // ptsx.push_back(ref_x_prev);
    // ptsx.push_back(ref_x);
    
    // ptsy.push_back(ref_y_prev);
    // ptsy.push_back(ref_y);
  // }
  
  // // In Frenet add evenly 30m spaced points ahead of the starting reference
  // int lane = 1;
  // Point next_wp0, next_wp1, next_wp2;
  // next_wp0.SetFrenet(car_s+30, 2+4*lane);
  // next_wp1.SetFrenet(car_s+60, 2+4*lane);
  // next_wp2.SetFrenet(car_s+90, 2+4*lane);
  
  // ptsx.push_back(next_wp0.GetX());
  // ptsx.push_back(next_wp1.GetX());
  // ptsx.push_back(next_wp2.GetX());
  
  // ptsy.push_back(next_wp0.GetY());
  // ptsy.push_back(next_wp1.GetY());
  // ptsy.push_back(next_wp2.GetY());
  
  // for (int i = 0; i < ptsx.size(); ++i)
  // {
    // // siht car reference angle to 0 degrees
    // double shift_x = ptsx[i] - ref_x;
    // double shift_y = ptsy[i] - ref_y;
    
    // ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    // ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
    // LOG(logDEBUG4) << "WalkthroughStrategy::GenerateTrajectory() - ptsx[" << i << "] = " << ptsx[i];
    // if (prev_size > 1)
      // LOG(logDEBUG4) << "WalkthroughStrategy::GenerateTrajectory() - previous_path_x[" << prev_size-1-i << "] = " << previous_path_x[prev_size-1-i];
  // }
  
  // // create a spline
  // tk::spline s;
  
  // // set (x,y) points to the spline
  // s.set_points(ptsx, ptsy);
  
  // // Start with all of the previous path points from the last time
  // for(int i = 0; i < previous_path.size(); i++)
  // {
    // Point p;
    // p.SetXY(previous_path_x[i], previous_path_y[i]);
    // trajectory.push_back(p);
  // }
  
  // // Calculate how to break up spline points so that we travel at our desired reference velocity
  // double target_x = 30.0;
  // double target_y = s(target_x);
  // double target_dist = distance(target_x, target_y, 0, 0);
  
  // double x_add_on  = 0;
  
  // // Fill up the rest of our path planner after filling it with previous points,
  // // here we will always output 50 points
  // for (int i = 1; i <= 50 - previous_path_x.size(); ++i)
  // {
    // double N = target_dist / (0.02 * ref_vel / 2.24);
    // double x_point = x_add_on + target_x/N;
    // double y_point = s(x_point);
    
    // x_add_on  = x_point;
    
    // double x_ref = x_point;
    // double y_ref = y_point;
    
    // // rotate back to normal after rotating it earlier
    // x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
    // y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);
    
    // x_point += ref_x;
    // y_point += ref_y;
    
    // Point p;
    // p.SetXY(x_point, y_point);
    // trajectory.push_back(p);
  // }
  
  // double car_s = start_point.GetS();
  // LOG(logDEBUG3) << "StraightLineStrategy::GenerateSDTrajectory() - car_s = " << car_s;
  
  // // Straight line test
  // double dist_inc = 0.2;
  // for(int i = 0; i < 50; i++)
  // {
    // double s = car_s + double(i+1) * dist_inc;
    // double d = 2 + 4;
    // Point p;
    // p.SetFrenet(s, d);
    // trajectory.push_back(p);
    // LOG(logDEBUG4) << "StraightLineStrategy::GenerateSDTrajectory() - p : " << p;
  // }
}
  
  