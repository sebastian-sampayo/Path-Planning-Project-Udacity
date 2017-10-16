#ifndef TRAJECTORY_STRATEGY_H
#define TRAJECTORY_STRATEGY_H

#include "point.h"
#include "road.h"
#include "trajectory.h"

#include "spline.h"

class TrajectoryStrategy {
public:
  //! Number of points in the trajectory
  int N_points = 150;
  
  //!Number of points removed at the end
  int N_end_points_removed = 100;
  
  //! Starting point
  Point start_point;
  double start_yaw;
  
  //! Goal point
  Point goal_point;
  
  //! Reference speed (defines the space between points)
  double reference_speed = 49.5 * 0.44704; // [m/s]
  
  //! Reference acceleration
  double reference_accel = 0; // [m/s^2]
  
  //! Generated trajectory
  Trajectory trajectory;
  
  //! Previous generated trajectory (only points which haven't been passed)
  Trajectory previous_path;
  
  //! Previous car position
  Point previous_end_point;
  
  //! Number of points already passed
  int N_points_passed = 0;

  //! Generates the trajectory
  virtual void GenerateTrajectory() = 0;
  
  // DEBUG
  tk::spline debug_spline;
};

#endif