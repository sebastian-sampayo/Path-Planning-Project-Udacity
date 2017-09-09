#ifndef TRAJECTORY_STRATEGY_H
#define TRAJECTORY_STRATEGY_H

#include "kinematic_state.h"
#include "point.h"
#include "road.h"
#include "trajectory.h"

class TrajectoryStrategy {
public:
  //! Number of points in the trajectory
  int N_points = 50;
  
  //! Starting point
  KinematicState start;
  
  //! Goal point
  Point goal;
  
  //! Reference speed (defines the space between points)
  double reference_speed = 40; // mph
  
  //! Generated trajectory
  Trajectory trajectory;
  
  //! Previous generated trajectory (only points which haven't been passed)
  Trajectory previous_path;
  
  //! Previous car position
  Point previous_end_point;

  //! Generates the trajectory
  virtual void GenerateTrajectory() = 0;
};

#endif