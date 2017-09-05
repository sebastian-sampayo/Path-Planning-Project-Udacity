#ifndef TRAJECTORY_STRATEGY_H
#define TRAJECTORY_STRATEGY_H

#include "kinematic_state.h"
#include "point.h"
#include "road.h"
#include "trajectory.h"

class TrajectoryStrategy {
public:
  KinematicState start;
  Point goal;
  Trajectory trajectory;
  Trajectory previous_path;
  Point previous_end_point;

  virtual void GenerateTrajectory() = 0;
};

#endif