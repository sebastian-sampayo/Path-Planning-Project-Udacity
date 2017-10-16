#ifndef SPLINE_STRATEGY_H
#define SPLINE_STRATEGY_H

#include "trajectory_strategy.h"

class SplineStrategy : public TrajectoryStrategy {
public:
  void GenerateTrajectory();

  SplineStrategy();
private:
  void GenerateXYTrajectory();
  void GenerateSDTrajectory();
};

#endif