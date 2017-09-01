#ifndef STRAIGHT_LINE_STRATEGY_H
#define STRAIGHT_LINE_STRATEGY_H

#include "trajectory_strategy.h"

class StraightLineStrategy : public TrajectoryStrategy {
public:
  StraightLineStrategy() {};

  void GenerateTrajectory();
};

#endif