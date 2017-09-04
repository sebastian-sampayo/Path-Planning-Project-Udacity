#ifndef WALKTHROUGH_STRATEGY_H
#define WALKTHROUGH_STRATEGY_H

#include "trajectory_strategy.h"

class WalkthroughStrategy : public TrajectoryStrategy {
public:
  void GenerateTrajectory();

  WalkthroughStrategy();
};

#endif