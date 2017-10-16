#ifndef STRAIGHT_LINE_STRATEGY_H
#define STRAIGHT_LINE_STRATEGY_H

#include "trajectory_strategy.h"

class StraightLineStrategy : public TrajectoryStrategy {
public:
  void GenerateTrajectory();

  StraightLineStrategy();
  
private:
  void GenerateXYTrajectory();
  void GenerateSDTrajectory();
  void GenerateSDTrajectory2();
};

#endif