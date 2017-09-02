#ifndef KINEMATIC_STATE_H
#define KINEMATIC_STATE_H

#include "trajectory.h"

struct Velocity {
  double vx;
  double vy;
  double magnitude;
};

class KinematicState {
public:
  Point position;
  Velocity velocity;
  double yaw;
  double accel;
};

#endif