#ifndef KINEMATIC_STATE_H
#define KINEMATIC_STATE_H

#include "point.h"

class KinematicState {
public:
  Point position;
  double speed;
  double yaw;
  double accel;
};

#endif