#ifndef KINEMATIC_STATE_H
#define KINEMATIC_STATE_H

#include "point.h"

class KinematicState {
public:
  // Constructors and destructors
  KinematicState() {};
  virtual ~KinematicState() {};

  Point position;
  double yaw;
  double speed;
  double accel;
};

#endif
