#ifndef KINEMATIC_STATE_H
#define KINEMATIC_STATE_H

#include "position.h"

class KinematicState {
public:
  // Constructors and destructors
  KinematicState() {};
  virtual ~KinematicState() {};

  Position position;
  double speed;
  double accel;
};

#endif