#include "kinematic_state.h"
#include "position.h"

// ----------------------------------------------------------------------------
// PUBLIC
// ----------------------------------------------------------------------------

// Assignment operator
KinematicState& KinematicState::operator=(const KinematicState& other)
{
  if (&other != this)
  {
    this->position = other.position;
    this->speed = other.speed;
    this->accel = other.accel;
  }

  return *this;
}