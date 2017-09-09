#ifndef KINEMATIC_STATE_H
#define KINEMATIC_STATE_H

#include "point.h"
#include "sensor_data.h"

class KinematicState {
public:
  // Constructors and destructors
  //! Default Constructor
  KinematicState();

  //! Converts from SensedVehicleData (x,y,s,d,vx,vy). Doesn't set accel. Assumes the heading is in the same direction as the velocity: yaw = atan2(vy,xy).
  KinematicState(const EnvironmentSensorData::SensedVehicleData& data);
  KinematicState(const EgoSensorData& data);
  virtual ~KinematicState() {};

  Point position;
  double yaw;
  double speed;
  double accel;
};

#endif
