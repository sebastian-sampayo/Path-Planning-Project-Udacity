#include "point.h"
#include "road.h"
#include "vehicle.h"

#include "logger.h"
#include "utils.h"

#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>

// ----------------------------------------------------------------------------
// PUBLIC
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Initializes Vehicle
Vehicle::Vehicle()
{
  LOG(logDEBUG4) << "Vehicle::Vehicle()";
  speed = 0;
  yaw = 0;
  
  road_ptr = NULL;
}

// ----------------------------------------------------------------------------
Vehicle::Vehicle(const EnvironmentSensorData::SensedVehicleData& data, Road* road)
{
  this->road_ptr = road;
  UpdateSensorData(data);
}

// ----------------------------------------------------------------------------
Vehicle::~Vehicle() {}

// ----------------------------------------------------------------------------
void Vehicle::Move(double delta_t)
{
  position = PredictPosition(delta_t);
}

// ----------------------------------------------------------------------------
Point Vehicle::PredictPosition(double delta_t) const
{
  // The idea here is to assume constant velocity and constant yaw
  // However, the yaw must be transformed into Frenet coordinate system
  // so that the vehicle follow a line in that system.
  // Algorithm:
  //  Let pc0 be the current position of the vehicle in cartesian coordinates
  //  and pf0 the same position in frenet. At the same time, let vc be the
  //  velocity in cartesian coordinates. Then let pc1 be a point such:
  //  pc1 = pc0 + normalize(vc) * 0.001
  //  pf1 = Frenet(pc1)
  //  Then we can calculate the angle in Frenet coordinates as the orientation
  //  of the difference vector: 
  //    vf = pf1 - pf0:
  //  So,
  //    yaw_frenet = atan2(vf.d, vf.s);
  
  // // Current position in Cartesian coordinates
  // const double pc0x = this->position.GetX();
  // const double pc0y = this->position.GetY();
  // // Current velocity
  // const double vcx = speed * cos(yaw);
  // const double vcy = speed * sin(yaw);
  // // Scaled velocity
  // const double ds = 0.001;
  // const double vcsx = ds * vcx/speed;
  // const double vcsy = ds * vcy/speed;
  // // Aux position in Cartesian coordinates
  // const double pc1x = pc0x + vcsx;
  // const double pc1y = pc0y + vcsy;
  // // Current position in Frenet coordinates
  // const double pf0s = this->position.GetS();
  // const double pf0d = this->position.GetD();
  // // Aux position in Frenet coordinates
  // const PointFrenet pf1 = PointCartesian(pc1x, pc1y);
  // const double pf1s = pf1.s;
  // const double pf1d = pf1.d;
  
  // // Orientation in Frenet system
  // const double yaw_frenet = atan2(pf1d - pf0d, pf1s - pf0s);
  
  // // Prediction in Frenet system
  // const double vfs = speed * cos(yaw_frenet);
  // const double vfd = speed * sin(yaw_frenet);
  // const double future_s = pf0s + vfs * delta_t;
  // const double future_d = pf0d + vfd * delta_t;
  
  // Algorithm 2: Assume yaw_frenet = 0, so the vehicle will stay in the same lane (constant d-coordinate).
  const double s = this->position.GetS();
  const double d = this->position.GetD();
  const double future_s = s + speed * delta_t;
  
  return PointFrenet(future_s, d);
}

// ----------------------------------------------------------------------------
void Vehicle::Translate(const Point& new_pos)
{
  double dy = new_pos.GetY() - position.GetY();
  double dx = new_pos.GetX() - position.GetX();
  yaw = atan2(dy, dx);
  position = new_pos;
}

// ----------------------------------------------------------------------------
void Vehicle::UpdateSensorData(const EnvironmentSensorData::SensedVehicleData& data)
{
  PointCartesian pc(data.x, data.y);
  // PointFrenet pf(data.s, data.d); // Let's ignore frenet coordinates. I trust cartesian!
  // Point p(pc, pf);
  Point p(pc);
  position = p;
  speed = Magnitude(data.vx, data.vy); // [m/s]
  
  yaw = atan3(data.vy, data.vx);
  
  // Update current Lane
  lane = int(p.GetD() / road_ptr->LANE_WIDTH);
}
