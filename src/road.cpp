#include <iostream>
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>

#include "logger.h"
#include "road.h"
#include "vehicle.h"

using namespace std;

// ----------------------------------------------------------------------------
// PUBLIC
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Initializes Road
Road::Road(double width, vector<int> lane_speeds)
{
  LOG(logDEBUG4) << "Road::Road()";

  for (const int speed_limit : lane_speeds)
  {
    Lane lane;
    lane.width = width;
    lane.speed_limit = speed_limit;
    lanes.push_back(lane);
  }
}

// ----------------------------------------------------------------------------
Road::~Road() {}

// ----------------------------------------------------------------------------
void Road::UpdateEgoKinematics(EgoSensorData data)
{
  ego.kinematic_state.position.SetXY(data.x, data.y);
  ego.kinematic_state.position.SetFrenet(data.s, data.d);
  ego.kinematic_state.position.SetYaw(data.yaw);
  ego.kinematic_state.speed = data.speed;
}

// ----------------------------------------------------------------------------
void Road::populate_traffic() {

}
