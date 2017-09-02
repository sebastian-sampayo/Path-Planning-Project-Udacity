#include <iostream>
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>

#include "road.h"
#include "vehicle.h"

using namespace std;

// Initializes Road
Road::Road(double width, vector<int> lane_speeds)
{
  for (const int speed_limit : lane_speeds)
  {
    Lane lane;
    lane.width = width;
    lane.speed_limit = speed_limit;
    lanes.push_back(lane);
  }
}

Road::~Road() {}

void Road::populate_traffic() {

}

