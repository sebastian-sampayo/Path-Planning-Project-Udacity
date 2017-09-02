#ifndef ROAD_H
#define ROAD_H

#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <set>
#include <map>
#include <string>
#include <iterator>

#include "vehicle.h"

using namespace std;

struct Lane {
  double width;
  int speed_limit;
};

class Road {
public:

  vector<Lane> lanes;
  map<int, Vehicle> vehicles;
  Vehicle ego;

  // Default Constructor
  Road() {};
  
  // Constructor with lanes: Builds the specified number of lanes at initialization.
  // @param width Width of each lane
  // @param lane_speeds Speed limit for each lane, starting from the right lane
  Road(double width, vector<int> lane_speeds);
  // Road(int speed_limit, double traffic_density, vector<int> lane_speeds);

  // Destructor
  virtual ~Road();

  void populate_traffic();
};

#endif