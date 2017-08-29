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
#include "lane.h"

using namespace std;

class Road {
public:

  vector<Lane> lanes;
  map<int, Vehicle> vehicles;
  Vehicle ego;
  // int update_width = 70;
  // string ego_rep = " *** ";
  // int ego_key = -1;
  // int num_lanes;
  // vector<int> lane_speeds;
  // int speed_limit;
  // double density;
  // int camera_center;
  // int vehicles_added = 0;

  /**
   * Default Constructor
   */
  Road() {};
  
  /**
   * Constructor with lanes: Builds the specified number of lanes at initialization.
   * @param width Width of each lane
   * @param lane_speeds Speed limit for each lane, starting from the right lane
   */
  Road(double width, vector<int> lane_speeds);
  // Road(int speed_limit, double traffic_density, vector<int> lane_speeds);

  /**
  * Destructor
  */
  virtual ~Road();

  void populate_traffic();
  // Vehicle get_ego();
  // void advance();
  // void display(int timestep);
  // void add_ego(int lane_num, int s, vector<int> config_data);
  // void cull();
};

#endif