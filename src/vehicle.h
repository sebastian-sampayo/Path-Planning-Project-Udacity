#ifndef VEHICLE_H
#define VEHICLE_H

#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>

using namespace std;

class Vehicle {
public:

  // struct collider{

    // bool collision ; // is there a collision?
    // int  time; // time collision happens

  // };

  double L = 1;
  // int preferred_buffer = 6; // impacts "keep lane" behavior.
  double x;
  double y;
  double s;
  double d;
  double v;
  double a;
  int lane;
  int lanes_available;
  double target_speed;
  // int max_acceleration;
  int goal_lane;
  double goal_s;
  // string state;

  /**
  * Constructor
  */
  Vehicle() {};
  Vehicle(int lane, double s, double v, double a);

  /**
  * Destructor
  */
  virtual ~Vehicle();
  
  // void predict(double deltaT);

  // void update_state(map<int, vector <vector<int> > > predictions);
  // void configure(vector<int> road_data);
  // string display();
  // void increment(int dt);
  // vector<int> state_at(int t);
  // bool collides_with(Vehicle other, int at_time);
  // collider will_collide_with(Vehicle other, int timesteps);
  // void realize_state(map<int, vector < vector<int> > > predictions);
  // void realize_constant_speed();
  // int _max_accel_for_lane(map<int,vector<vector<int> > > predictions, int lane, int s);
  // void realize_keep_lane(map<int, vector< vector<int> > > predictions);
  // void realize_lane_change(map<int,vector< vector<int> > > predictions, string direction);
  // void realize_prep_lane_change(map<int,vector< vector<int> > > predictions, string direction);
  // vector<vector<int> > generate_predictions(int horizon);
};

#endif