// Singleton

#ifndef MAP_H
#define MAP_H

#include <vector>
#include <string>

#include "logger.h"
#include "spline.h"

using namespace std;

  
class Map {
public:
  //! Name of the waypoints file
  static constexpr const char* MAP_FILENAME = "../data/highway_map.csv";
  
  //! Waypoints of the center line of the road in the global map coordinates
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  static Map& GetInstance();

  //! Convert back and forth between Frenet and Cartesian coordinates
  void ToFrenet(double x, double y, double &s, double &d);
  void ToCartesian(double s, double d, double &x, double &y);
  
  //! Load the waypoints from file
  void SetWaypoints(string map_filename);
  
  // C++11: delete explicitly copy ctor and assignment operator
  Map(Map const&)               = delete;
  void operator=(Map const&)    = delete;

private:
  // Splines for the components of the parametric curve q(s) = (qx(s), qy(s))
  tk::spline qx_s_;
  tk::spline qy_s_;
  
  Map();
};

#endif
