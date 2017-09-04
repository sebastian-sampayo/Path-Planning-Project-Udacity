#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <iostream>
#include <vector>

#include "point.h"

using namespace std;

class Trajectory {
public:
  vector<Point> points;
  
  vector<double> GetXvalues();
  vector<double> GetYvalues();
  
  //! Overload << to print the trajectory
  friend ostream& operator<<(ostream& os, const Trajectory& t);
  
  size_t size() { return points.size();};
  size_t push_back(Point& p) { return points.push_back(p);};
};

#endif
