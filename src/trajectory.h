#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <iostream>
#include <vector>

#include "position.h"

using namespace std;

class Trajectory {
public:
  vector<Position> points;
  
  vector<double> GetXvalues();
  vector<double> GetYvalues();
  
  //! Overload << to print the trajectory
  friend ostream& operator<<(ostream& os, const Trajectory& t);
  
  size_t size() { return points.size();};
};

#endif