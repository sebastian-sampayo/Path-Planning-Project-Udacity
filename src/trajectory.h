#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <iostream>
#include <vector>

using namespace std;

struct Point {
  double x;
  double y;
  double s;
  double d;
};

class Trajectory {
public:
  vector<Point> points;
  
  vector<double> GetXvalues();
  vector<double> GetYvalues();
  
  //! Overload << to print the trajectory
  friend ostream& operator<<(ostream& os, const Trajectory& t); 
};

#endif