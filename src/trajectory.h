#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <iostream>
#include <vector>

#include "point.h"

using namespace std;

class Trajectory : public vector<Point> {
public:
  vector<double> GetXvalues();
  vector<double> GetYvalues();
  vector<double> GetSvalues();
  vector<double> GetDvalues();

  //! Overload << to print the trajectory
  friend ostream& operator<<(ostream& os, const Trajectory& t);

private:
  vector<double> GetValues(double (Point::*getValue)() const);
};

#endif
