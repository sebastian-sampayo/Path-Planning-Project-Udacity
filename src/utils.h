#ifndef UTILS_H
#define UTILS_H

#include <math.h>
#include <vector>

using namespace std;

// For converting back and forth between radians and degrees.
inline constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

//! Calculates the Euclidean distance between (x1,y1) and (x2,y2)
double distance(double x1, double y1, double x2, double y2);

//! Calculates the magnitude of the vector (x,y)
double Magnitude(double x, double y);

int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y);

int NextWaypoint(double x, double y, double theta
  , vector<double> maps_x, vector<double> maps_y);

#endif