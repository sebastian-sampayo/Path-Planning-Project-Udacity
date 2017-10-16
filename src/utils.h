#ifndef UTILS_H
#define UTILS_H

#include <chrono>
#include <math.h>
#include <vector>

using namespace std;

// Tolerance value for atan3()
constexpr double ATAN3_TOL = 1e-5;

// For converting back and forth between radians and degrees.
inline constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

//! Wraps atan2() but guards against low values
double atan3(double y, double x);

//! Calculates the Euclidean distance between (x1,y1) and (x2,y2)
double distance(double x1, double y1, double x2, double y2);

//! Calculates the magnitude of the vector (x,y)
double Magnitude(double x, double y);

//! Gets the closest waypoint to the specified point x,y
int ClosestWaypoint(double x, double y, const vector<double>& maps_x, const vector<double>& maps_y);

//! Gets the next waypoint for the specified x,y
int NextWaypoint(double x, double y, double theta
  , const vector<double>& maps_x, const vector<double>& maps_y);
  
//! A helper function that takes in a c-style array and convert it to a c++ vector
template<class Tin, class Tout>
vector<Tout> CArrayToVector(const Tin* c_array, const int length)
{
  vector<Tout> output_vector;
  for (int i = 0; i < length; ++i)
  {
    output_vector.push_back(Tout(c_array[i]));
  }
  
  return output_vector;
}

//! A helper class to measure elapsed time for debugging and optimization purposes
class Timer
{
  public:
    typedef std::chrono::high_resolution_clock Clock;
    typedef std::chrono::time_point<Clock> ClockMeasure;
    
    Timer() : begin_(Clock::now()) {};
    ~Timer() {};
  
    double GetElapsedSeconds() {
      ClockMeasure end = Clock::now();
      std::chrono::duration<double> diff = end - begin_;
      return diff.count();
    };
    
    double GetElapsedMiliSeconds() {
      return 1000*GetElapsedSeconds();
    };
  
    void Reset() {begin_ = Clock::now();};
  
  private:
    ClockMeasure begin_; // Begin time of the timer
};

#endif
