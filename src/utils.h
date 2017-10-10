#ifndef UTILS_H
#define UTILS_H

#include <ctime>
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

int ClosestWaypoint(double x, double y, const vector<double>& maps_x, const vector<double>& maps_y);

int NextWaypoint(double x, double y, double theta
  , const vector<double>& maps_x, const vector<double>& maps_y);
  
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

class Timer
{
  public:
    Timer() : begin_(clock()) {};
    ~Timer() {};
  
    double GetElapsedSeconds() {
      const double end = clock();
      return double(end - begin_) / CLOCKS_PER_SEC;
    };
    
    double GetElapsedMiliSeconds() {
      return 1000*GetElapsedSeconds();
    };
  
    void Reset() {begin_ = clock();};
  
  private:
    double begin_; // Begin time of the timer
};

#endif
