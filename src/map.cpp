#include <vector>
#include <fstream>      // std::ifstream
#include <sstream>      // std::istringstream
#include <string>       // std::string

#include "map.h"
#include "utils.h"
#include "spline.h"

using namespace std;

// ----------------------------------------------------------------------------
// PUBLIC
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
void Map::ToFrenet(double x, double y, double& s, double& d)
{
  // Let p = (x,y) be the point in cartesian coordinates, and q = (qx(s), qy(s))
  // the parametric formula of the curve.
  // Also, let s* be the parameter for which q(s*) is closest to p, then the
  // following condition must be satisfied:
  // (p - q(s*)) q'(s*) = 0  (1)
  // which means that the distance between p and q(s*) is perpendicular to q'(s*)
  // where q'(s*) is actually the tangent vector of the curve at point s*.
  // Let theta be the curve angle at point s, then
  // q'(s) = (cos(theta), sin(theta)) (2)
  // The condition (1) could have multiple roots in a global domain, but only one is
  // the closest point to p. 
  // However, it can be shown that if s* is unique and we constrain to a local domain around
  // the closest waypoint to p, then (1) is sufficient to find a unique s*.
  
  // Start at the closest waypoint
  const int wp = ClosestWaypoint(x,y, map_waypoints_x,map_waypoints_y);
  s = map_waypoints_s[wp];
  
  // Define an increment size to calculate theta at some point s
  const double tolerance = 1e-5;
  const double ds = tolerance;
  
  // This variables will contain: p - q(s*)
  double pq_dist_x = -1;
  double pq_dist_y = -1;
  // This variables will contain q'(s*)
  double rx = -1;
  double ry = -1;
  
  double p_proj_r = 0;
  
  do
  {
    // Gradient-descent search
    s = CycleS(s+p_proj_r);
    const double s1 = CycleS(s+ds);
    const double qx0 = qx_s_(s);
    const double qy0 = qy_s_(s);
    const double qx1 = qx_s_(s1); // Bug here! (Vehicle stopped)
    const double qy1 = qy_s_(s1);
    const double theta = atan2(qy1-qy0, qx1-qx0);
    // Let r(s) = q'(s) = (rx(s), ry(s))
    rx = cos(theta);
    ry = sin(theta);
    // p - q(s)
    pq_dist_x = x - qx0;
    pq_dist_y = y - qy0;
    // (p - q) q'. This must be 0 for orthogonality
    p_proj_r = pq_dist_x*rx + pq_dist_y*ry;
  } while (fabs(p_proj_r) > tolerance);
  
  d = pq_dist_x*ry - pq_dist_y*rx;
}

// ----------------------------------------------------------------------------
// Transform from Frenet s,d coordinates to Cartesian x,y
void Map::ToCartesian(double s_in, double d, double& x, double& y)
{
  double s = CycleS(s_in);
  // p = q(s) + d
  x = qx_s_(s);
  y = qy_s_(s);
  
  // Algorithm 1 (has more error than Algorithm 2)
  // const double dx = d * dx_s_(s);
  // const double dy = d * dy_s_(s);
  
  // x += dx;
  // y += dy;
  
  // Algorithm 2
  double ds = 1e-5;
  double s1 = CycleS(s+ds);
  double dx = qx_s_(s1)-x;
  double dy = qy_s_(s1)-y;

  double theta = atan3(dy, dx);
  
  x += d*sin(theta);
  y -= d*cos(theta);
}

// ----------------------------------------------------------------------------
void Map::SetWaypoints(string map_filename)
{
  LOG(logDEBUG4) << "Map::SetWaypoints()";
  // TODO: clear waypoints

  ifstream in_map_(map_filename.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  
  qx_s_.set_points(map_waypoints_s, map_waypoints_x);
  qy_s_.set_points(map_waypoints_s, map_waypoints_y);
  dx_s_.set_points(map_waypoints_s, map_waypoints_dx);
  dy_s_.set_points(map_waypoints_s, map_waypoints_dy);
  
  LOG(logDEBUG4) << "Map::SetWaypoints() - map_waypoints_s.size() = " << map_waypoints_s.size();
}

// ----------------------------------------------------------------------------
Map& Map::GetInstance()
{
  LOG(logDEBUG5) << "Map& Map::GetInstance()";
  static Map instance;
  
  return instance;
}

// ----------------------------------------------------------------------------
// PRIVATE
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
Map::Map()
{
  LOG(logDEBUG4) << "Map::Map()";
  
  SetWaypoints(MAP_FILENAME);
}
