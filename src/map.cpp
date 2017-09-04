#include <vector>
#include <fstream>      // std::ifstream
#include <sstream>      // std::istringstream
#include <string>       // std::string

#include "map.h"
#include "utils.h"

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
  // q'(s) = (cos(theta), sin(theta))
  // The condition (1) could have multiple roots in a global domain, but only one is
  // the closest point to p. 
  // However, it can be shown that if we constrain to a local domain around
  // the closest waypoint to p, then (1) is sufficient to find a unique s*.
  
  // Start at the closest waypoint
  int wp = ClosestWaypoint(x,y, map_waypoints_x,map_waypoints_y);
  double s = map_waypoints_s[wp];
  double qx0 = map_waypoints_x[wp];
  double qy0 = map_waypoints_y[wp];
  
  // Define an increment size to calculate theta at some point s
  double ds = 0.001;
  
  // This variables will contain: p - q(s)
  double pq_dist_x = -1;
  double pq_dist_y = -1;
  
  double p_proj_r = 0;
  
  do
  {
    // Gradient-descent search
    s = CycleS(s+p_proj_r);
    s1 = CycleS(s+ds);
    double qx0 = qx_s_(s);
    double qy0 = qy_s_(s);
    double qx1 = qx_s_(s1);
    double qy1 = qy_s_(s1);
    double theta = atan2(qy1-qy0, qx1-qx0);
    // Let r(s) = q'(s) = (rx(s), ry(s))
    double rx = cos(theta);
    double ry = sin(theta);
    // p - q(s)
    pq_dist_x = x - qx0;
    pq_dist_y = y - qy0;
    // (p - q) q'. This must be 0 for orthogonality
    p_proj_r = pq_dist_x*rx + pq_dist_y*ry;
  } while (p_proj_r > 0.01)
  
  d = pq_dist_x*ry - pq_dist_y*rx;
}

// ----------------------------------------------------------------------------
// Transform from Frenet s,d coordinates to Cartesian x,y
void Map::ToCartesian(double s, double d, double& x, double& y)
{
  LOG(logDEBUG4) << "Map::GetXY()";
  int prev_wp = -1;
  
  double a = GetInstance().map_waypoints_s[0];

  while(s > map_waypoints_s[prev_wp+1] && (prev_wp < (int)(map_waypoints_s.size()-1) ))
  {
    prev_wp++;
  }
  LOG(logDEBUG4) << "Map::GetXY() - found prev_wp = " << prev_wp;

  int wp2 = (prev_wp+1)%map_waypoints_x.size();

  double heading = atan2((map_waypoints_y[wp2]-map_waypoints_y[prev_wp]),(map_waypoints_x[wp2]-map_waypoints_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-map_waypoints_s[prev_wp]);

  double seg_x = map_waypoints_x[prev_wp]+seg_s*cos(heading);
  double seg_y = map_waypoints_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  x = seg_x + d*cos(perp_heading);
  y = seg_y + d*sin(perp_heading);
  
  LOG(logDEBUG4) << "Map::GetXY() - end";
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
  
  LOG(logDEBUG4) << "Map::SetWaypoints() - map_waypoints_s.size() = " << map_waypoints_s.size();
}
// ----------------------------------------------------------------------------
// PRIVATE
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
Map& Map::GetInstance()
{
  LOG(logDEBUG4) << "Map& Map::GetInstance()";
  static Map instance;
  
  return instance;
}

// ----------------------------------------------------------------------------
Map::Map()
{
  LOG(logDEBUG4) << "Map::Map()";
  
  SetWaypoints(MAP_FILENAME);
}
