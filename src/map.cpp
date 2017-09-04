#include <vector>
#include <fstream>      // std::ifstream
// #include <iostream>     // std::cout
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
void Map::GetFrenet(double x, double y, double theta, double& s, double& d)
{
  int next_wp = NextWaypoint(x,y, theta, map_waypoints_x,map_waypoints_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
    prev_wp  = map_waypoints_x.size()-1;
  }

  double n_x = map_waypoints_x[next_wp]-map_waypoints_x[prev_wp];
  double n_y = map_waypoints_y[next_wp]-map_waypoints_y[prev_wp];
  double x_x = x - map_waypoints_x[prev_wp];
  double x_y = y - map_waypoints_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-map_waypoints_x[prev_wp];
  double center_y = 2000-map_waypoints_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(map_waypoints_x[i],map_waypoints_y[i],map_waypoints_x[i+1],map_waypoints_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  s = frenet_s;
  d = frenet_d;
}

// ----------------------------------------------------------------------------
// Transform from Frenet s,d coordinates to Cartesian x,y
void Map::GetXY(double s, double d, double& x, double& y)
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
  
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

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