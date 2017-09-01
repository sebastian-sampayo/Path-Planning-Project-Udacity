#include <vector>

#include "point.h"
#include "straight_line_strategy.h"

using namespace std;

void StraightLineStrategy::GenerateTrajectory()
{
  vector<double> next_x_vals;
  vector<double> next_y_vals;
  
  double car_x = start.x;
  double car_y = start.y;
  // double car_yaw = ego_data.yaw;
  
  // Straight line test
  double dist_inc = 0.5;
  for(int i = 0; i < 50; i++)
  {
    // next_x_vals.push_back( car_x + (dist_inc*i) * cos(deg2rad(car_yaw)) );
    // next_y_vals.push_back( car_y + (dist_inc*i) * sin(deg2rad(car_yaw)) );
    Point p;
    // p.x = car_x + (dist_inc*i) * cos(deg2rad(car_yaw));
    // p.y = car_y + (dist_inc*i) * sin(deg2rad(car_yaw));
    // next_path.points.push_back(p);
  }
}
