#include <iostream>
#include <vector>

#include "path_planner.h"
#include "point.h"
#include "road.h"
#include "utils.h"
#include "straight_line_strategy.h"
#include "trajectory.h"
#include "vehicle.h"

using namespace std;

// Static attributes
constexpr int PathPlanner::SPEED_LIMITS[];

PathPlanner::PathPlanner()
{
  strategy = new StraightLineStrategy();
}

PathPlanner::~PathPlanner()
{
  delete strategy;
}

Trajectory PathPlanner::Generate()
{
  strategy->GenerateTrajectory();
  return strategy->trajectory;
}

// void PathPlanner::GeneratePath()
// {
  // Vehicle ego();
  
  // // Convert from int[] to vector<int>
  // // TODO: encapsulate in a function
  // vector<int> speed_limits;
  // for (int i = 0; i < NUM_LANES; ++i)
  // {
    // speed_limits.push_back(SPEED_LIMITS[i]);
  // }
  // Road road(LANE_WIDTH, speed_limits);
  
// }

// void PathPlanner::GenerateStraightLine()
// {
  // vector<double> next_x_vals;
  // vector<double> next_y_vals;
  
  // double car_x = ego_data.position.x;
  // double car_y = ego_data.position.y;
  // double car_yaw = ego_data.yaw;
  
  // // Straight line test
  // double dist_inc = 0.5;
  // for(int i = 0; i < 50; i++)
  // {
    // // next_x_vals.push_back( car_x + (dist_inc*i) * cos(deg2rad(car_yaw)) );
    // // next_y_vals.push_back( car_y + (dist_inc*i) * sin(deg2rad(car_yaw)) );
    // Point p;
    // p.x = car_x + (dist_inc*i) * cos(deg2rad(car_yaw));
    // p.y = car_y + (dist_inc*i) * sin(deg2rad(car_yaw));
    // next_path.points.push_back(p);
  // }
// }

// void PathPlanner::GenerateCircle()
// {
  // double pos_x;
  // double pos_y;
  // double angle;
  // int path_size = previous_path_x.size();

  // for(int i = 0; i < path_size; i++)
  // {
      // next_x_vals.push_back(previous_path_x[i]);
      // next_y_vals.push_back(previous_path_y[i]);
  // }

  // if(path_size == 0)
  // {
      // pos_x = car_x;
      // pos_y = car_y;
      // angle = deg2rad(car_yaw);
  // }
  // else
  // {
      // pos_x = previous_path_x[path_size-1];
      // pos_y = previous_path_y[path_size-1];

      // double pos_x2 = previous_path_x[path_size-2];
      // double pos_y2 = previous_path_y[path_size-2];
      // angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
  // }

  // double dist_inc = 0.5;
  // for(int i = 0; i < 50-path_size; i++)
  // {
      // next_x_vals.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(pi()/100)));
      // next_y_vals.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(pi()/100)));
      // pos_x += (dist_inc)*cos(angle+(i+1)*(pi()/100));
      // pos_y += (dist_inc)*sin(angle+(i+1)*(pi()/100));
  // }
// }

void PathPlanner::SetPreviousPath(json previous_path_x, json previous_path_y)
{
  int path_size = previous_path_x.size();
  for (int i = 0; i < path_size; ++i)
  {
    Point p;
    p.x = previous_path_x[i];
    p.y = previous_path_y[i];
    strategy->previous_path.points.push_back(p);
  }
}

void PathPlanner::SetPreviousEndPoint(double end_path_s, double end_path_d)
{
  strategy->previous_end_point.s = end_path_s;
  strategy->previous_end_point.d = end_path_d;
}