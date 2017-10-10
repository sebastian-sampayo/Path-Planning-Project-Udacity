#include "stub.h"

#include "../map.h"
#include "../point.h"
#include "../straight_line_strategy.h"
#include "../spline_strategy.h"
#include "../trajectory.h"
#include "../trajectory_strategy.h"
#include "../walkthrough_strategy.h"

#include "../Eigen-3.3/Eigen/Core"
#include "../Eigen-3.3/Eigen/QR"
#include "../json.hpp"
#include "../logger.h"
#include "../matplotlibcpp.h"

#include <iostream>
#include <vector>

// for convenience
using namespace std;
using json = nlohmann::json;
namespace plt = matplotlibcpp;

void PlotTrajectory(Trajectory trajectory, const char* filename)
{
  plt::subplot(1, 2, 1);
  plt::title("Trajectory in Cartesian Coordinates");
  plt::plot(trajectory.GetXvalues(), trajectory.GetYvalues());
  plt::xlabel("x");
  plt::ylabel("y");
  plt::subplot(1, 2, 2);
  plt::title("Trajectory in Frenet Coordinates");
  plt::plot(trajectory.GetDvalues(), trajectory.GetSvalues());
  plt::xlabel("d");
  plt::ylabel("s");
  // plt::axis("equal");
  // plt::xlim(0, 2500);
  // plt::plot(Map::, maps_y_, "r."
           // , xs, ys, "k-");
  plt::save(filename);
}

int main()
{
  SET_LOG_LEVEL(logDEBUG3);
  LOG(logINFO) << "----- Test Strategy -----";

  TrajectoryStrategy *strategy;

  // strategy = new StraightLineStrategy();
  // strategy = new WalkthroughStrategy();
  strategy = new SplineStrategy();

  // Set starting point
  strategy->start_point = PointFrenet(0, 6);

  // Set goal
  const double goal_s = 30;
  strategy->goal_point = PointFrenet(goal_s, 2);

  // Generate trajectory
  strategy->GenerateTrajectory();
  
  // Simulate second cycle
  // strategy->trajectory.erase(strategy->trajectory.begin(), strategy->trajectory.begin()+3);
  strategy->previous_path = strategy->trajectory;
  strategy->GenerateTrajectory();

  // Print
  cout << "Generated Trajectory: " << endl << strategy->trajectory << endl;

  // Plot
  PlotTrajectory(strategy->trajectory, "../img/strategy_test_trajectory.png");

  // DEBUG spline
  const double MAX_S = Map::GetInstance().MAX_S;
  Trajectory debug;
  
  // Frenet spline
  for (int s = -goal_s; s < goal_s*2; ++s)
  {
    double d = strategy->debug_spline(s);
    debug.push_back(PointFrenet(s, d));
  }

  // Plot
  PlotTrajectory(debug, "../img/strategy_test_debug.png");

  return 0;
}