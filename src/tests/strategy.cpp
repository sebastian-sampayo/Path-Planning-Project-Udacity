#include <iostream>
#include <vector>

#include "../Eigen-3.3/Eigen/Core"
#include "../Eigen-3.3/Eigen/QR"
#include "../json.hpp"

#include "../kinematic_state.h"
#include "../logger.h"
#include "../straight_line_strategy.h"
#include "../spline_strategy.h"
#include "../trajectory.h"
#include "../trajectory_strategy.h"
#include "../walkthrough_strategy.h"

#include "stub.h"

// for convenience
using namespace std;
using json = nlohmann::json;

int main()
{
  SET_LOG_LEVEL(logDEBUG3);
  LOG(logINFO) << "----- Test Strategy -----";

  TrajectoryStrategy *strategy;

  // strategy = new StraightLineStrategy();
  // strategy = new WalkthroughStrategy();
  strategy = new SplineStrategy();

  // Set starting point
  // ...

  // Set goal
  // ...

  // Generate trajectory
  strategy->GenerateTrajectory();

  // Print
  cout << "Generated Trajectory: " << endl << strategy->trajectory << endl;

  return 0;
}