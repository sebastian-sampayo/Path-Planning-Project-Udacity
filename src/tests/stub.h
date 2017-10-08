#include <vector>

#include "../json.hpp"
#include "../trajectory.h"

// for convenience
using namespace std;
using json = nlohmann::json;

json stub_ego_data = {
  {"x", 784.44},
  {"y", 1129.57},
  {"s", 0},
  {"d", 6},
  {"yaw", 0},
  {"speed", 50}
};

// Be carefull: Cartesian and Frenet coordinates not related!
// id, x, y, vx, vy, s, d
json stub_environment_data = json::array({
  {0, 0, 0, 50, 6, 0, 0},
  {1, 1, 1, 100, 2, 0, 0},
  {8, 8, 8, 0, 0, 0, 0},
  {5, 5, 5, 0, 0, 0, 0}
});

json stub_environment_data2 = json::array({
  {0, 0, 0, 0, 0, 0, 0},
  {1, 1, 2, 0, 0, 0, 0},
  {5, 5, 6, 0, 0, 0, 0}
});

json stub_environment_cost_function = json::array({
  // {0, 784.44, 1129.57, 0, 0, 0, 6},
  {1, 804.479, 1129.09, 0, 0, 20, 6},
  {2, 834.597, 1132.9, 0, 0, 50, 2},
  {3, 789.336, 1125.44, 10, 0, 5, 10},
  // {4, 789.441, 1129.44, 0, 0, 5, 6},
});