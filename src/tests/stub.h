#include <vector>

#include "../json.hpp"
#include "../trajectory.h"
#include "../utils.h"

// for convenience
using namespace std;
using json = nlohmann::json;

const double MPH2MPS = 0.44704; // TODO: Move this to a config file

// speed in mph
// json stub_ego_data = {
  // {"x", 784.44},
  // {"y", 1129.57},
  // {"s", 0},
  // {"d", 6},
  // {"yaw", 0},
  // {"speed", 30}
// };

// json stub_ego_data = {
  // {"x", 2049.33},
  // {"y", 1264.28},
  // {"s", 1316.89},
  // {"d", 5.10218},
  // {"yaw", 1.22779 * 180 / pi()},
  // {"speed", 17.5879 / MPH2MPS}
// };

json stub_ego_data = {
  {"x", 192.511},
  {"y", 2465.14},
  {"s", 5368.95},
  {"d", 6.00002},
  {"yaw", 4.43941 * 180 / pi()},
  {"speed", 19.728 / MPH2MPS}
};

// Be carefull: Cartesian and Frenet coordinates not related!
// id, x, y, vx, vy, s, d
// speed in m/s
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
  {1, 804.479, 1129.09, 5, 0, 20, 6},
  {2, 834.597, 1132.9, 20, 0, 50, 2},
  {3, 789.336, 1125.44, 10, 0, 5, 10},
  // {4, 789.441, 1129.44, 0, 0, 5, 6},
});

json stub_front_vehicle = json::array({
  {1, 178.724, 2411.89, 0, -18.0431, 5423.74, 5.87362}, // don't perform lane change when there is actually free space available
  // {0, 2055.35, 1275.02, 7.53, 16.111, 1328.98, 5.88} // space ahead not detected
  // {1, 804.479, 1129.09, 30*MPH2MPS, 0, 20, 6},
  // {2, 834.597, 1132.9, 20, 0, 50, 2},
  // {3, 834.597, 1129, 20, 0, 50, 6},
});