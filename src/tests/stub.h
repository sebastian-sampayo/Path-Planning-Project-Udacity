#include <vector>

#include "../json.hpp"
#include "../trajectory.h"

// for convenience
using namespace std;
using json = nlohmann::json;

json stub_ego_data = {
  {"x", 784.6001},
  {"y", 1135.571},
  {"s", 0},
  {"d", 6},
  {"yaw", 0},
  {"speed", 50}
};

// Be carefull: Cartesian and Frenet coordinates not related!
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
