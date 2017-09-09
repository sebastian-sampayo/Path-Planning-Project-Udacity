#include <vector>

#include "../json.hpp"
#include "../trajectory.h"

// for convenience
using namespace std;
using json = nlohmann::json;

json stub_ego_data = {
  {"x", 0},
  {"y", 0},
  {"s", 0},
  {"d", 0},
  {"yaw", 0},
  {"speed", 0}
};

json stub_environment_data = json::array({
  {0, 0, 0, 0, 0, 0, 0},
  {1, 1, 1, 0, 0, 0, 0},
  {8, 8, 8, 0, 0, 0, 0},
  {5, 5, 5, 0, 0, 0, 0}
});

json stub_environment_data2 = json::array({
  {0, 0, 0, 0, 0, 0, 0},
  {1, 1, 2, 0, 0, 0, 0},
  {5, 5, 6, 0, 0, 0, 0}
});