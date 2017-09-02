#ifndef MAP_H
#define MAP_H

#include <vector>
// #include "trajectory_strategy.h"

using namespace std;

class Map {
public:
  static Map& GetInstance();

  vector<double>GetXY();
  vector<double>GetFrenet();
  
  // C++11: delete explicitly copy ctor and assignment operator
  Map(Map const&)               = delete;
  void operator=(Map const&)    = delete;

private:
  Map() {};
};

#endif