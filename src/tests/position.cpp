#include <iostream>

#include "../map.h"
#include "../point.h"
#include "../kinematic_state.h"
#include "../trajectory.h"

using namespace std;

int main()
{
  Point p;
  
  p.SetXY(1,1);
  p.GetX();
  p.GetS();
  p.SetFrenet(2,2);
  p.GetY();
  p.GetD();

  KinematicState k;

  // k.GetX();

  Trajectory t;

  cout << t;

  return 0;
}
