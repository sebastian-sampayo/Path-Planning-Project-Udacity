#include <iostream>

#include "../map.h"
#include "../position.h"
#include "../kinematic_state.h"
#include "../trajectory.h"

using namespace std;

int main()
{
  Position p;
  
  p.SetXY(1,1);
  p.GetX();
  p.GetS();
  p.SetFrenet(2,2);
  p.GetY();
  p.GetD();
  p.SetYaw(1);
  p.GetX();
  p.GetS();

  KinematicState k;

  // k.GetX();

  Trajectory t;

  cout << t;

  return 0;
}