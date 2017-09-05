#include <iostream>

#include "../logger.h"
#include "../map.h"
#include "../point.h"
// #include "../kinematic_state.h"
#include "../trajectory.h"

#include "../matplotlibcpp.h"

using namespace std;
namespace plt = matplotlibcpp;

int main()
{
  SET_LOG_LEVEL(logDEBUG3);
  PointCartesian pc(784.6001, 1130.571);
  Point p(pc);
  
  cout << "p.GetS() = " << p.GetS() << endl;
  
  PointFrenet pf(pc);
  
  for (int i = 0; i<1; ++i)
  {
    pf = PointFrenet(pc);
    pc = PointCartesian(pf);
    cout << pc << endl;
    cout << pf << endl;
  }
  
  // p.SetXY(1,1);
  // p.GetX();
  // p.GetS();
  // p.SetFrenet(2,2);
  // p.GetY();
  // p.GetD();

  // KinematicState k;

  // k.GetX();

  Trajectory t;
  
  for (int i= 0; i < 10; ++i)
  {
    Point p;
    p.SetFrenet(i*10, 0);
    t.push_back(p);
  }

  cout << "Trajectory t: \n" << t;
  
  
  // Matplotlibcpp
  // plt::subplot(2, 2, 1);
  plt::title("Red = Waypoints, Black=x(s)+y(s)");
  //plt::axis("equal");
  plt::xlim(0, 2500);
  // plt::plot(Map::, maps_y_, "r."
           // , xs, ys, "k-");

  plt::show();

  return 0;
}
