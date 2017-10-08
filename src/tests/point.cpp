#include <iostream>

#include "../logger.h"
#include "../map.h"
#include "../point.h"
#include "../trajectory.h"

#include "../matplotlibcpp.h"

using namespace std;
namespace plt = matplotlibcpp;

int main()
{
  SET_LOG_LEVEL(logDEBUG3);
  // {1, 804.479, 1129.09, 0, 0, 20, 6},
  // {2, 834.597, 1132.9, 0, 0, 50, 2},
  // {3, 789.336, 1125.44, 10, 0, 5, 10},
  // PointCartesian pc(784.6001, 1130.571); // 
  // PointCartesian pc(804.479, 1129.09); // 20, 6
  // PointCartesian pc(834.597, 1132.9); // 50, 2
  // PointCartesian pc(789.336, 1125.44); // 5, 10
  PointFrenet pf0(18, 0);
  PointCartesian pc(pf0);
  Point p(pc);
  
  cout << pc << endl;
  cout << pf0 << endl;
  cout << p << endl;
  
  cout << "p.GetS() = " << p.GetS() << endl;
  
  PointFrenet pf(pc);
  
  for (int i = 0; i<5; ++i)
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

  Trajectory t;
  
  for (int i= 0; i < 10; ++i)
  {
    Point p;
    p.SetFrenet(i*10, 0);
    t.push_back(p);
  }

  cout << "Trajectory t: \n" << t;
  
  
  // Matplotlibcpp
  plt::plot({1,2,3,4});
  plt::save("../img/test.png");
  // plt::show();
  // plt::subplot(2, 2, 1);
  // plt::title("Red = Waypoints, Black=x(s)+y(s)");
  //plt::axis("equal");
  // plt::xlim(0, 2500);
  // plt::plot(Map::, maps_y_, "r."
           // , xs, ys, "k-");

  // plt::show();

  return 0;
}
