#include <iostream>

#include "../logger.h"
#include "../map.h"
#include "../point.h"
#include "../trajectory.h"
#include "../utils.h"

#include "../matplotlibcpp.h"

using namespace std;
namespace plt = matplotlibcpp;

int main()
{
  SET_LOG_LEVEL(logDEBUG3);
  
  // Print some points
  cout << Point(PointCartesian(1105, 1180.27)) << endl;
  cout << Point(PointCartesian(1109.98, 1178.01)) << endl;
  cout << Point(PointFrenet(328.072, 6)) << endl;
  cout << Point(PointFrenet(332.78, 9)) << endl;
  cout << "Initial point: " << endl;
  cout << Point(PointFrenet(0, 0)) << endl;
  cout << Point(PointFrenet(0, 10)) << endl;
  cout << Point(PointFrenet(Map::GetInstance().MAX_S, 0)) << endl;
  cout << Point(PointFrenet(Map::GetInstance().MAX_S, 10)) << endl;
  
  // {1, 804.479, 1129.09, 0, 0, 20, 6},
  // {2, 834.597, 1132.9, 0, 0, 50, 2},
  // {3, 789.336, 1125.44, 10, 0, 5, 10},
  // PointCartesian pc(784.6001, 1130.571); // 
  // PointCartesian pc(804.479, 1129.09); // 20, 6
  // PointCartesian pc(834.597, 1132.9); // 50, 2
  // PointCartesian pc(789.336, 1125.44); // 5, 10
  // PointFrenet pf0(1037.03, 10.0461); // This point has issues with speed!!
  // PointFrenet pf0(1037, 10); // This point has issues with speed!!
  PointFrenet pf0(300, 10); // This point has issues with speed!!
  // PointFrenet pf0(Map::GetInstance().MAX_S-0.1, 10);
  PointCartesian pc(pf0);
  cout << "qx_s_(0): " << Map::GetInstance().qx_s_(0) << endl;
  cout << "qx_s_(max-0.1): " << Map::GetInstance().qx_s_(Map::GetInstance().MAX_S-0.1) << endl;
  cout << "qy_s_(0): " << Map::GetInstance().qy_s_(0) << endl;
  cout << "qy_s_(max-0.1): " << Map::GetInstance().qy_s_(Map::GetInstance().MAX_S-0.1) << endl;
  Point p(pc);
  
  cout << "pf0: " << pf0 << endl;
  cout << " pc: " << pc << endl;
  cout << "  p: " << p << endl;

  double delta_s = 18*0.02;
  PointFrenet pf2(pf0.s+delta_s, pf0.d);
  cout << "Point(pf0): " << Point(pf0) << endl;
  cout << "Point(pf2): " << Point(pf2) << endl;
  PointCartesian pc2(pf2);
  PointCartesian vc = PointCartesian(pc2.x - pc.x, pc2.y - pc.y);
  cout << "delta_s: " << delta_s << " |vc|: " << Magnitude(vc.x, vc.y)/0.02 << " | |vf|: " << delta_s/0.02 << endl;
  
  PointFrenet pf(pc);
  
  for (int i = 0; i<5; ++i)
  {
    pf = PointFrenet(pc);
    pc = PointCartesian(pf);
    cout << pf << endl;
    cout << pc << endl;
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

  // cout << "Trajectory t: \n" << t;
  
  
  // Matplotlibcpp
  // plt::plot({1,2,3,4});
  // plt::save("../img/test.png");
  // // plt::subplot(2, 2, 1);
  // // plt::title("Red = Waypoints, Black=x(s)+y(s)");
  // //plt::axis("equal");
  // // plt::xlim(0, 2500);
  // // plt::plot(Map::, maps_y_, "r."
           // // , xs, ys, "k-");

  // // plt::show();

  return 0;
}
