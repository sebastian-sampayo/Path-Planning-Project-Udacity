#ifndef POINT_H
#define POINT_H

#include <iostream>

using namespace std;

class Point {
public:
  // Constructors and destructors
  Point();
  virtual ~Point();

  // "Get" methods
  double GetX() const;
  double GetY() const;
  double GetS() const;
  double GetD() const;
  double GetYaw() const;

  // "Set" methods
  void SetXY(double x, double y);
  void SetFrenet(double s, double d);
  
  // Printer
  friend ostream& operator<<(ostream& os, const Point& p);

private:
  double x_;
  double y_;
  double s_;
  double d_;
  bool valid_cartesian_;
  bool valid_frenet_;
};

#endif
