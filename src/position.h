#ifndef POSITION_H
#define POSITION_H

#include <iostream>

using namespace std;

class Position {
public:
  // Constructors and destructors
  Position();
  virtual ~Position();

  // "Get" methods
  double GetX() const;
  double GetY() const;
  double GetS() const;
  double GetD() const;
  double GetYaw() const;

  // "Set" methods
  void SetXY(double x, double y);
  void SetFrenet(double s, double d);
  void SetYaw(double yaw);
  
  // Printer
  friend ostream& operator<<(ostream& os, const Position& p);

private:
  double x_;
  double y_;
  double s_;
  double d_;
  double yaw_;
  bool valid_cartesian_;
  bool valid_frenet_;
  bool valid_yaw_;
};

#endif