#ifndef POINT_H
#define POINT_H

#include <iostream>

using namespace std;

// ----------------------------------------------------------------------------
struct PointCartesian
{
  double x;
  double y;
  
  // Constructor by parameters
  explicit PointCartesian(double x = 0, double y = 0) : x(x), y(y) {}
  // Conversion constructor from PointFrenet
  PointCartesian(const struct PointFrenet&);

  // Printer
  friend ostream& operator<<(ostream& os, const PointCartesian& p);

};

// ----------------------------------------------------------------------------
struct PointFrenet
{
  double s;
  double d;

  // Constructor by parameters
  explicit PointFrenet(double s = 0, double d = 0) : s(s), d(d) {}
  // Conversion constructor from PointCartesian
  PointFrenet(const struct PointCartesian&);

  // Printer
  friend ostream& operator<<(ostream& os, const PointFrenet& p);
};

// ----------------------------------------------------------------------------
class Point {
public:
  // Constructors and destructors
  //! Default constructor. Creates a point in s=0, d=0
  Point();

  //! Convert from a Cartesian point
  Point(const PointCartesian& p);

  //! Convert from a Frenet point
  Point(const PointFrenet& p);

  //! Sets both Cartesian and Frenet coordinates without making any conversion
  Point(const PointCartesian& pc, const PointFrenet& pf);

  //! Destructor
  virtual ~Point();

  // "Get" methods
  double GetX() const;
  double GetY() const;
  double GetS() const;
  double GetD() const;

  // "Set" methods
  void SetXY(double x, double y);
  void SetFrenet(double s, double d);

  // Arithmetic operations
  Point& operator+=(const PointFrenet& rhs);
  Point& operator-=(const PointFrenet& rhs);
  // friend Point operator+(const Point& lhs, const Point& rhs);
  // friend Point operator+(const Point& lhs, const PointFrenet& rhs);
  // friend Point operator+(const PointFrenet& lhs, const Point& lhs);

  // Printer
  friend ostream& operator<<(ostream& os, const Point& p);

private:
  // Use more than one representation under the hood
  PointCartesian point_cartesian_;
  PointFrenet point_frenet_;
};

#endif
