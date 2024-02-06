#include "turtlelib/geometry2d.hpp"
#include <iostream>

namespace turtlelib
{

std::ostream & operator<<(std::ostream & os, const Point2D & p)
{
  os << "[" << p.x << " " << p.y << "]";
  return os;
}

std::istream & operator>>(std::istream & is, Point2D & p)
{
  char ch;
  ch = is.peek();
  if (ch != '[') {
    is >> p.x;
    is >> p.y;
  }
  // if the first one starts with [
  else {
    //consume "["
    is.get();
    //the next one for input is p.x
    is >> p.x;
    // is >> trash;
    is >> p.y;
  }// did not consume the closing ]

  return is;
}

double normalize_angle(double rad)
{
  while (rad <= -PI) {
      rad += 2 * PI; // 2.0
  }
  while (rad > PI) {
    rad -= 2 * PI;
  }
  return rad;
}


Vector2D operator-(const Point2D & head, const Point2D & tail)
{
  Vector2D v;
  v.x = head.x - tail.x;
  v.y = head.y - tail.y;
  return v; // return {head.x - tail.x, head.y - tail.y}
}

Point2D operator+(const Point2D & tail, const Vector2D & disp)
{
  Point2D pt;
  pt.x = tail.x + disp.x;
  pt.y = tail.y + disp.y;
  return pt; // return {tail.x + disp.x, tail.y + disp.y}
}

std::ostream & operator<<(std::ostream & os, const Vector2D & v)
{
  os << "[" << v.x << " " << v.y << "]";
  return os;
}


std::istream & operator>>(std::istream & is, Vector2D & v)
{
  is >> std::ws;
  char input;
  input = is.peek();
  //Sceanario: the first char is [
  if (input == '[') {
    is.get();         //consume [
    double x; // unitiailized
    double y;
    is >> x;
    is.get();         //consumes the whitespace between x and y
    is >> y;
    if (is.get() == ']') {
      v.x = x;
      v.y = y;
    } else {
      //if the last one is not ], then fails
      is.setstate(std::ios::failbit);
    }
  } else {
      double x; // uninitialized
    double y;
    is >> x;
    is >> y;
    v.x = x;
    v.y = y;
  }
  return is;
}

}
