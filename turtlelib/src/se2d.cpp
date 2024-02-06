#include <iosfwd> // contains forward definitions for iostream objects
#include <iostream>
#include "turtlelib/se2d.hpp"

namespace turtlelib
{

std::ostream & operator<<(std::ostream & os, const Twist2D & tw)
{
  os << "[" << tw.omega << " " << tw.x << " " << tw.y << "]";
  return os;
}

std::istream & operator>>(std::istream & is, Twist2D & tw)
{
    char ch; //uniit
  ch = is.peek();
  if (ch != '[') {
    is >> tw.omega;
    is >> tw.x;
    is >> tw.y;
  }
  // if the first one starts with [
  else {
    //consume "["
    is.get();
    is >> tw.omega;
    is >> tw.x;
    is >> tw.y;
  }
  return is;
}


Transform2D::Transform2D()
: linear{Vector2D{0, 0}},
  angular{0}
{}


Transform2D::Transform2D(Vector2D trans)
: linear{trans},
  angular{0}
{}


Transform2D::Transform2D(double radians)
: linear{Vector2D{0, 0}},
  angular{radians}
{}

Transform2D::Transform2D(Vector2D trans, double radians)
: linear{trans},
  angular{radians}
{}

Point2D Transform2D::operator()(Point2D p) const
{
  Point2D newPt;
  newPt.x = p.x * cos(angular) - sin(angular) * p.y + linear.x;
  newPt.y = p.x * sin(angular) + cos(angular) * p.y + linear.y;
  return newPt; // can return {p.x * cos .., ...}
}

Vector2D Transform2D::operator()(Vector2D v) const
{
  const auto new_x = v.x * cos(angular) - sin(angular) * v.y;     //for vector there is no translation
  const auto new_y = v.x * sin(angular) + cos(angular) * v.y;
  return Vector2D{new_x, new_y}; // good use of const auto, can just return {new_x, new_y}
}

Twist2D Transform2D::operator()(Twist2D v) const
{
    Twist2D newTwist; // no need for this temporar
  newTwist.x = v.omega * linear.y + v.x * cos(angular) - v.y * sin(angular);
  newTwist.y = v.omega * (-linear.x) + v.x * sin(angular) + v.y * cos(angular);
  newTwist.omega = v.omega;
  return newTwist;
}

Transform2D Transform2D::inv() const
{
  Transform2D newTrans;
  newTrans.linear.x = -linear.x * cos(angular) - linear.y * sin(angular);
  newTrans.linear.y = -linear.y * cos(angular) + linear.x * sin(angular);
  auto oldAngular = angular; // const
  newTrans.angular = -oldAngular;
  return newTrans; // return {}
}

Transform2D & Transform2D::operator*=(const Transform2D & rhs)
{
  const auto new_angular = angular + rhs.angular;
  const auto new_x = rhs.linear.x * cos(angular) - rhs.linear.y * sin(angular) + linear.x;
  const auto new_y = rhs.linear.x * sin(angular) + rhs.linear.y * cos(angular) + linear.y;

  angular = new_angular;
  linear.x = new_x;
  linear.y = new_y;
  return *this;
}

Vector2D Transform2D::translation() const
{
  return linear;
}


double Transform2D::rotation() const
{
  return angular;
}


std::ostream & operator<<(std::ostream & os, const Transform2D & tf)
{
  os << "deg: " << rad2deg(tf.angular) << " x: " << tf.linear.x << " y: " << tf.linear.y;
  return os;
}


std::istream & operator>>(std::istream & is, Transform2D & tf)
{
  // Remove whitespace
  is >> std::ws;
  double ang_val, x_val, y_val;
  std::string label_ang, label_x, label_y;
  char c = is.peek();
  if (c == 'd') {
    is >> label_ang >> ang_val >> label_x >> x_val >> label_y >> y_val;
  } else {
    is >> ang_val >> x_val >> y_val;
  }
  // Read \n character?????????????????
  c = is.get();
  tf = Transform2D(Vector2D{x_val, y_val}, deg2rad(ang_val));

  return is;
}


Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
{
  return lhs *= rhs;
}

}
