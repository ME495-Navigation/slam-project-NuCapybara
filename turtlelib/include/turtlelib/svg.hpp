#ifndef TURTLELIB_SVG_INCLUDE_GUARD_HPP
#define TURTLELIB_SVG_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.
#include<iosfwd> // contains forward definitions for iostream objects
#include"turtlelib/geometry2d.hpp"
#include"turtlelib/se2d.hpp"
#include <string>

namespace turtlelib
{

 void OpenTag(std::ostream & os, double width, double height);
 void defTag(std::ostream & os);
 void drawCircle(std::ostream & os, Point2D pt, Transform2D t, std::string stroke);
 void drawVector(std::ostream & os, Vector2D v1, Vector2D v2, Transform2D t, std::string stroke);
 void groupTag(std::ostream & os, Transform2D t, std::string text);
 void endSvg(std::ostream & os);
}

#endif