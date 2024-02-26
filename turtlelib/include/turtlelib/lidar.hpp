#ifndef TURTLELIB_LIDAR_INCLUDE_GUARD_HPP
#define TURTLELIB_LIDAR_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.
#include<iosfwd> // contains forward definitions for iostream objects
#include"turtlelib/geometry2d.hpp"
#include"turtlelib/se2d.hpp"
#include <cmath>
#include <string>

namespace turtlelib
{   
    /// @brief find if the lidar beam intersects with the column
    /// @param column the column's transform in the world frame
    /// @param robot the robot's transform in the world frame
    /// @return bool
    bool if_intersect(Point2D column, Point2D robot, double max_range, double theta, double column_radius); 

    Point2D intersectPoint(Point2D column, Point2D robot, double max_range, double theta, double column_radius);
    // Point2D intersectPointB(Point2D column, Point2D robot, double max_range, double theta, double column_radius);


}

#endif