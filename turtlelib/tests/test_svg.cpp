#include<iosfwd> // contains forward definitions for iostream objects
#include <iostream>
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <string>
#include <sstream>
#include "turtlelib/svg.hpp"
#include <fstream>


namespace turtlelib
{
    TEST_CASE("svgfile", "test"){
        std::ostringstream os;  
        double width = 8.50000;
        double height = 11.00000;
        double cx = 500.0000;
        double cy = 500.0000000;
        double r = 3.0000;
        double x1 = 312.0000;
        double x2 = 408.000000;
        double y1 = 624.00000;
        double y2 = 528.000;
        std::string stroke = "purple";
        std::string text = "a";
        OpenTag(os, width, height);
        defTag(os);
        drawCircle(os, cx, cy, r, stroke);
        drawVector(os, x1, x2, y1, y2, stroke);
        groupTag(os, x2, y2, text);
        endSvg(os);

        std::string result = os.str();
        std::ofstream("output.svg") << os.str();
        }
}
