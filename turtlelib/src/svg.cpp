#include<iosfwd> // contains forward definitions for iostream objects
#include <iostream>
#include"turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/svg.hpp"
#include <ostream>
#include <cmath>
#include <string>


namespace turtlelib
{
    void OpenTag(std::ostream & os, double width, double height){
        double xpixel = width*96;
        double ypixel = height*96;
        os << "<svg width=\"" << width << "in\" height=\"" << height << "in\" "
            << "viewBox=\"0 0 " << xpixel << " " << ypixel << "\"" << " xmlns=\"http://www.w3.org/2000/svg\">" << "\n";
    }

    void defTag(std::ostream &os) {
        os << "<defs>\n"
        << "  <marker\n"
        << "     style=\"overflow:visible\"\n"
        << "     id=\"Arrow1Sstart\"\n"
        << "     refX=\"0.0\"\n"
        << "     refY=\"0.0\"\n"
        << "     orient=\"auto\">\n"
        << "    <path\n"
        << "       transform=\"scale(0.2) translate(6,0)\"\n"
        << "       style=\"fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt\"\n"
        << "       d=\"M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z \"\n"
        << "       />\n"
        << "  </marker>\n"
        << "</defs>\n";
    }

    void drawCircle(std::ostream & os, double cx, double cy, double r, std::string stroke){
        os << "<circle cx=\""<< cx << "\" cy=\""<< cy <<"\" r=\" "<< r <<"\" stroke=\"" << stroke << "\" fill=\"" << stroke <<"\" stroke-width=\"1\" />\n";
    }

    void drawVector(std::ostream & os, double x1, double x2, double y1, double y2, std::string stroke){
        os << "<line x1=\""<< x1 << "\" x2=\""<< x2 << "\" y1=\""<< y1<< "\" y2=\"" << y2 <<"\" stroke=\"" << stroke << "\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />\n";
    }
    /// the x1 x2 y1 y2 here is from drawVector
    void groupTag(std::ostream & os, double x2, double y2, std::string text){
        os << "<g>\n";
        double add_x1 = 96 + x2;
        double add_y1 = y2 - 96;
        double offsety = 0.25+y2;
        drawVector(os, add_x1, x2, y2, y2, "red");
        drawVector(os, x2, x2, add_y1, y2, "green");
        os << "<text x=\""<< x2 << "\" y=\"" << offsety << "\">{" << text << "}</text>\n";
        os << "</g>\n";
    }
    void endSvg(std::ostream & os){
        os << "</svg>\n";
    }
}

    

