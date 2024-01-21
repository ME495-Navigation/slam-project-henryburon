#include "turtlelib/svg.hpp"
#include "turtlelib/se2d.hpp"
#include <cmath>
#include <iostream>
#include <sstream>

namespace turtlelib

{
    void Svg::DrawPoint(double cx, double cy, const std::string &pcolor, std::ofstream &outFile)
    {
        // Make a 2D translation object
        turtlelib::Transform2D turtlelibCoordinates(turtlelib::Vector2D{408,528});

        // Turn point into Vector2D object
        turtlelib::Vector2D originalPoint{cx, cy};

        // Apply the transformation
        turtlelib::Vector2D transformedPoint = turtlelibCoordinates(originalPoint);


        outFile << "<circle cx=\"" << transformedPoint.x << "\" cy=\"" << transformedPoint.y
                << "\" r=\"3\" stroke=\"" << pcolor << "\" fill=\"" << pcolor << "\" stroke-width=\"1\" />\n";
    }

    void Svg::DrawVector(double x1, double x2, double y1, double y2, const std::string &vcolor, std::ofstream &outFile)
    {
        outFile << "<line x1=\"" << x1 << "\" x2=\"" << x2
                << "\" y1=\"" << y1 << "\" y2=\"" << y2 << "\" stroke=\""
                << vcolor << "\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />\n";
    }

}