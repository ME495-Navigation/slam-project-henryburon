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
        turtlelib::Transform2D turtlelibCoordinates(turtlelib::Vector2D{408, 528});

        // Turn point into Vector2D object
        turtlelib::Vector2D originalPoint{cx, cy};

        // Apply the transformation
        turtlelib::Vector2D transformedPoint = turtlelibCoordinates(originalPoint);

        outFile << "<circle cx=\"" << transformedPoint.x << "\" cy=\"" << transformedPoint.y
                << "\" r=\"3\" stroke=\"" << pcolor << "\" fill=\"" << pcolor << "\" stroke-width=\"1\" />\n";
    }

    void Svg::DrawVector(double x1, double x2, double y1, double y2, const std::string &vcolor, std::ofstream &outFile)
    {
        // Make a 2D translation object
        turtlelib::Transform2D turtlelibCoordinates(turtlelib::Vector2D{408, 528});

        // Turn both head and tail into Vector2D objects
        turtlelib::Vector2D originalHead{x1, y1};
        turtlelib::Vector2D originalTail{x2, y2};

        // Apply the transformation to each
        turtlelib::Vector2D transformedHead = turtlelibCoordinates(originalHead);
        turtlelib::Vector2D transformedTail = turtlelibCoordinates(originalTail);

        outFile << "<line x1=\"" << transformedHead.x << "\" x2=\"" << transformedTail.x
                << "\" y1=\"" << transformedHead.y << "\" y2=\"" << transformedTail.y << "\" stroke=\""
                << vcolor << "\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />\n";
    }

    void Svg::DrawCoordinateFrame(double tail_x, double tail_y, 
                             double head_xc_x, double head_yc_x, 
                             double head_xc_y, double head_yc_y, 
                             const std::string &text, std::ofstream &outFile)
    {

        outFile << "<g>\n";

        // Turtlelib coordinates are obtained through DrawVector
        Svg::DrawVector(head_xc_x, tail_x, head_yc_x, tail_y, "red", outFile);
        Svg::DrawVector(head_xc_y, tail_x, head_yc_y, tail_y, "green", outFile);

        // Convert tail coordinates turtlelib coordinates
        turtlelib::Transform2D turtlelibCoordinates(turtlelib::Vector2D{408,528});
        turtlelib::Vector2D originalTail{tail_x, tail_y};
        turtlelib::Vector2D transformedTail = turtlelibCoordinates(originalTail);
        
        outFile << "<text x=\"" << transformedTail.x << "\" y=\"" << transformedTail.y << "\">{" << text << "}</text>\n";
        outFile << "</g>\n";

    }
}