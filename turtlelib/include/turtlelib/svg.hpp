#ifndef TURTLELIB_SVG_INCLUDE_GUARD_HPP
#define TURTLELIB_SVG_INCLUDE_GUARD_HPP
/// \file
/// \brief Visualization in svg

#include<iosfwd>
#include<iostream>
#include<fstream>
#include"turtlelib/geometry2d.hpp"
#include"turtlelib/se2d.hpp"

namespace turtlelib
{


    class Svg {
    private:

    public:

    /// \brief Draw a point
    /// \param cx - x-coordinate
    /// \param cy - y-coordinate
    /// \param pcolor - color of the stroke and fill
    /// \param outFile - output svg file
    void DrawPoint(double cx, double cy, const std::string& pcolor, std::ofstream& outFile);


    void DrawVector(Point2D origin, Vector2D vector, const std::string& vcolor, std::ofstream& outFile);

    /// \brief Draw a coordinate frame
    /// \param tail_x - the frame origin's x-coordinate
    /// \param tail_y - the frame origin's y-coordinate
    /// \param head_xc_x - the x-coordinate of the "x" vector's head
    /// \param head_yc_x - the y-coordinate of the "x" vector's head
    /// \param head_xc_y - the x-coordinate of the "y" vector's head
    /// \param head_yc_y - the y-coordinate of the "y" vector's head
    /// \param text - text identifying the coordinate frame
    /// \param outFile - output svg file
    void DrawCoordinateFrame(double tail_x, double tail_y, double head_xc_x, double head_yc_x, double head_xc_y, double head_yc_y, const std::string& text, std::ofstream& outFile);





    };




























}

#endif