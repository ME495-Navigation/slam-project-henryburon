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

    Svg();

    /// \brief Draw a point
    /// \param cx - x-coordinate
    /// \param cy - y-coordinate
    /// \param pcolor - color of the stroke and fill
    /// \param outFile - output svg file
    void DrawPoint(double cx, double cy, const std::string& pcolor, std::ofstream& outFile);

    /// \brief Draw a vector
    /// \param x1 - the head's x-coordinate
    /// \param y1 - the head's y-coordinate
    /// \param x2 - the tail's x-coordiante
    /// \param y2 - the tail's y-coordinate
    /// \param vcolor - color of the stroke
    void DrawVector(double x1, double y1, double x2, double y2, std::string vcolor);


    };




























}

#endif