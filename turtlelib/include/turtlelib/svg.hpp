#ifndef TURTLELIB_SVG_INCLUDE_GUARD_HPP
#define TURTLELIB_SVG_INCLUDE_GUARD_HPP
/// \file
/// \brief Visualization in svg

#include <iosfwd>
#include <iostream>
#include <fstream>
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include <vector>

namespace turtlelib
{

    // this class design could be greatly improved.
    // it seems to only aid the user in outputting svg primitives, but it does not encapsulate or abstract writing out the svg header.
    // this puts a burden on your user to know details they should not need to know about svgs
    class Svg
    {
    private:
        std::vector<std::string> svg_Vec;
    public:
        /// \brief Draw a point
        /// \param cx - x-coordinate
        /// \param cy - y-coordinate
        /// \param pcolor - color of the stroke and fill
        /// \param outFile - output svg file
        void DrawPoint(double cx, double cy, const std::string &pcolor, std::ofstream &outFile);

        /// \brief Draw a 2D vector
        /// \param origin - vector's (0,0) relative to turtlelib coordinates
        /// \param vector - vector's head relative to its origin
        /// \param vcolor - color of vector
        /// \param outFile - output svg file
        void DrawVector(Point2D origin, Vector2D vector, const std::string &vcolor, std::ofstream &outFile);

        /// \brief Draw a 2D coordinate frame
        /// \param origin - frame's (0,0) relative to turtlelib coordinates
        /// \param x_vector - x-vector's head relative to the frame's (0,0), the origin
        /// \param text - displayed frame identification text
        /// \param outFile - output svg file
        void DrawCoordinateFrame(Point2D origin, Vector2D x_vector, const std::string &text, std::ofstream &outFile);

        /// \brief Gets the string
        std::string getSvgString() const;
    };

}

#endif
