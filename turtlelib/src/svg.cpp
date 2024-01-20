#include "turtlelib/svg.hpp"
#include <cmath>
#include <iostream>
#include <sstream>



namespace turtlelib

{
    void Svg::DrawPoint(double cx, double cy, const std::string& pcolor, std::ofstream& outFile)
    {
        outFile << "<circle cx=\"" << cx << "\" cy=\"" << cy 
            << "\" r=\"1\" fill=\"" << pcolor << "\" />\n";
    }
}