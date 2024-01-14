#include "turtlelib/se2d.hpp"
#include <cmath>
#include <iostream>
#include <sstream>


namespace turtlelib {

    std::ostream & operator<<(std::ostream & os, const Twist2D & tw)
    {
        // Print the Twist2D in the format [w x y]
        return os << "[" << tw.omega << " " << tw.x << " " << tw.y << "]";
    }

    std::istream & operator>>(std::istream & is, Twist2D & tw)
    {
        // Read the Twist2D in the format [w x y] or w x y
        char c1 = is.peek();

        if (c1 == '[')
        {
            is.get();
            is >> tw.omega >> tw.x >> tw.y;
            is.get();
        }
        else
        {
            is >> tw.omega >> tw.x >> tw.y;
        }
        return is;

    }

}