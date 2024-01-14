#include "turtlelib/se2d.hpp"
#include <cmath>
#include <iostream>


namespace turtlelib {

    std::ostream &operator<<(std::ostream &os, const Twist2D &tw)
    {
        // Print the Twist2D in the format [w x y]
        return os << "[" << tw.omega << " " << tw.x << " " << tw.y << "]";
    }



}