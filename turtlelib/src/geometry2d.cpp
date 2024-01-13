#include "turtlelib/geometry2d.hpp"
#include <cmath>
#include <iostream>

namespace turtlelib {


    double normalize_angle(double rad) {
        // Normalize the angle to be within the range (-PI, PI]
        double normalizedAngle = rad - (ceil((rad + PI)/(2*PI))-1)*2*PI;
        return normalizedAngle;
    }


    std::ostream & operator<<(std::ostream & os, const Point2D & p) {
        // Outputs a 2-dimensional point as [xcomponent ycomponent]
        return os << "[" << p.x << " " << p.y << "]";
    }

    std::istream & operator>>(std::istream & is, Point2D & p) {
        return is;
    }

    Vector2D operator-(const Point2D & head, const Point2D & tail) {
        return Vector2D{};
    }

    Point2D operator+(const Point2D & tail, const Vector2D & disp) {
        return Point2D{};
    }

    std::ostream & operator<<(std::ostream & os, const Vector2D & v) {
        return os;
    }

    std::istream & operator>>(std::istream & is, Vector2D & v) {
        return is;
    }


}
