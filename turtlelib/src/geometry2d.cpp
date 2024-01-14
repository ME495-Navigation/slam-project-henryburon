#include "turtlelib/geometry2d.hpp"
#include <cmath>
#include <iostream>

namespace turtlelib {


    double normalize_angle(double rad) {
        // Normalize the angle to be within the range (-PI, PI]
        double normalizedAngle = rad - (ceil((rad + PI)/(2*PI))-1)*2*PI;
        return normalizedAngle;
    }

    // ########################################################################

    std::ostream & operator<<(std::ostream & os, const Point2D & p) {
        // Outputs a 2-dimensional point as [x y]
        return os << "[" << p.x << " " << p.y << "]";
    }

    // ########################################################################

    std::istream & operator>>(std::istream & is, Point2D & p) {
        // Read vectors entered as [x y] or x y
        char ch;
        // Extracts characters from stream
        // This if-statement is designed to work with the multiple input types
        // So, if successful extraction from the input stream...
        if (is >> ch) {
            // First, check if the character is a '[
            if (ch == '[') {
                // If the input starts with '[', read x and y values, then expect a closing ']'
                is >> p.x >> p.y;
                is >> ch; // Read the expected ']'
                if (ch != ']') {
                    // If the next character is not ']', set the stream to a fail state
                    is.setstate(std::ios::failbit);
                }
            } else {
                // If the input does not start with '[', put the character back, then read x and y
                is.putback(ch);
                is >> p.x >> p.y;
            }
        }
        return is;
    }

    // ########################################################################

    Vector2D operator-(const Point2D & head, const Point2D & tail) {
        // Subtracting one point from another yields a vector
        return {head.x - tail.x, head.y - tail.y};
    }

    // ########################################################################

    Point2D operator+(const Point2D & tail, const Vector2D & disp) {
        // Adding a vector to a point yields a new point displaced by the vector
        return {tail.x + disp.x, tail.y + disp.y};
    }

    // ########################################################################

    std::ostream & operator<<(std::ostream & os, const Vector2D & v) {
        // Outputs a 2-dimensional vector as [x y]
        return os << "[" << v.x << " " << v.y << "]";
        
    }

    // ########################################################################

    std::istream & operator>>(std::istream & is, Vector2D & v) {
        // Read vectors entered as [x y] or x y
        char ch;
        // Extracts characters from stream
        // This if-statement is designed to work with the multiple input types
        // So, if successful extraction from the input stream...
        if (is >> ch) {
            // First, check if the character is a '[
            if (ch == '[') {
                // If the input starts with '[', read x and y values, then expect a closing ']'
                is >> v.x >> v.y;
                is >> ch; // Read the expected ']'
                if (ch != ']') {
                    // If the next character is not ']', set the stream to a fail state
                    is.setstate(std::ios::failbit);
                }
            } else {
                // If the input does not start with '[', put the character back, then read x and y
                is.putback(ch);
                is >> v.x >> v.y;
            }
        }
        return is;
    }


}
