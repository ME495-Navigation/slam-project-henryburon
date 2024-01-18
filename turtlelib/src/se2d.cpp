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

    // Identity transform
    Transform2D::Transform2D(): transf{0.0, 0.0}, rot{0.0} {}

    // Transformation that is pure translation
    Transform2D::Transform2D(Vector2D trans): transf{trans.x, trans.y}, rot{0.0} {}

    // Transformation that is pure rotation, in radians
    Transform2D::Transform2D(double radians): transf{0.0, 0.0}, rot{radians} {}

    // Transformation with a translational and rotational component
    Transform2D::Transform2D(Vector2D trans, double radians)
        : transf{trans.x, trans.y}, 
          rot{radians} {}

    // Apply a transformation to a 2D point
    Point2D Transform2D::operator()(Point2D p) const{
        double x_transf = std::cos(rot)*p.x - std::sin(rot)*p.y + transf.x;
        double y_transf = std::sin(rot)*p.x + std::cos(rot)*p.y + transf.y;
        return {x_transf, y_transf};
    }

    // Apply a transformation to a 2D vector
    Vector2D Transform2D::operator()(Vector2D v) const{
        double x_transf = std::cos(rot)*v.x - std::sin(rot)*v.y + transf.x;
        double y_transf = std::sin(rot)*v.x + std::cos(rot)*v.y + transf.y;
        return {x_transf, y_transf};
    }




    Vector2D Transform2D::translation() const{
        return {transf.x, transf.y};
    }

    double Transform2D::rotation() const{
        return rot;
    }





}