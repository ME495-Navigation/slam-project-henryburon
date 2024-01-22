#include <iostream>
#include <fstream>
#include <iosfwd>
#include"turtlelib/svg.hpp"
#include"turtlelib/se2d.hpp"
#include"turtlelib/geometry2d.hpp"

int main()
{
    // Create and open a text file
    std::ofstream svg_file("../tmp/frames.svg");

    // Create an instance of the Svg class
    turtlelib::Svg mySvg;

    // Initialize the svg file
    svg_file << "<svg width=\"8.500000in\" height=\"11.000000in\" viewBox=\"0 0 816.000000 1056.000000\" xmlns=\"http://www.w3.org/2000/svg\">\n";

    // Define arrow head attached to a line (to draw a vector)
    svg_file << "<defs>\n";
    svg_file << "<marker\n";
    svg_file << "style=\"overflow:visible\"\n";
    svg_file << "id=\"Arrow1Sstart\"\n";
    svg_file << "refX=\"0.0\"\n";
    svg_file << "refY=\"0.0\"\n";
    svg_file << "orient=\"auto\">\n";
    svg_file << "<path\n";
    svg_file << "transform=\"scale(0.2) translate(6,0)\"\n";
    svg_file << "style=\"fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt\"\n";
    svg_file << "d=\"M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z \"\n";
    svg_file << "/>\n";
    svg_file << "</marker>\n";
    svg_file << "</defs>\n";

    // Draw frame A at (0,0)
    turtlelib::Point2D a_origin{0,0};
    turtlelib::Vector2D a_xvector{1,0};
    mySvg.DrawCoordinateFrame(a_origin, a_xvector, "a", svg_file);

    // Give formatting info
    std::cout << "Enter all transforms in the form: deg x y (ex. 90 0 1)\n";

    // Ask for T_ab
    double T_ab_deg, T_ab_x, T_ab_y;
    std::cout << "Enter transform T_ab: ";
    std::cin >> T_ab_deg >> T_ab_x >> T_ab_y;

    // Make transform object (rotation then translation)
    turtlelib::Transform2D ab_trans(turtlelib::Vector2D{T_ab_x, T_ab_y});
    turtlelib::Transform2D ab_rot(turtlelib::deg2rad(T_ab_deg)); // Does it need to be able to accept radians?

    // Apply the transformation
    turtlelib::Point2D ab_origin = ab_trans(a_origin);
    turtlelib::Vector2D ab_xvector = ab_rot(a_xvector);

    // Draw T_ab
    mySvg.DrawCoordinateFrame(ab_origin, ab_xvector, "T_ab", svg_file);

    // Ask for T_bc
    double T_bc_deg, T_bc_x, T_bc_y;
    std::cout << "Enter transform T_bc: ";
    std::cin >> T_bc_deg >> T_bc_x >> T_bc_y;

    // Make transform object
    turtlelib::Transform2D bc_trans(turtlelib::Vector2D{T_bc_x, T_bc_y});
    turtlelib::Transform2D bc_rot(turtlelib::deg2rad(T_bc_deg));

    // Apply the transformation
    turtlelib::Point2D bc_origin = bc_trans(ab_origin);
    turtlelib::Vector2D bc_xvector = bc_rot(ab_xvector);

    // Draw T_bc
    mySvg.DrawCoordinateFrame(bc_origin, bc_xvector, "T_bc", svg_file);



































    

    




    svg_file << "</svg>\n";
    svg_file.close();
}