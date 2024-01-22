#include <iostream>
#include <fstream>
#include <iosfwd>
#include "turtlelib/svg.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"

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

    // Ask for T_ab
    turtlelib::Transform2D T_ab;
    std::cout << "Enter transform T_{ab}:\n";
    std::cin >> T_ab;

    // Ask for T_bc
    turtlelib::Transform2D T_bc;
    std::cout << "Enter transform T_{bc}:\n";
    std::cin >> T_bc;

    // Calculate other transformations
    turtlelib::Transform2D T_ba = T_ab.inv();
    turtlelib::Transform2D T_cb = T_bc.inv();
    turtlelib::Transform2D T_ac = T_ab * T_bc;
    turtlelib::Transform2D T_ca = T_ac.inv();

    // Output the transformations to cli
    std::cout << "T_{ab}: " << T_ab << std::endl;
    std::cout << "T_{ba}: " << T_ba << std::endl;
    std::cout << "T_{bc}: " << T_bc << std::endl;
    std::cout << "T_{cb}: " << T_cb << std::endl;
    std::cout << "T_{ac}: " << T_ac << std::endl;
    std::cout << "T_{ca}: " << T_ca << std::endl;

    // Frame A
    turtlelib::Point2D a_origin{0, 0};
    turtlelib::Vector2D a_xvector{1, 0};
    mySvg.DrawCoordinateFrame(a_origin, a_xvector, "A", svg_file);

    // Frame B
    turtlelib::Point2D b_origin = T_ab(a_origin);
    turtlelib::Vector2D b_xvector = T_ab(a_xvector);
    mySvg.DrawCoordinateFrame(b_origin, b_xvector, "B", svg_file);

    // Frame C
    turtlelib::Point2D c_origin = T_bc(b_origin);
    turtlelib::Vector2D c_xvector = T_bc(b_xvector);
    mySvg.DrawCoordinateFrame(c_origin, c_xvector, "C", svg_file);

    // Ask for point
    turtlelib::Point2D p_a;
    std::cout << "Enter point p_a:\n";
    std::cin >> p_a;

    // Calculate all the points
    turtlelib::Point2D p_b = T_ba(p_a);
    turtlelib::Point2D p_c = T_cb(p_b);

    // Output the points to cli
    std::cout << "p_a: " << p_a << std::endl;
    std::cout << "p_b: " << p_b << std::endl;
    std::cout << "p_c: " << p_c << std::endl;

    // Draw the points
    mySvg.DrawPoint(p_a.x, p_a.y, "purple", svg_file);
    mySvg.DrawPoint(p_b.x, p_b.y, "brown", svg_file);
    mySvg.DrawPoint(p_c.x, p_c.y, "orange", svg_file);


    svg_file << "</svg>\n";
    svg_file.close();
}