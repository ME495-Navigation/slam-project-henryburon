#include <catch2/catch_test_macros.hpp>
#include "turtlelib/geometry2d.hpp"
#include<catch2/matchers/catch_matchers_floating_point.hpp>
#include <sstream>

TEST_CASE( "Angles are normalized to (PI, PI]", "[normalize_angle]") {
    REQUIRE_THAT( turtlelib::normalize_angle(turtlelib::PI), 
        Catch::Matchers::WithinAbs(turtlelib::PI, 1e-5));
    REQUIRE_THAT( turtlelib::normalize_angle(-turtlelib::PI),
        Catch::Matchers::WithinAbs(turtlelib::PI, 1e-5));
    REQUIRE_THAT( turtlelib::normalize_angle(0),
        Catch::Matchers::WithinAbs(0, 1e-5));
    REQUIRE_THAT( turtlelib::normalize_angle(-turtlelib::PI/4),
        Catch::Matchers::WithinAbs(-turtlelib::PI/4, 1e-5));
    REQUIRE_THAT( turtlelib::normalize_angle(3*turtlelib::PI/2),
        Catch::Matchers::WithinAbs(-turtlelib::PI/2, 1e-5));
    REQUIRE_THAT( turtlelib::normalize_angle(-5*turtlelib::PI/2),
        Catch::Matchers::WithinAbs(-turtlelib::PI/2, 1e-5));
    }

    TEST_CASE("Outputs a 2D point as [x y]", "[Point2D]") {
        turtlelib::Point2D p{3.4, 5.6};  // Initialize Point2D with x = 3.4 and y = 5.6
        std::ostringstream output_stream;  // Create a stringstream to capture the output
        output_stream << p;  // Insert the Point2D object into the stringstream
        std::string expected_output = "[3.4 5.6]"; // Define the expected output string
        REQUIRE(output_stream.str() == expected_output);  // Check if the captured string matches the expected string
    }
