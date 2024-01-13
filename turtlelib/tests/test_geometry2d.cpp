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

    TEST_CASE("Read vectors entered as [x y] or x y", "[Point2D]") {
        turtlelib::Point2D p;
        SECTION("Read vector in format [x y]") {
            std::string input_string_1 = "[1.8 5.1]";
            std::istringstream input_stream_1(input_string_1);

            input_stream_1 >> p;
            REQUIRE_THAT(p.x, Catch::Matchers::WithinAbs(1.8, 1e-5));
            REQUIRE_THAT(p.y, Catch::Matchers::WithinAbs(5.1, 1e-5));
    }
        SECTION("Read vector in format x y") {
            std::string input_string_2 = "1.6 9.3";
            std::istringstream input_stream_2(input_string_2);

            input_stream_2 >> p;  // Read into Point2D object

            REQUIRE_THAT(p.x, Catch::Matchers::WithinAbs(1.6, 1e-5));
            REQUIRE_THAT(p.y, Catch::Matchers::WithinAbs(9.3, 1e-5));
    }
}
