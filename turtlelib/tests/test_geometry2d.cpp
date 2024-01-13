#include <catch2/catch_test_macros.hpp>
#include "turtlelib/geometry2d.hpp"
#include<catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>




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




