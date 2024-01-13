#include <catch2/catch_test_macros.hpp>
#include "turtlelib/geometry2d.hpp"
#include<catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>




TEST_CASE( "Angles are normalized to (PI, PI]", "[normalize_angle]") {
    REQUIRE_THAT( turtlelib::normalize_angle(turtlelib::PI), 
        Catch::Matchers::WithinAbs(turtlelib::PI, 1e-5));
}




