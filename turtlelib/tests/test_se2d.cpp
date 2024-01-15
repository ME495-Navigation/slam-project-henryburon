 #include <catch2/catch_test_macros.hpp>
#include "turtlelib/se2d.hpp"
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <sstream>


TEST_CASE("Prints the Twist2D in the format [w x y]", "[Twist2D]") 
{
    turtlelib::Twist2D tw{0.5, 4.4, 3.2}; // Initialize a turtlelib Twist2D object

    std::ostringstream output; // Create an output stream to capture the printed output

    output << tw;

    REQUIRE(output.str() == "[0.5 4.4 3.2]");
}

TEST_CASE("Reads the Twist2D in the format [w x y] or w x y", "[Twist2D]")
{
    // Create a Twist2D object to store the parsed values
    turtlelib::Twist2D tw;
    SECTION("Read twist in format [w x y]")
    {
        // Create a string containing the input data in the format [w x y]
        std::string input_string = "[1.4 3.8 6.4]";

        // Create an input string stream and initialize it with the input string
        std::istringstream input_sstream(input_string);

        // Attempt to read the Twist2D object from the input string stream
        input_sstream >> tw;

        // Check if the parsed values match the expected values (with a tolerance)
        REQUIRE_THAT(tw.omega, Catch::Matchers::WithinAbs(1.4, 1e-5));
        REQUIRE_THAT(tw.x, Catch::Matchers::WithinAbs(3.8, 1e-5));
        REQUIRE_THAT(tw.y, Catch::Matchers::WithinAbs(6.4, 1e-5));
    }
    SECTION("Read twist in format w x y")
    {
        std::string input_string = "[1.2 1.8 8.4]";
        std::istringstream input_sstream(input_string);

        input_sstream >> tw;

        REQUIRE_THAT(tw.omega, Catch::Matchers::WithinAbs(1.2, 1e-5));
        REQUIRE_THAT(tw.x, Catch::Matchers::WithinAbs(1.8, 1e-5));
        REQUIRE_THAT(tw.y, Catch::Matchers::WithinAbs(8.4, 1e-5));
    }
}

// Still need to write test for identity transform
