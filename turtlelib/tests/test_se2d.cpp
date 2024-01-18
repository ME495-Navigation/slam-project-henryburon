#include <catch2/catch_test_macros.hpp>
#include"turtlelib/se2d.hpp"
#include"turtlelib/geometry2d.hpp"
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

TEST_CASE("Identity Transformation Test", "[Transform2D]") {    
    turtlelib::Transform2D identity;

    SECTION("Testing transform")
    {
        turtlelib::Vector2D vec{3.5, 4.1};
        turtlelib::Vector2D transformed_vector = identity(vec);
        REQUIRE_THAT(transformed_vector.x, Catch::Matchers::WithinAbs(3.5, 1e-5));
        REQUIRE_THAT(transformed_vector.y, Catch::Matchers::WithinAbs(4.1, 1e-5));
    }

    //COME BACK TO. NEED TO DEFINE operator() with a double. or do it another way
    // SECTION("Testing rotation")
    // {
    //     double angle = 3.14;
    //     turtlelib::Transform2D identity(angle);
    //     turtlelib::Point2D point{1.0, 2.0};

    //     turtlelib::Point2D rotated_point = identity(point);
    //     // double expected_x = 1.0;
    //     // double expected_y = 2.0;

    //     REQUIRE_THAT(rotated_point.x, Catch::Matchers::WithinAbs(1.0, 1e-5));
    //     REQUIRE_THAT(rotated_point.y, Catch::Matchers::WithinAbs(2.0, 1e-5));
    // }
    
}

TEST_CASE("Translation", "[Transform]")
{
    turtlelib::Vector2D vector{9.9, 8.8};
    turtlelib::Transform2D test_translation(vector);

    // Check that the transform was a pure translation
    REQUIRE_THAT(test_translation.translation().x, Catch::Matchers::WithinAbs(9.9, 1e-5));
    REQUIRE_THAT(test_translation.translation().y, Catch::Matchers::WithinAbs(8.8, 1e-5));
    REQUIRE_THAT(test_translation.rotation(), Catch::Matchers::WithinAbs(0.0, 1e-5));


}

TEST_CASE("Rotation", "[Transform]")
{
    double radians{3.14};
    turtlelib::Transform2D test_rotation(radians);

    // Check that the transform was a pure rotation
    REQUIRE_THAT(test_rotation.translation().x, Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(test_rotation.translation().y, Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(test_rotation.rotation(), Catch::Matchers::WithinAbs(3.14, 1e-5));

}

TEST_CASE("Translation and rotation", "[Transform]")
{
    turtlelib::Vector2D vector{1.2, 3.4};
    double radians{5.6};
    turtlelib::Transform2D test_transform(vector, radians);

    REQUIRE_THAT(test_transform.translation().x, Catch::Matchers::WithinAbs(1.2, 1e-5));
    REQUIRE_THAT(test_transform.translation().y, Catch::Matchers::WithinAbs(3.4, 1e-5));
    REQUIRE_THAT(test_transform.rotation(), Catch::Matchers::WithinAbs(5.6, 1e-5));

}

TEST_CASE("Applying a transformation (operator()) to a 2D point.", "[Transform]")
{
    // Define the original point
    turtlelib::Point2D original_point{2.0, 4.5};

    // Define the transformation
    turtlelib::Transform2D transform_test(turtlelib::Vector2D{1.0, -2.6}, -1.57079632);

    // Apply the transformation
    turtlelib::Point2D transformed_point = transform_test(original_point);

    // Define the expected result point
    turtlelib::Point2D expected_point{5.5, -4.6};

    REQUIRE_THAT(transformed_point.x, Catch::Matchers::WithinAbs(expected_point.x, 1e-5));
    REQUIRE_THAT(transformed_point.y, Catch::Matchers::WithinAbs(expected_point.y, 1e-5));

}




























TEST_CASE("Applying a transformation (operator()) to a 2D vector", "[Transform]")
{
    // Define the original vector to be transformed
    turtlelib::Vector2D original_vector{1.0, 1.0};

    // Define the transformation
    turtlelib::Transform2D transform_test(turtlelib::Vector2D{3.0, 4.0}, 1.57079632);

    // Apply the transformation to the vector
    turtlelib::Vector2D transformed_vector = transform_test(original_vector);

    // Define the expected result vector
    turtlelib::Vector2D expected_vector{2.0, 5.0};

    REQUIRE_THAT(transformed_vector.x, Catch::Matchers::WithinAbs(expected_vector.x, 1e-3));
    REQUIRE_THAT(transformed_vector.y, Catch::Matchers::WithinAbs(expected_vector.y, 1e-3));
}


// Next step is to make a test for the Point2D transformation. make it above the Vector2D one.
// but before that, make make a harder test case (non-round numbers) for the above test case