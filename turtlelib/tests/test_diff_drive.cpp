#include <catch2/catch_test_macros.hpp>
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <sstream>


TEST_CASE("Get wheels", "[DiffDrive]")
{
    turtlelib::DiffDrive robot;
    turtlelib::Wheels test_wheels = {2.24, 1.19};
    robot.set_wheels(test_wheels);

    turtlelib::Wheels actual_wheels = robot.get_wheels();

    REQUIRE_THAT(actual_wheels.phi_r, Catch::Matchers::WithinAbs(test_wheels.phi_r, 1e-5));
    REQUIRE_THAT(actual_wheels.phi_l, Catch::Matchers::WithinAbs(test_wheels.phi_l, 1e-5));

}

TEST_CASE("Forward Kinematics", "[DiffDrive]")
{
    SECTION("Robot drive forward")
    {
        // Initialize the DiffDrive robot
        // wheel_track, wheel_radius, wheels, q
        turtlelib::DiffDrive robot(0.16, 0.33, {0.0, 0.0}, {0.0, 0.0, 0.0});

        // Set the change in wheel position
        turtlelib::Wheels delta_wheels{turtlelib::PI/2, turtlelib::PI/2};

        // Update the robot config accordingly
        robot.do_forward_kinematics(delta_wheels);

        turtlelib::RobotConfig robot_config = robot.get_robot_config();
        // theta, x, y
        turtlelib::RobotConfig expected_config{0.0, 0.51836, 0.0};

        REQUIRE_THAT(robot_config.theta, Catch::Matchers::WithinAbs(expected_config.theta, 1e-5));
        REQUIRE_THAT(robot_config.x, Catch::Matchers::WithinAbs(expected_config.x, 1e-5));
        REQUIRE_THAT(robot_config.y, Catch::Matchers::WithinAbs(expected_config.y, 1e-5));

    }

    SECTION("Robot pure rotation (90 degrees CCW)")
    {
        // Initialize the DiffDrive robot
        // wheel_track, wheel_radius, wheels, q
        turtlelib::DiffDrive robot(2.0, 1.0, {0.0, 0.0}, {0.0, 0.0, 0.0});

        // Set the change in wheel position
        turtlelib::Wheels delta_wheels{-turtlelib::PI, turtlelib::PI};

        // Update the robot config accordingly
        robot.do_forward_kinematics(delta_wheels);

        turtlelib::RobotConfig robot_config = robot.get_robot_config();
        // theta, x, y
        turtlelib::RobotConfig expected_config{turtlelib::PI, 0.0, 0.0};

        REQUIRE_THAT(robot_config.theta, Catch::Matchers::WithinAbs(expected_config.theta, 1e-5));
        REQUIRE_THAT(robot_config.x, Catch::Matchers::WithinAbs(expected_config.x, 1e-5));
        REQUIRE_THAT(robot_config.y, Catch::Matchers::WithinAbs(expected_config.y, 1e-5));

    }

    SECTION("Robot follows arc")
    {
        // Initialize the DiffDrive robot
        // wheel_track, wheel_radius, wheels, q
        turtlelib::DiffDrive robot(2.0, 1.0, {0.0, 0.0}, {0.0, 0.0, 0.0});

        // Set the change in wheel position
        turtlelib::Wheels delta_wheels{1.5*turtlelib::PI, turtlelib::PI};

        // Update the robot config accordingly
        robot.do_forward_kinematics(delta_wheels);

        turtlelib::RobotConfig robot_config = robot.get_robot_config();
        // theta, x, y
        turtlelib::RobotConfig expected_config{-turtlelib::PI/4, 3.535533, -1.464466};

        REQUIRE_THAT(robot_config.theta, Catch::Matchers::WithinAbs(expected_config.theta, 1e-5));
        REQUIRE_THAT(robot_config.x, Catch::Matchers::WithinAbs(expected_config.x, 1e-5));
        REQUIRE_THAT(robot_config.y, Catch::Matchers::WithinAbs(expected_config.y, 1e-5));

    }

    
}