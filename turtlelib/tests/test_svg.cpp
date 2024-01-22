#include <catch2/catch_test_macros.hpp>
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/svg.hpp"
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <sstream>
#include <fstream>

TEST_CASE("Svg::DrawVector generates correct SVG output", "[Svg]") 
{
    SECTION("Test Point")
    {
        // File path
        std::string filepath = "../tmp/test_frames.svg";

        // Create an instance of the Svg class and draw to a file
        {
            std::ofstream svg_file(filepath);
            turtlelib::Svg mySvg;

            if (svg_file.is_open()) {
                mySvg.DrawPoint(0, 0, "red", svg_file);
            } else {
                FAIL("Could not open file for writing");
            }
        } // Ensure the file is closed by going out of scope

        // Now read from the file
        std::ifstream svg_file_read(filepath);
        std::string firstLine;
        if (svg_file_read.is_open()) {
            std::getline(svg_file_read, firstLine);
        } else {
            FAIL("Could not open file for reading");
        }

        std::string expected_first_line = "<circle cx=\"408\" cy=\"528\" r=\"3\" stroke=\"red\" fill=\"red\" stroke-width=\"1\" />";
        REQUIRE(firstLine == expected_first_line);
    }
    
    SECTION("Test Vector")
    {
        // File path
        std::string filepath = "../tmp/test_frames.svg";

        // Create an instance of the Svg class and draw to a file
        {
            std::ofstream svg_file(filepath);
            turtlelib::Svg mySvg;

            if (svg_file.is_open()) {
                mySvg.DrawVector(turtlelib::Point2D{-0.5, 2}, turtlelib::Vector2D{1, -3}, "blue", svg_file);
            } else {
                FAIL("Could not open file for writing");
            }
        } // Ensure the file is closed by going out of scope

        // Now read from the file
        std::ifstream svg_file_read(filepath);
        std::string firstLine;
        if (svg_file_read.is_open()) {
            std::getline(svg_file_read, firstLine);
        } else {
            FAIL("Could not open file for reading");
        }

        std::string expected_first_line = "<line x1=\"456\" x2=\"360\" y1=\"624\" y2=\"336\" stroke=\"blue\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />";
        REQUIRE(firstLine == expected_first_line);
    }

    SECTION("Test Coordinate Frame")
    {
        std::string filepath = "../tmp/test_frames.svg";

        // Create an instance of the Svg class and create coordinate frame
        {
            std::ofstream svg_file(filepath);
            turtlelib::Svg mySvg;

            if (svg_file.is_open()) {
                mySvg.DrawCoordinateFrame(turtlelib::Point2D{-1, 3}, turtlelib::Vector2D{1,0}, "T_ab", svg_file);
            } else {
                FAIL("Could not open file for writing");
            }
        } // Ensure the file is closed by going out of scope

        std::ifstream svg_file_read(filepath);
        std::string line1, line2, line3, line4, line5;
        if (svg_file_read.is_open()) {
            // Read the first five lines and store them in separate variables
            std::getline(svg_file_read, line1);
            std::getline(svg_file_read, line2);
            std::getline(svg_file_read, line3);
            std::getline(svg_file_read, line4);
            std::getline(svg_file_read, line5);
        } else {
            FAIL("Could not open file for reading");
        }

        std::string expected_line1 = "<g>";
        std::string expected_line2 = "<line x1=\"408\" x2=\"312\" y1=\"240\" y2=\"240\" stroke=\"red\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />";
        std::string expected_line3 = "<line x1=\"312\" x2=\"312\" y1=\"144\" y2=\"240\" stroke=\"green\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\" />";
        std::string expected_line4 = "<text x=\"312\" y=\"240\">{T_ab}</text>";
        std::string expected_line5 = "</g>";

        REQUIRE(line1 == expected_line1);
        REQUIRE(line2 == expected_line2);
        REQUIRE(line3 == expected_line3);
        REQUIRE(line4 == expected_line4);
        REQUIRE(line5 == expected_line5);
    }
}
