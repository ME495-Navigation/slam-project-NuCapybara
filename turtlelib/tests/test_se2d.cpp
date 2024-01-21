#include<iosfwd> // contains forward definitions for iostream objects
#include <iostream>
#include"turtlelib/se2d.hpp"
#include"turtlelib/geometry2d.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <string>
#include <sstream>
#include "turtlelib/geometry2d.hpp"


namespace turtlelib
{

    TEST_CASE("Empty Constructor", "[transform]")
    { //Stella
        Transform2D Tran;
        REQUIRE(Tran.translation().x == 0.0);
        REQUIRE(Tran.translation().y == 0.0);
        REQUIRE(Tran.rotation() == 0.0);
    }


    TEST_CASE("Translation Only Constructor", "[transform]")
    { // Stella
        double my_x = 5;
        double my_y = 6;
        Transform2D T(Vector2D{my_x, my_y});
        REQUIRE(T.rotation() == 0.0);
        REQUIRE(T.translation().x == my_x);
        REQUIRE(T.translation().y == my_y);
    }

    TEST_CASE("Rotation Only Constructor", "[transform]")
    { // Hughes, Katie
        double ang = PI/4;
        Transform2D T(ang);
        REQUIRE(T.rotation() == ang);
        REQUIRE(T.translation().x == 0.0);
        REQUIRE(T.translation().y == 0.0);
    }

    TEST_CASE("Rotation and Translation", "[transform]")
    { // Hughes, Katie
        double x = 3.0;
        double y = 5.0;
        double ang = 6.2876;
        Transform2D T(Vector2D{x, y}, ang);
        REQUIRE(T.rotation() == ang);
        REQUIRE(T.translation().x == x);
        REQUIRE(T.translation().y == y);
    }

    TEST_CASE("operator()(Vector2D v)", "[transform]") // Marno, Nel
    {
        double test_rot = PI / 2.0;
        double test_x = 0.0;
        double test_y = 1.0;
        Transform2D T_ab{{test_x, test_y}, test_rot};
        Vector2D v_b{1, 1};
        Vector2D v_a = T_ab(v_b);
        REQUIRE_THAT(v_a.x, Catch::Matchers::WithinAbs(-1, 1e-5));
        REQUIRE_THAT(v_a.y, Catch::Matchers::WithinAbs(1, 1e-5));
    }

}