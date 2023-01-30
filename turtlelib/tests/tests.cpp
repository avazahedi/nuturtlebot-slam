#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/rigid2d.hpp"
#include <iostream>
#include <sstream>

// Contributors: Ava Zahedi, Katie Hughes, Hang Yin, Megan Sindelar

// My test cases
TEST_CASE( "Translation", "[transform]" ) // Ava, Zahedi
{
    turtlelib::Vector2D vec;
    vec.x = 1.0;
    vec.y = 3.4;
    turtlelib::Transform2D tf = turtlelib::Transform2D(vec);
    REQUIRE( turtlelib::almost_equal(tf.translation().x, 1.0, 0.00001) );
    REQUIRE( turtlelib::almost_equal(tf.translation().y, 3.4, 0.00001) );
}

TEST_CASE( "Rotation", "[transform]" ) // Ava, Zahedi
{
    double phi = 90;
    turtlelib::Transform2D tf = turtlelib::Transform2D(phi);
    REQUIRE( turtlelib::almost_equal(tf.rotation(), phi, 0.00001) );
}

TEST_CASE( "Stream insertion operator <<", "[transform]" ) // Ava, Zahedi
{
    turtlelib::Vector2D vec;
    vec.x = 1.0;
    vec.y = 3.4;
    double phi = 0.0;
    turtlelib::Transform2D tf = turtlelib::Transform2D(vec, phi);
    std::string str = "deg: 0 x: 1 y: 3.4";
    std::stringstream sstr;
    sstr << tf;
    REQUIRE( sstr.str() == str );
}

TEST_CASE( "Stream extraction operator >>", "[transform]" ) // Ava, Zahedi
{
    turtlelib::Transform2D tf = turtlelib::Transform2D();
    std::stringstream sstr;
    sstr << "deg: 90 x: 1 y: 3.4";
    sstr >> tf;
    REQUIRE( turtlelib::almost_equal(tf.rotation(), turtlelib::deg2rad(90), 0.00001) );
    REQUIRE( turtlelib::almost_equal(tf.translation().x, 1.0, 0.00001) );
    REQUIRE( turtlelib::almost_equal(tf.translation().y, 3.4, 0.00001) );
}

// Classmates' test cases
TEST_CASE( "Inverse", "[transform]" ) { // Katie, Hughes
   float my_x = 0.;
   float my_y = 1.;
   float my_ang = turtlelib::PI/2;
   turtlelib::Transform2D Ttest = {turtlelib::Vector2D{my_x,my_y}, my_ang};
   turtlelib::Transform2D Ttest_inv = Ttest.inv();
   REQUIRE( (Ttest.inv()).rotation() == -my_ang);
   REQUIRE( turtlelib::almost_equal(Ttest_inv.translation().x, -1.0, 1.0e-5) );
   REQUIRE( turtlelib::almost_equal(Ttest_inv.translation().y,  0.0, 1.0e-5) );
}

TEST_CASE( "Operator () for Vector2D", "[transform]" ) { // Hang, Yin
   float my_x = 2;
   float my_y = 3;
   float my_ang = turtlelib::PI;
   turtlelib::Transform2D Ttest = {turtlelib::Vector2D{my_x,my_y}, my_ang};
   turtlelib::Vector2D v = {2,2};
   turtlelib::Vector2D result = Ttest(v);
   REQUIRE( turtlelib::almost_equal(result.x, 0.0, 1.0e-5) );
   REQUIRE( turtlelib::almost_equal(result.y, 1.0, 1.0e-5) );
}
 
TEST_CASE( "Operator () for Twist2D", "[transform]" ) { // Hang, Yin
   float my_x = 2;
   float my_y = 3;
   float my_ang = turtlelib::PI;
   turtlelib::Transform2D Ttest = {turtlelib::Vector2D{my_x,my_y}, my_ang};
   turtlelib::Twist2D twist = {2,2,2};
   turtlelib::Twist2D result = Ttest(twist);
   REQUIRE( turtlelib::almost_equal(result.w, 2.0, 1.0e-5) );
   REQUIRE( turtlelib::almost_equal(result.x, 4.0, 1.0e-5) );
   REQUIRE( turtlelib::almost_equal(result.y, -6.0, 1.0e-5) );
}

TEST_CASE("operator *=", "[transform]"){    //Megan, Sindelar
   turtlelib::Vector2D trans_ab = {1,2};
   double rotate_ab = 0;
   turtlelib::Transform2D T_ab_1 = {trans_ab, rotate_ab};      //T_ab's are all the same,
   turtlelib::Transform2D T_ab_2 = {trans_ab, rotate_ab};      //but, need different vars
   turtlelib::Transform2D T_ab_3 = {trans_ab, rotate_ab};      //b/c getting overwritten otherwise
   turtlelib::Vector2D trans_bc = {3,4};
   double rotate_bc = turtlelib::PI/2;
   turtlelib::Transform2D T_bc = {trans_bc, rotate_bc};
 
   REQUIRE(turtlelib::almost_equal((T_ab_1*=T_bc).translation().x, 4.0));
   REQUIRE(turtlelib::almost_equal((T_ab_2*=T_bc).translation().y, 6.0));
   REQUIRE(turtlelib::almost_equal((T_ab_3*=T_bc).rotation(), (turtlelib::PI/2)));
}

TEST_CASE("Normalize angle"){   // Ava, Zahedi
    double pi = turtlelib::PI;
    REQUIRE_THAT( turtlelib::normalize_angle(pi), Catch::Matchers::WithinAbs(pi, 1e-5));
    REQUIRE_THAT( turtlelib::normalize_angle(-pi), Catch::Matchers::WithinAbs(pi, 1e-5));
    REQUIRE_THAT( turtlelib::normalize_angle(0), Catch::Matchers::WithinAbs(0, 1e-5));
    REQUIRE_THAT( turtlelib::normalize_angle(-pi/4), Catch::Matchers::WithinAbs(-pi/4, 1e-5));
    REQUIRE_THAT( turtlelib::normalize_angle(3*pi/2), Catch::Matchers::WithinAbs(-pi/2, 1e-5));
    REQUIRE_THAT( turtlelib::normalize_angle(-5*pi/2), Catch::Matchers::WithinAbs(-pi/2, 1e-5));
}

TEST_CASE("Vector addition"){   // Ava, Zahedi
    turtlelib::Vector2D v1, v2, sum;
    v1.x = 1;
    v1.y = 1.2;
    v2.x = -4.5;
    v2.y = 0.1;
    sum = v1+v2;
    REQUIRE_THAT( sum.x, Catch::Matchers::WithinAbs(-3.5, 1e-5));
    REQUIRE_THAT( sum.y, Catch::Matchers::WithinAbs(1.3, 1e-5));
}

TEST_CASE("Vector subtraction"){   // Ava, Zahedi
    turtlelib::Vector2D v1, v2, diff;
    v1.x = 1;
    v1.y = 1.2;
    v2.x = -4.5;
    v2.y = 0.1;
    diff = v1-v2;
    REQUIRE_THAT( diff.x, Catch::Matchers::WithinAbs(5.5, 1e-5));
    REQUIRE_THAT( diff.y, Catch::Matchers::WithinAbs(1.1, 1e-5));
}

TEST_CASE("Scalar multiplication with a vector"){   // Ava, Zahedi
    turtlelib::Vector2D v1, prod1, prod2;
    double s;
    v1.x = 1;
    v1.y = 1.2;
    s = 2.5;
    prod1 = v1*s;
    prod2 = s*v1;
    REQUIRE_THAT( prod1.x, Catch::Matchers::WithinAbs(2.5, 1e-5));
    REQUIRE_THAT( prod1.y, Catch::Matchers::WithinAbs(3, 1e-5));
    REQUIRE_THAT( prod2.x, Catch::Matchers::WithinAbs(2.5, 1e-5));
    REQUIRE_THAT( prod2.y, Catch::Matchers::WithinAbs(3, 1e-5));
}

TEST_CASE("Dot product of two vectors"){   // Ava, Zahedi
    turtlelib::Vector2D v1, v2;
    double dp;
    v1.x = 1;
    v1.y = 1.2;
    v2.x = -4.5;
    v2.y = 0.1;
    dp = dot(v1, v2);
    REQUIRE_THAT( dp, Catch::Matchers::WithinAbs(-4.38, 1e-5));
}

TEST_CASE("Vector magnitude"){   // Ava, Zahedi
    turtlelib::Vector2D v;
    double mag;
    v.x = 6.0;
    v.y = 8.0;
    mag = magnitude(v);
    REQUIRE_THAT( mag, Catch::Matchers::WithinAbs(10, 1e-5));
}

TEST_CASE("Angle between two vectors"){   // Ava, Zahedi
    turtlelib::Vector2D v1, v2;
    double ang;
    v1.x = 3;
    v1.y = -2;
    v2.x = 1;
    v2.y = 7;
    ang = angle(v1, v2);
    REQUIRE_THAT( ang, Catch::Matchers::WithinAbs(2.0169, 1e-5));
}