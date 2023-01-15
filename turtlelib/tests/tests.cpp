#include <catch2/catch_test_macros.hpp>
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
 
   REQUIRE((T_ab_1*=T_bc).translation().x == 4.0);
   REQUIRE((T_ab_2*=T_bc).translation().y == 6.0);
   REQUIRE((T_ab_3*=T_bc).rotation() == (turtlelib::PI/2));
}

