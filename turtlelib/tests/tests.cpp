#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/circle_fit.hpp"
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
    double theta = -90;
    turtlelib::Transform2D tf2 = turtlelib::Transform2D(theta);
    REQUIRE( turtlelib::almost_equal(tf.rotation(), phi, 0.00001) );
    REQUIRE( turtlelib::almost_equal(tf2.rotation(), theta, 0.00001) );
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

// My test cases
TEST_CASE("Normalize angle"){
    double pi = turtlelib::PI;
    REQUIRE_THAT( turtlelib::normalize_angle(pi), Catch::Matchers::WithinAbs(pi, 1e-5));
    REQUIRE_THAT( turtlelib::normalize_angle(-pi), Catch::Matchers::WithinAbs(pi, 1e-5));
    REQUIRE_THAT( turtlelib::normalize_angle(0), Catch::Matchers::WithinAbs(0, 1e-5));
    REQUIRE_THAT( turtlelib::normalize_angle(-pi/4), Catch::Matchers::WithinAbs(-pi/4, 1e-5));
    REQUIRE_THAT( turtlelib::normalize_angle(3*pi/2), Catch::Matchers::WithinAbs(-pi/2, 1e-5));
    REQUIRE_THAT( turtlelib::normalize_angle(-5*pi/2), Catch::Matchers::WithinAbs(-pi/2, 1e-5));
}

TEST_CASE("Vector addition"){
    turtlelib::Vector2D v1, v2, sum;
    v1.x = 1;
    v1.y = 1.2;
    v2.x = -4.5;
    v2.y = 0.1;
    sum = v1+v2;
    REQUIRE_THAT( sum.x, Catch::Matchers::WithinAbs(-3.5, 1e-5));
    REQUIRE_THAT( sum.y, Catch::Matchers::WithinAbs(1.3, 1e-5));
}

TEST_CASE("Vector subtraction"){
    turtlelib::Vector2D v1, v2, diff;
    v1.x = 1;
    v1.y = 1.2;
    v2.x = -4.5;
    v2.y = 0.1;
    diff = v1-v2;
    REQUIRE_THAT( diff.x, Catch::Matchers::WithinAbs(5.5, 1e-5));
    REQUIRE_THAT( diff.y, Catch::Matchers::WithinAbs(1.1, 1e-5));
}

TEST_CASE("Scalar multiplication with a vector"){
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

TEST_CASE("Dot product of two vectors"){
    turtlelib::Vector2D v1, v2;
    double dp;
    v1.x = 1;
    v1.y = 1.2;
    v2.x = -4.5;
    v2.y = 0.1;
    dp = dot(v1, v2);
    REQUIRE_THAT( dp, Catch::Matchers::WithinAbs(-4.38, 1e-5));
}

TEST_CASE("Vector magnitude"){
    turtlelib::Vector2D v;
    double mag;
    v.x = 6.0;
    v.y = 8.0;
    mag = magnitude(v);
    REQUIRE_THAT( mag, Catch::Matchers::WithinAbs(10, 1e-5));
}

TEST_CASE("Angle between two vectors"){ 
    turtlelib::Vector2D v1, v2;
    double ang;
    v1.x = 3;
    v1.y = -2;
    v2.x = 1;
    v2.y = 7;
    ang = angle(v1, v2);
    REQUIRE_THAT( ang, Catch::Matchers::WithinAbs(2.0169, 1e-5));
}

TEST_CASE("Integrate twist pure translation") {
    turtlelib::Twist2D t;
    t.w = 0;
    t.x = 1.2;
    t.y = -3;
    turtlelib::Transform2D tf = turtlelib::integrate_twist(t);
    REQUIRE_THAT( tf.rotation(), Catch::Matchers::WithinAbs(0, 1e-5));
    REQUIRE_THAT( tf.translation().x, Catch::Matchers::WithinAbs(1.2, 1e-5)); 
    REQUIRE_THAT( tf.translation().y, Catch::Matchers::WithinAbs(-3, 1e-5));
}

TEST_CASE("Integrate twist pure rotation") {
    turtlelib::Twist2D t;
    t.w = 2.1;
    t.x = 0;
    t.y = 0;
    turtlelib::Transform2D tf = turtlelib::integrate_twist(t);
    REQUIRE_THAT( tf.rotation(), Catch::Matchers::WithinAbs(2.1, 1e-5));
    REQUIRE_THAT( tf.translation().x, Catch::Matchers::WithinAbs(0, 1e-5)); 
    REQUIRE_THAT( tf.translation().y, Catch::Matchers::WithinAbs(0, 1e-5));
}

TEST_CASE("Integrate twist simultaneous translation and rotation") {
    turtlelib::Twist2D t;
    t.w = -2.1;
    t.x = 1.2;
    t.y = -3;
    turtlelib::Transform2D tf = turtlelib::integrate_twist(t);
    REQUIRE_THAT( tf.rotation(), Catch::Matchers::WithinAbs(-2.1, 1e-5));
    REQUIRE_THAT( tf.translation().x, Catch::Matchers::WithinAbs(-1.65651765, 1e-5)); 
    REQUIRE_THAT( tf.translation().y, Catch::Matchers::WithinAbs(-2.09306829, 1e-5));
}

TEST_CASE("IK - forward", "[diffdrive]") {
    double track = 2.0;
    double radius = 1.0;

    turtlelib::Twist2D Vb;
    Vb.w = 0;
    Vb.x = 1;
    Vb.y = 0;

    turtlelib::DiffDrive dd {track, radius};
    turtlelib::WheelPosn ik_wheels = dd.InverseKinematics(Vb);
    REQUIRE_THAT( ik_wheels.left, Catch::Matchers::WithinAbs(1.0, 1e-5));
    REQUIRE_THAT( ik_wheels.right, Catch::Matchers::WithinAbs(1.0, 1e-5));
}

TEST_CASE("FK - forward", "[diffdrive]") {
    turtlelib::WheelPosn wheels;
    wheels.left = 1;
    wheels.right = 1;
    double track = 2;
    double radius = 1;

    turtlelib::DiffDrive dd = turtlelib::DiffDrive(track, radius);
    dd.ForwardKinematics(wheels);
    turtlelib::RobotConfig q_new = dd.getConfig();
    REQUIRE_THAT( q_new.theta, Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT( q_new.x, Catch::Matchers::WithinAbs(1.0, 1e-5));
    REQUIRE_THAT( q_new.y, Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("IK/FK - backward", "[diffdrive]") {
    double track = 2.0;
    double radius = 1.0;

    turtlelib::Twist2D Vb;
    Vb.w = 0;
    Vb.x = -1;
    Vb.y = 0;

    turtlelib::DiffDrive dd {track, radius};
    turtlelib::WheelPosn ik_wheels = dd.InverseKinematics(Vb);
    REQUIRE_THAT( ik_wheels.left, Catch::Matchers::WithinAbs(-1.0, 1e-5));
    REQUIRE_THAT( ik_wheels.right, Catch::Matchers::WithinAbs(-1.0, 1e-5));


    turtlelib::DiffDrive dd2 {track, radius};
    dd2.ForwardKinematics(ik_wheels);
    turtlelib::RobotConfig q_new = dd2.getConfig();
    REQUIRE_THAT( q_new.theta, Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT( q_new.x, Catch::Matchers::WithinAbs(-1.0, 1e-5));
    REQUIRE_THAT( q_new.y, Catch::Matchers::WithinAbs(0.0, 1e-5));
}

TEST_CASE("IK/FK - pure rotation CCW", "[diffdrive]") {
    double track = 2.0;
    double radius = 1.0;

    turtlelib::Twist2D Vb;
    Vb.w = 1;
    Vb.x = 0;
    Vb.y = 0;

    turtlelib::DiffDrive dd {track, radius};
    turtlelib::WheelPosn ik_wheels = dd.InverseKinematics(Vb);
    REQUIRE_THAT( ik_wheels.left, Catch::Matchers::WithinAbs(-1.0, 1e-5));
    REQUIRE_THAT( ik_wheels.right, Catch::Matchers::WithinAbs(1.0, 1e-5));

    turtlelib::DiffDrive dd2 {track, radius};
    dd2.ForwardKinematics(ik_wheels);
    turtlelib::RobotConfig q_new = dd2.getConfig();
    REQUIRE_THAT( q_new.theta, Catch::Matchers::WithinAbs(1, 1e-5));
    REQUIRE_THAT( q_new.x, Catch::Matchers::WithinAbs(0, 1e-5));
    REQUIRE_THAT( q_new.y, Catch::Matchers::WithinAbs(0, 1e-5));
}

TEST_CASE("IK/FK - pure rotation CW", "[diffdrive]") {
    double track = 2.0;
    double radius = 1.0;

    turtlelib::Twist2D Vb;
    Vb.w = -1;
    Vb.x = 0;
    Vb.y = 0;

    turtlelib::DiffDrive dd {track, radius};
    turtlelib::WheelPosn ik_wheels = dd.InverseKinematics(Vb);
    REQUIRE_THAT( ik_wheels.left, Catch::Matchers::WithinAbs(1, 1e-5));
    REQUIRE_THAT( ik_wheels.right, Catch::Matchers::WithinAbs(-1, 1e-5));

    turtlelib::DiffDrive dd2 {track, radius};
    dd2.ForwardKinematics(ik_wheels);
    turtlelib::RobotConfig q_new = dd2.getConfig();
    REQUIRE_THAT( q_new.theta, Catch::Matchers::WithinAbs(-1, 1e-5));
    REQUIRE_THAT( q_new.x, Catch::Matchers::WithinAbs(0, 1e-5));
    REQUIRE_THAT( q_new.y, Catch::Matchers::WithinAbs(0, 1e-5));
}

TEST_CASE("IK/FK - following an arc (rotation & translation)", "[diffdrive]") {
    double track = 2.0;
    double radius = 1.0;

    turtlelib::Twist2D Vb;
    Vb.w = 1;
    Vb.x = 1;
    Vb.y = 0;

    turtlelib::DiffDrive dd {track, radius};
    turtlelib::WheelPosn ik_wheels = dd.InverseKinematics(Vb);
    REQUIRE_THAT( ik_wheels.left, Catch::Matchers::WithinAbs(0, 1e-5));
    REQUIRE_THAT( ik_wheels.right, Catch::Matchers::WithinAbs(2, 1e-5));

    turtlelib::DiffDrive dd2 {track, radius};
    dd2.ForwardKinematics(ik_wheels);
    turtlelib::RobotConfig q_new = dd2.getConfig();
    REQUIRE_THAT( q_new.theta, Catch::Matchers::WithinAbs(1, 1e-5));
    REQUIRE_THAT( q_new.x, Catch::Matchers::WithinAbs(0.8414709848, 1e-5));
    REQUIRE_THAT( q_new.y, Catch::Matchers::WithinAbs(0.4596976941, 1e-5));
}

TEST_CASE("IK - wheels slipping", "[diffdrive]") {
    double track = 2.0;
    double radius = 1.0;

    turtlelib::Twist2D Vb;
    Vb.w = 1;
    Vb.x = 1;
    Vb.y = 1;

    turtlelib::DiffDrive dd {track, radius};
    REQUIRE_THROWS_AS(dd.InverseKinematics(Vb), std::logic_error);
}

// Circle detection tests
TEST_CASE("Circle detection test 1", "[circle_fit]") {
    std::vector<turtlelib::Vector2D> cluster{};
    cluster.push_back(turtlelib::Vector2D{1,7});
    cluster.push_back(turtlelib::Vector2D{2,6});
    cluster.push_back(turtlelib::Vector2D{5,8});
    cluster.push_back(turtlelib::Vector2D{7,7});
    cluster.push_back(turtlelib::Vector2D{9,5});
    cluster.push_back(turtlelib::Vector2D{3,7});

    turtlelib::Vector2D center {4.615482, 2.807354};
    float radius = 4.8275;
    turtlelib::Circle circle = turtlelib::circle_fit(cluster);

    REQUIRE_THAT( circle.center.x, Catch::Matchers::WithinAbs(center.x, 1e-4));
    REQUIRE_THAT( circle.center.y, Catch::Matchers::WithinAbs(center.y, 1e-4));
    REQUIRE_THAT( circle.radius, Catch::Matchers::WithinAbs(radius, 1e-4));
}

TEST_CASE("Circle detection test 2", "[circle_fit]") {
    std::vector<turtlelib::Vector2D> cluster{};
    cluster.push_back(turtlelib::Vector2D{-1,0});
    cluster.push_back(turtlelib::Vector2D{-0.3,-0.06});
    cluster.push_back(turtlelib::Vector2D{0.3,0.1});
    cluster.push_back(turtlelib::Vector2D{1,0});

    turtlelib::Vector2D center {0.4908357, -22.15212};
    float radius = 22.17979;
    turtlelib::Circle circle = turtlelib::circle_fit(cluster);
    REQUIRE_THAT( circle.center.x, Catch::Matchers::WithinAbs(center.x, 1e-4));
    REQUIRE_THAT( circle.center.y, Catch::Matchers::WithinAbs(center.y, 1e-4));
    REQUIRE_THAT( circle.radius, Catch::Matchers::WithinAbs(radius, 1e-4));
}