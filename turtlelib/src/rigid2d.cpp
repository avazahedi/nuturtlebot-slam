/// \file
/// \brief Two-dimensional rigid body transformations.

#include <iostream>
#include "turtlelib/rigid2d.hpp"
#include <cmath>

namespace turtlelib {

    // Vector2D
    std::ostream & operator<<(std::ostream & os, const Vector2D & v)
    {
        os << "[" << v.x << " " << v.y << "]";
        return os;
    }

    std::istream & operator>>(std::istream & is, Vector2D & v)
    {
        char c = is.peek();
        if (c == '[') {
            is.get();
        }

        is >> v.x >> v.y;
        
        is.clear();
        is.ignore(100, '\n');
        return is;
    }

    // Twist2D
    std::ostream & operator<<(std::ostream & os, const Twist2D & t)
    {
        os << "[" << t.w << " " << t.x << " " << t.y << "]";
        return os;
    }

    std::istream & operator>>(std::istream & is, Twist2D & t)
    {
        char c = is.peek();
        if (c == '[') {
            is.get();
        }

        is >> t.w >> t.x >> t.y;

        is.clear();
        is.ignore(100, '\n');
        return is;
    }

    // Transform2D
    Transform2D::Transform2D(): tvec{0.0, 0.0}, phi{0.0} {}

    Transform2D::Transform2D(Vector2D trans): tvec{trans.x, trans.y}, phi{0.0} {}

    Transform2D::Transform2D(double radians): tvec{0.0, 0.0}, phi{radians} {}

    Transform2D::Transform2D(Vector2D trans, double radians): tvec{trans.x, trans.y}, phi{radians} {}

    Vector2D Transform2D::operator()(Vector2D v) const
    {
        //! No need to create a temporary vector here.
        //! Can return {vector x componet, vector y component directly}
        Vector2D new_v;
        new_v.x = cos(phi)*v.x - sin(phi)*v.y + tvec.x;
        new_v.y = sin(phi)*v.x + cos(phi)*v.y + tvec.y;

        return new_v;
    }

    Transform2D Transform2D::inv() const
    {
        //! You should be using the Transform2D constructor to create
        //! The new Transform2D here. Otherwise you are bypassing the
        //! constructor and could theoretically initialize incorrectly
        //! return {}; what goes in the {} is arguments to the Transform2D constructor
        Transform2D inverse;
        inverse.phi = -phi;
        inverse.tvec.x = -tvec.x*cos(phi)-tvec.y*sin(phi);
        inverse.tvec.y = -tvec.y*cos(phi)+tvec.x*sin(phi);

        return inverse;
    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs)
    {
        //! Should not create this temporary Transform2D and bypass the constructor
        //! Here you are essentially using tmp as temporary storage for 3 temporary doubles
        //! If you need a temporary use a temporary variable since you don't actually
        //! use tmp as a Transform2D 
        Transform2D tmp;
        tmp.phi = phi;
        tmp.tvec.x = tvec.x;
        tmp.tvec.y = tvec.y;

        phi = acos(cos(tmp.phi)*cos(rhs.phi) - sin(tmp.phi)*sin(rhs.phi));
        tvec.x = cos(tmp.phi)*rhs.tvec.x - sin(tmp.phi)*rhs.tvec.y + tmp.tvec.x;
        tvec.y = sin(tmp.phi)*rhs.tvec.x + cos(tmp.phi)*rhs.tvec.y + tmp.tvec.y;

        return *this;
    }

    Vector2D Transform2D::translation() const
    {
        //! No need for a temporary, can return {tvec.x, tvec.y};
        Vector2D trans;
        trans.x = tvec.x;
        trans.y = tvec.y;

        return trans;
    }

    double Transform2D::rotation() const
    {
        return phi;
    }

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf)
    {
        os << "deg: " << rad2deg(tf.phi) << " x: " << tf.tvec.x << " y: " << tf.tvec.y;
        return os;
    }

    std::istream & operator>>(std::istream & is, Transform2D & tf)
    {
        double deg; //! This double is unitialized which is dangerous
        Vector2D vec; //! Since vec has default =0 in its definition this is less dangerous, but Vector2D vec{} would ensure initialization

        if (is.peek() == 'd')
        {
            std::string s1;
            std::string s2;
            std::string s3;

            is >> s1 >> deg >> s2 >> vec.x >> s3 >> vec.y;
        }
        else
        {
            is >> deg >> vec.x >> vec.y;
        }

        //! const auto rad = deg2rad(deg);
        double rad = deg2rad(deg);

        //! const auto t = Transform2D(vec, rad)
        Transform2D t = Transform2D(vec, rad);

        tf = t;

        is.clear();
        is.ignore(100, '\n');
        return is;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
    {
        return lhs*=rhs;
    }

    Twist2D Transform2D::operator()(Twist2D t) const
    {
        //! No need for the temporary, return with {}
        Twist2D new_t;
        new_t.w = t.w;
        new_t.x = tvec.y*t.w + cos(phi)*t.x - sin(phi)*t.y;
        new_t.y = -tvec.x*t.w + sin(phi)*t.x + cos(phi)*t.y;

        return new_t;
    }

    Vector2D normalize(const Vector2D v)
    {
        //! no need for the temporary
        Vector2D norm;
        //! const auto magnitude =
        double magnitude = pow(pow(v.x, 2) + pow(v.y, 2), 0.5);
        norm.x = v.x / magnitude;
        norm.y = v.y / magnitude;

        return norm;
    }

}

//! Do not leave commented out debugging code in the final version

// int main(void) {
//     // std::cout << turtlelib::deg2rad(180) << "\n";
//     // struct turtlelib::Vector2D v;
//     // v.x = 1.4;
//     // v.y = 2.8;
//     // std::cout << v << std::endl;

//     // struct turtlelib::Vector2D w;
//     // std::cin >> w;
//     // std::cout << w << std::endl;

//     // struct turtlelib::Twist2D t;
//     // t.w = 0.9;
//     // t.x = 1.6;
//     // t.y = 1.2;
//     // std::cout << t << std::endl;

//     // struct turtlelib::Twist2D t1;
//     // std::cin >> t1;
//     // std::cout << t1 << std::endl;   

//     struct turtlelib::Vector2D v;
//     v.x = 0.;
//     v.y = 1.;

//     double rad = 3.14;
//     turtlelib::Transform2D tf = turtlelib::Transform2D(v, rad);

//     turtlelib::Transform2D x;
//     std::cin >> x;
//     std::cout << x << std::endl;

//     std::cout << tf*x << std::endl;

//     // turtlelib::Transform2D Tab(1.5708, v);

//     return 0;
// }
